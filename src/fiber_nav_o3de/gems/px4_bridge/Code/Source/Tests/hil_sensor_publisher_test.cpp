#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include "../hil_sensor_publisher.h"
#include "../mavlink_connection.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <memory>

namespace px4_bridge::tests
{
    /// Simple mock server that captures sent MAVLink messages
    class CaptureServer
    {
    public:
        bool Start(uint16_t port)
        {
            m_serverSocket = ::socket(AF_INET, SOCK_STREAM, 0);
            int opt = 1;
            setsockopt(m_serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

            if (::bind(m_serverSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
                return false;

            ::listen(m_serverSocket, 1);
            return true;
        }

        int AcceptClient()
        {
            m_clientSocket = ::accept(m_serverSocket, nullptr, nullptr);
            return m_clientSocket;
        }

        /// Read and parse MAVLink messages from the client, count by type
        void ReadMessages(int timeout_ms = 500)
        {
            timeval tv{};
            tv.tv_usec = timeout_ms * 1000;
            setsockopt(m_clientSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            uint8_t buf[4096];
            mavlink_status_t status{};

            auto start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - start).count() < timeout_ms)
            {
                auto n = ::recv(m_clientSocket, buf, sizeof(buf), 0);
                if (n <= 0) break;

                for (ssize_t i = 0; i < n; ++i)
                {
                    mavlink_message_t msg{};
                    if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status))
                    {
                        if (msg.msgid == MAVLINK_MSG_ID_HIL_SENSOR) ++m_hilSensorCount;
                        else if (msg.msgid == MAVLINK_MSG_ID_HIL_GPS) ++m_hilGPSCount;
                        else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) ++m_heartbeatCount;
                    }
                }
            }
        }

        int m_hilSensorCount{0};
        int m_hilGPSCount{0};
        int m_heartbeatCount{0};

        ~CaptureServer()
        {
            if (m_clientSocket >= 0) ::close(m_clientSocket);
            if (m_serverSocket >= 0) ::close(m_serverSocket);
        }

    private:
        int m_serverSocket{-1};
        int m_clientSocket{-1};
    };

    class HILSensorPublisherTest : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(HILSensorPublisherTest, SendsHILSensorMessage)
    {
        CaptureServer server;
        ASSERT_TRUE(server.Start(14563));

        auto conn = std::make_shared<MAVLinkConnection>();
        MAVLinkConnection::Config config{};
        config.host = "127.0.0.1";
        config.port = 14563;
        ASSERT_TRUE(conn->Start(config));

        server.AcceptClient();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        HILSensorPublisher pub;
        pub.Init(conn);

        SensorData data{};
        data.accel = AZ::Vector3(0.0f, 0.0f, -9.81f);
        data.gyro = AZ::Vector3::CreateZero();
        data.mag = AZ::Vector3(0.21f, 0.05f, -0.42f);
        data.abs_pressure = 1013.25f;
        data.temperature = 25.0f;

        pub.SendHILSensor(data, 1000000);

        server.ReadMessages(300);
        EXPECT_GE(server.m_hilSensorCount, 1);
        EXPECT_EQ(pub.GetSensorCount(), 1u);

        conn->Stop();
    }

    TEST_F(HILSensorPublisherTest, SendsHILGPSMessage)
    {
        CaptureServer server;
        ASSERT_TRUE(server.Start(14564));

        auto conn = std::make_shared<MAVLinkConnection>();
        MAVLinkConnection::Config config{};
        config.host = "127.0.0.1";
        config.port = 14564;
        ASSERT_TRUE(conn->Start(config));

        server.AcceptClient();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        HILSensorPublisher pub;
        pub.Init(conn);

        GPSData gps{};
        gps.lat = 311640930;   // 31.164093 * 1e7
        gps.lon = 345322270;   // 34.532227 * 1e7
        gps.alt = 141000;      // 141m in mm
        gps.fix_type = 3;
        gps.satellites_visible = 12;

        pub.SendHILGPS(gps, 1000000);

        server.ReadMessages(300);
        EXPECT_GE(server.m_hilGPSCount, 1);
        EXPECT_EQ(pub.GetGPSCount(), 1u);

        conn->Stop();
    }

    TEST_F(HILSensorPublisherTest, SendsHeartbeat)
    {
        CaptureServer server;
        ASSERT_TRUE(server.Start(14565));

        auto conn = std::make_shared<MAVLinkConnection>();
        MAVLinkConnection::Config config{};
        config.host = "127.0.0.1";
        config.port = 14565;
        ASSERT_TRUE(conn->Start(config));

        server.AcceptClient();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        HILSensorPublisher pub;
        pub.Init(conn);

        pub.SendHeartbeat();

        server.ReadMessages(300);
        EXPECT_GE(server.m_heartbeatCount, 1);

        conn->Stop();
    }

} // namespace px4_bridge::tests
