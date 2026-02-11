#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include "../mavlink_connection.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>

namespace px4_bridge::tests
{
    /// Mock PX4 TCP server that accepts a connection and echoes MAVLink heartbeats
    class MockPX4Server
    {
    public:
        bool Start(uint16_t port)
        {
            m_serverSocket = ::socket(AF_INET, SOCK_STREAM, 0);
            if (m_serverSocket < 0) return false;

            int opt = 1;
            setsockopt(m_serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(port);
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

            if (::bind(m_serverSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            {
                ::close(m_serverSocket);
                return false;
            }

            ::listen(m_serverSocket, 1);
            m_port = port;
            return true;
        }

        int AcceptClient()
        {
            m_clientSocket = ::accept(m_serverSocket, nullptr, nullptr);
            return m_clientSocket;
        }

        void SendHeartbeat()
        {
            if (m_clientSocket < 0) return;

            mavlink_message_t msg{};
            mavlink_msg_heartbeat_pack(1, 1, &msg,
                MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4, 0, 0, 0);

            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            auto len = mavlink_msg_to_send_buffer(buf, &msg);
            ::send(m_clientSocket, buf, len, 0);
        }

        void SendActuatorControls(const float controls[16])
        {
            if (m_clientSocket < 0) return;

            mavlink_message_t msg{};
            mavlink_msg_hil_actuator_controls_pack(1, 1, &msg,
                0, controls, 0, 0);

            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            auto len = mavlink_msg_to_send_buffer(buf, &msg);
            ::send(m_clientSocket, buf, len, 0);
        }

        void Stop()
        {
            if (m_clientSocket >= 0) { ::close(m_clientSocket); m_clientSocket = -1; }
            if (m_serverSocket >= 0) { ::close(m_serverSocket); m_serverSocket = -1; }
        }

        ~MockPX4Server() { Stop(); }

    private:
        int m_serverSocket{-1};
        int m_clientSocket{-1};
        uint16_t m_port{0};
    };

    class MAVLinkConnectionTest : public UnitTest::LeakDetectionFixture
    {
    };

    TEST_F(MAVLinkConnectionTest, ConnectsToMockServer)
    {
        MockPX4Server server;
        ASSERT_TRUE(server.Start(14560));

        MAVLinkConnection conn;
        MAVLinkConnection::Config config{};
        config.host = "127.0.0.1";
        config.port = 14560;
        config.connect_timeout_ms = 2000;

        // Start connection in background
        ASSERT_TRUE(conn.Start(config));

        // Accept the connection on server side
        auto client = server.AcceptClient();
        EXPECT_GE(client, 0);

        // Give connection time to establish
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        EXPECT_TRUE(conn.IsConnected());

        conn.Stop();
        server.Stop();
    }

    TEST_F(MAVLinkConnectionTest, ReceivesHeartbeat)
    {
        MockPX4Server server;
        ASSERT_TRUE(server.Start(14561));

        std::atomic<bool> receivedHeartbeat{false};

        MAVLinkConnection conn;
        conn.SetMessageCallback([&](const mavlink_message_t& msg) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
            {
                receivedHeartbeat.store(true);
            }
        });

        MAVLinkConnection::Config config{};
        config.host = "127.0.0.1";
        config.port = 14561;
        ASSERT_TRUE(conn.Start(config));

        server.AcceptClient();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        server.SendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        EXPECT_TRUE(receivedHeartbeat.load());

        conn.Stop();
        server.Stop();
    }

    TEST_F(MAVLinkConnectionTest, SendsMessage)
    {
        MockPX4Server server;
        ASSERT_TRUE(server.Start(14562));

        MAVLinkConnection conn;
        MAVLinkConnection::Config config{};
        config.host = "127.0.0.1";
        config.port = 14562;
        ASSERT_TRUE(conn.Start(config));

        server.AcceptClient();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Send a heartbeat from client
        mavlink_message_t msg{};
        mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, 0);
        EXPECT_TRUE(conn.Send(msg));

        conn.Stop();
        server.Stop();
    }

    TEST_F(MAVLinkConnectionTest, ReportsDisconnectedWhenNotStarted)
    {
        MAVLinkConnection conn;
        EXPECT_FALSE(conn.IsConnected());
    }

} // namespace px4_bridge::tests
