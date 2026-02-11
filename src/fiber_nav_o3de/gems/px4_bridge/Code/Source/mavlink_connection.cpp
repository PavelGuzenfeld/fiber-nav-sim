#include "mavlink_connection.h"

#include <AzCore/Console/ILogger.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <cstring>

namespace px4_bridge
{
    MAVLinkConnection::~MAVLinkConnection()
    {
        Stop();
    }

    bool MAVLinkConnection::Start(const Config& config)
    {
        if (m_running.load())
        {
            return false;
        }

        m_config = config;
        m_running.store(true);
        m_receiveThread = std::thread(&MAVLinkConnection::ReceiveThread, this);
        return true;
    }

    void MAVLinkConnection::Stop()
    {
        m_running.store(false);
        if (m_receiveThread.joinable())
        {
            m_receiveThread.join();
        }
        Disconnect();
    }

    bool MAVLinkConnection::Send(const mavlink_message_t& msg)
    {
        if (!m_connected.load(std::memory_order_relaxed))
        {
            return false;
        }

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        auto len = mavlink_msg_to_send_buffer(buffer, &msg);

        std::lock_guard lock(m_sendMutex);
        auto sent = ::send(m_socket, buffer, len, MSG_NOSIGNAL);
        if (sent < 0)
        {
            AZLOG_WARN("PX4Bridge: Send failed: %s", strerror(errno));
            Disconnect();
            return false;
        }
        return sent == static_cast<ssize_t>(len);
    }

    void MAVLinkConnection::SetMessageCallback(MessageCallback callback)
    {
        m_callback = AZStd::move(callback);
    }

    bool MAVLinkConnection::Connect()
    {
        m_socket = ::socket(AF_INET, SOCK_STREAM, 0);
        if (m_socket < 0)
        {
            AZLOG_ERROR("PX4Bridge: socket() failed: %s", strerror(errno));
            return false;
        }

        // Set non-blocking for connect with timeout
        int flags = fcntl(m_socket, F_GETFL, 0);
        fcntl(m_socket, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_config.port);
        inet_pton(AF_INET, m_config.host, &addr.sin_addr);

        int ret = ::connect(m_socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        if (ret < 0 && errno != EINPROGRESS)
        {
            ::close(m_socket);
            m_socket = -1;
            return false;
        }

        if (ret < 0)
        {
            // Wait for connect with timeout
            pollfd pfd{};
            pfd.fd = m_socket;
            pfd.events = POLLOUT;

            ret = ::poll(&pfd, 1, m_config.connect_timeout_ms);
            if (ret <= 0 || !(pfd.revents & POLLOUT))
            {
                ::close(m_socket);
                m_socket = -1;
                return false;
            }

            // Check for connection error
            int err = 0;
            socklen_t errlen = sizeof(err);
            getsockopt(m_socket, SOL_SOCKET, SO_ERROR, &err, &errlen);
            if (err != 0)
            {
                ::close(m_socket);
                m_socket = -1;
                return false;
            }
        }

        // Set back to blocking with TCP_NODELAY
        fcntl(m_socket, F_SETFL, flags);
        int nodelay = 1;
        setsockopt(m_socket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        // Set receive timeout (100ms — don't block forever on recv)
        timeval tv{};
        tv.tv_usec = 100000;
        setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        m_connected.store(true, std::memory_order_release);
        AZLOG_INFO("PX4Bridge: Connected to PX4 at %s:%u", m_config.host, m_config.port);
        return true;
    }

    void MAVLinkConnection::Disconnect()
    {
        if (m_socket >= 0)
        {
            ::close(m_socket);
            m_socket = -1;
        }
        m_connected.store(false, std::memory_order_release);
    }

    void MAVLinkConnection::ReceiveThread()
    {
        while (m_running.load(std::memory_order_relaxed))
        {
            if (!m_connected.load(std::memory_order_relaxed))
            {
                if (!Connect())
                {
                    // Wait before retry
                    std::this_thread::sleep_for(
                        std::chrono::milliseconds(m_config.reconnect_interval_ms));
                    continue;
                }
            }

            uint8_t buf[1024];
            auto n = ::recv(m_socket, buf, sizeof(buf), 0);
            if (n <= 0)
            {
                if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    AZLOG_WARN("PX4Bridge: Connection lost");
                    Disconnect();
                }
                continue;
            }

            // Parse MAVLink messages from stream
            for (ssize_t i = 0; i < n; ++i)
            {
                mavlink_message_t msg{};
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &m_rxStatus))
                {
                    if (m_callback)
                    {
                        m_callback(msg);
                    }
                }
            }
        }
    }

} // namespace px4_bridge
