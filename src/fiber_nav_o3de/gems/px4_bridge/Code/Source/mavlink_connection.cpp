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

        if (!Listen())
        {
            m_running.store(false);
            return false;
        }

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
        DisconnectClient();
        if (m_listenSocket >= 0)
        {
            ::close(m_listenSocket);
            m_listenSocket = -1;
        }
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
        auto sent = ::send(m_clientSocket, buffer, len, MSG_NOSIGNAL);
        if (sent < 0)
        {
            AZLOG_WARN("PX4Bridge: Send failed: %s", strerror(errno));
            DisconnectClient();
            return false;
        }
        return sent == static_cast<ssize_t>(len);
    }

    void MAVLinkConnection::SetMessageCallback(MessageCallback callback)
    {
        m_callback = AZStd::move(callback);
    }

    bool MAVLinkConnection::Listen()
    {
        m_listenSocket = ::socket(AF_INET, SOCK_STREAM, 0);
        if (m_listenSocket < 0)
        {
            AZLOG_ERROR("PX4Bridge: socket() failed: %s", strerror(errno));
            return false;
        }

        int reuse = 1;
        setsockopt(m_listenSocket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_config.port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (::bind(m_listenSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            AZLOG_ERROR("PX4Bridge: bind() on port %u failed: %s", m_config.port, strerror(errno));
            ::close(m_listenSocket);
            m_listenSocket = -1;
            return false;
        }

        if (::listen(m_listenSocket, 1) < 0)
        {
            AZLOG_ERROR("PX4Bridge: listen() failed: %s", strerror(errno));
            ::close(m_listenSocket);
            m_listenSocket = -1;
            return false;
        }

        // Non-blocking so we can poll with timeout
        int flags = fcntl(m_listenSocket, F_GETFL, 0);
        fcntl(m_listenSocket, F_SETFL, flags | O_NONBLOCK);

        AZLOG_INFO("PX4Bridge: TCP server listening on port %u", m_config.port);
        return true;
    }

    bool MAVLinkConnection::AcceptClient()
    {
        pollfd pfd{};
        pfd.fd = m_listenSocket;
        pfd.events = POLLIN;

        int ret = ::poll(&pfd, 1, 1000); // 1s timeout
        if (ret <= 0 || !(pfd.revents & POLLIN))
        {
            return false;
        }

        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        m_clientSocket = ::accept(m_listenSocket,
            reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (m_clientSocket < 0)
        {
            return false;
        }

        // Low-latency sensor data
        int nodelay = 1;
        setsockopt(m_clientSocket, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        // Receive timeout (100ms)
        timeval tv{};
        tv.tv_usec = 100000;
        setsockopt(m_clientSocket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        m_connected.store(true, std::memory_order_release);

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
        AZLOG_INFO("PX4Bridge: PX4 connected from %s:%u", client_ip, ntohs(client_addr.sin_port));
        return true;
    }

    void MAVLinkConnection::DisconnectClient()
    {
        if (m_clientSocket >= 0)
        {
            ::close(m_clientSocket);
            m_clientSocket = -1;
        }
        m_connected.store(false, std::memory_order_release);
    }

    void MAVLinkConnection::ReceiveThread()
    {
        while (m_running.load(std::memory_order_relaxed))
        {
            if (!m_connected.load(std::memory_order_relaxed))
            {
                if (!AcceptClient())
                {
                    continue; // AcceptClient has 1s poll timeout
                }
            }

            uint8_t buf[1024];
            auto n = ::recv(m_clientSocket, buf, sizeof(buf), 0);
            if (n <= 0)
            {
                if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    AZLOG_WARN("PX4Bridge: PX4 disconnected, waiting for reconnect");
                    DisconnectClient();
                }
                continue;
            }

            // Parse MAVLink messages
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
