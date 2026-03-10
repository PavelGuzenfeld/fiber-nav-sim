#include "fiber_nav_physics/mavlink_connection.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace fiber_nav_physics
{

namespace
{
    constexpr int recv_timeout_us = 100'000; // 100ms
    constexpr int recv_buf_size = 1024;
} // namespace

MavlinkConnection::~MavlinkConnection()
{
    stop();
}

bool MavlinkConnection::start(const Config& config)
{
    if (running_.load())
    {
        return false;
    }
    config_ = config;
    running_.store(true);
    receive_thread_ = std::thread(&MavlinkConnection::receive_thread, this);
    return true;
}

void MavlinkConnection::stop()
{
    running_.store(false);
    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }
    disconnect();
}

bool MavlinkConnection::send(const mavlink_message_t& msg)
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    auto len = mavlink_msg_to_send_buffer(buffer, &msg);

    std::lock_guard lock(send_mutex_);
    auto sent = ::send(socket_, buffer, len, MSG_NOSIGNAL);
    if (sent < 0)
    {
        if (on_log_)
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "MAVLink send failed: %s", strerror(errno));
            on_log_(buf);
        }
        disconnect();
        return false;
    }
    return sent == static_cast<ssize_t>(len);
}

void MavlinkConnection::set_message_callback(MessageCallback callback)
{
    on_message_ = std::move(callback);
}

void MavlinkConnection::set_log_callback(LogCallback callback)
{
    on_log_ = std::move(callback);
}

bool MavlinkConnection::connect()
{
    socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_ < 0)
    {
        return false;
    }

    // Non-blocking connect with timeout
    int flags = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.port);
    inet_pton(AF_INET, config_.host, &addr.sin_addr);

    int ret = ::connect(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (ret < 0 && errno != EINPROGRESS)
    {
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    if (ret < 0)
    {
        pollfd pfd{};
        pfd.fd = socket_;
        pfd.events = POLLOUT;

        ret = ::poll(&pfd, 1, config_.connect_timeout_ms);
        if (ret <= 0 || !(pfd.revents & POLLOUT))
        {
            ::close(socket_);
            socket_ = -1;
            return false;
        }

        int err = 0;
        socklen_t errlen = sizeof(err);
        getsockopt(socket_, SOL_SOCKET, SO_ERROR, &err, &errlen);
        if (err != 0)
        {
            ::close(socket_);
            socket_ = -1;
            return false;
        }
    }

    // Restore blocking + TCP_NODELAY
    fcntl(socket_, F_SETFL, flags);
    int nodelay = 1;
    setsockopt(socket_, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    // Receive timeout
    timeval tv{};
    tv.tv_usec = recv_timeout_us;
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    connected_.store(true, std::memory_order_release);
    if (on_log_)
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "MAVLink connected to %s:%u", config_.host, config_.port);
        on_log_(buf);
    }
    return true;
}

void MavlinkConnection::disconnect()
{
    if (socket_ >= 0)
    {
        ::close(socket_);
        socket_ = -1;
    }
    connected_.store(false, std::memory_order_release);
}

void MavlinkConnection::receive_thread()
{
    while (running_.load(std::memory_order_relaxed))
    {
        if (!connected_.load(std::memory_order_relaxed))
        {
            if (!connect())
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.reconnect_interval_ms));
                continue;
            }
        }

        uint8_t buf[recv_buf_size];
        auto n = ::recv(socket_, buf, sizeof(buf), 0);
        if (n <= 0)
        {
            if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK))
            {
                if (on_log_)
                {
                    on_log_("MAVLink connection lost");
                }
                disconnect();
            }
            continue;
        }

        for (ssize_t i = 0; i < n; ++i)
        {
            mavlink_message_t msg{};
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &rx_status_))
            {
                if (on_message_)
                {
                    on_message_(msg);
                }
            }
        }
    }
}

} // namespace fiber_nav_physics
