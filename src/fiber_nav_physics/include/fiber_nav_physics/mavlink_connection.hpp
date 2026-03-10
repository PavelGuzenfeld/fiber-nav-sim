#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#define MAVLINK_USE_MESSAGE_INFO
#include <mavlink.h>

namespace fiber_nav_physics
{

/// Low-level MAVLink TCP connection to PX4 SITL (port 4560).
/// Engine-agnostic — extracted from O3DE px4_bridge.
class MavlinkConnection
{
public:
    using MessageCallback = std::function<void(const mavlink_message_t&)>;
    using LogCallback = std::function<void(const char*)>;

    struct Config
    {
        const char* host = "127.0.0.1";
        uint16_t port = 4560;
        uint8_t system_id = 1;
        uint8_t component_id = 1;
        int connect_timeout_ms = 5000;
        int reconnect_interval_ms = 2000;
    };

    MavlinkConnection() = default;
    ~MavlinkConnection();

    MavlinkConnection(const MavlinkConnection&) = delete;
    MavlinkConnection& operator=(const MavlinkConnection&) = delete;

    /// Start connection (spawns receive thread)
    bool start(const Config& config);

    /// Stop connection and join receive thread
    void stop();

    /// Send a MAVLink message (thread-safe)
    bool send(const mavlink_message_t& msg);

    /// Register callback for received messages
    void set_message_callback(MessageCallback callback);

    /// Set optional log callback (default: no logging)
    void set_log_callback(LogCallback callback);

    bool is_connected() const { return connected_.load(std::memory_order_relaxed); }

    uint8_t system_id() const { return config_.system_id; }
    uint8_t component_id() const { return config_.component_id; }

private:
    void receive_thread();
    bool connect();
    void disconnect();

    Config config_{};
    int socket_{-1};
    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    MessageCallback on_message_;
    LogCallback on_log_;
    mavlink_status_t rx_status_{};
    std::mutex send_mutex_;
};

} // namespace fiber_nav_physics
