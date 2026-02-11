#pragma once

#include <AzCore/std/functional.h>
#include <AzCore/std/containers/vector.h>

#include <cstdint>
#include <atomic>
#include <thread>

// MAVLink v2 C headers (fetched via CMake)
#define MAVLINK_USE_MESSAGE_INFO
#include <mavlink.h>

namespace px4_bridge
{
    /// Low-level MAVLink TCP connection to PX4 SITL simulator API (port 4560).
    /// PX4 SITL listens on TCP 4560 for HIL simulator connections.
    /// Protocol: MAVLink v2 over TCP stream.
    class MAVLinkConnection
    {
    public:
        using MessageCallback = AZStd::function<void(const mavlink_message_t&)>;

        struct Config
        {
            const char* host = "127.0.0.1";
            uint16_t port = 4560;
            uint8_t system_id = 1;
            uint8_t component_id = 1;
            int connect_timeout_ms = 5000;
            int reconnect_interval_ms = 2000;
        };

        MAVLinkConnection() = default;
        ~MAVLinkConnection();

        // Non-copyable, non-movable
        MAVLinkConnection(const MAVLinkConnection&) = delete;
        MAVLinkConnection& operator=(const MAVLinkConnection&) = delete;

        /// Start connection (spawns receive thread)
        bool Start(const Config& config);

        /// Stop connection and join receive thread
        void Stop();

        /// Send a MAVLink message (thread-safe)
        bool Send(const mavlink_message_t& msg);

        /// Register callback for received messages
        void SetMessageCallback(MessageCallback callback);

        /// Check if connected
        bool IsConnected() const { return m_connected.load(std::memory_order_relaxed); }

        /// Get MAVLink system/component IDs
        uint8_t GetSystemId() const { return m_config.system_id; }
        uint8_t GetComponentId() const { return m_config.component_id; }

    private:
        void ReceiveThread();
        bool Connect();
        void Disconnect();

        Config m_config{};
        int m_socket{-1};
        std::atomic<bool> m_connected{false};
        std::atomic<bool> m_running{false};
        std::thread m_receiveThread;
        MessageCallback m_callback;
        mavlink_status_t m_rxStatus{};
        std::mutex m_sendMutex;
    };

} // namespace px4_bridge
