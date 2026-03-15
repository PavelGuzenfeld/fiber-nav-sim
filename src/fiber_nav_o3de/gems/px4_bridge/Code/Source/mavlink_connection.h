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
    /// MAVLink TCP server for PX4 SITL HIL interface (port 4560).
    /// PX4's simulator_mavlink module connects as a TCP client to this server.
    /// Protocol: MAVLink v2 over TCP stream.
    class MAVLinkConnection
    {
    public:
        using MessageCallback = AZStd::function<void(const mavlink_message_t&)>;

        struct Config
        {
            uint16_t port = 4560;
            uint8_t system_id = 1;
            uint8_t component_id = 1;
        };

        MAVLinkConnection() = default;
        ~MAVLinkConnection();

        // Non-copyable, non-movable
        MAVLinkConnection(const MAVLinkConnection&) = delete;
        MAVLinkConnection& operator=(const MAVLinkConnection&) = delete;

        /// Start server (spawns accept/receive thread)
        bool Start(const Config& config);

        /// Stop server and join thread
        void Stop();

        /// Send a MAVLink message (thread-safe)
        bool Send(const mavlink_message_t& msg);

        /// Register callback for received messages
        void SetMessageCallback(MessageCallback callback);

        /// Check if a client (PX4) is connected
        bool IsConnected() const { return m_connected.load(std::memory_order_relaxed); }

        /// Get MAVLink system/component IDs
        uint8_t GetSystemId() const { return m_config.system_id; }
        uint8_t GetComponentId() const { return m_config.component_id; }

    private:
        void ReceiveThread();
        bool Listen();
        bool AcceptClient();
        void DisconnectClient();

        Config m_config{};
        int m_listenSocket{-1};  // Server socket (persists across reconnects)
        int m_clientSocket{-1};  // Connected PX4 client
        std::atomic<bool> m_connected{false};
        std::atomic<bool> m_running{false};
        std::thread m_receiveThread;
        MessageCallback m_callback;
        mavlink_status_t m_rxStatus{};
        std::mutex m_sendMutex;
    };

} // namespace px4_bridge
