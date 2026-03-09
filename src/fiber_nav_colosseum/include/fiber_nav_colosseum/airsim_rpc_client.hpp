#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <msgpack.hpp>

namespace fiber_nav_colosseum {

/// Error type for RPC operations
enum class RpcError {
    connection_failed,
    connection_lost,
    timeout,
    invalid_response,
    server_error,
};

/// Raw TCP + msgpack-rpc client for AirSim API (port 41451)
/// Implements the msgpack-rpc spec: [type, msgid, method, params]
class AirsimRpcClient {
public:
    explicit AirsimRpcClient(std::string_view host = "127.0.0.1",
                              uint16_t port = 41451);
    ~AirsimRpcClient();

    AirsimRpcClient(const AirsimRpcClient&) = delete;
    AirsimRpcClient& operator=(const AirsimRpcClient&) = delete;
    AirsimRpcClient(AirsimRpcClient&&) noexcept;
    AirsimRpcClient& operator=(AirsimRpcClient&&) noexcept;

    /// Connect to AirSim API server
    [[nodiscard]] std::expected<void, RpcError> connect();

    /// Disconnect from server
    void disconnect();

    /// Check if connected
    [[nodiscard]] bool is_connected() const;

    /// Call an RPC method and return the msgpack response object
    [[nodiscard]] std::expected<msgpack::object_handle, RpcError>
    call(std::string_view method, const msgpack::sbuffer& params);

    /// Convenience: call with no params
    [[nodiscard]] std::expected<msgpack::object_handle, RpcError>
    call(std::string_view method);

    /// Convenience: pack params and call
    template <typename... Args>
        requires(sizeof...(Args) > 0 &&
                 !(sizeof...(Args) == 1 &&
                   (std::is_same_v<std::decay_t<Args>, msgpack::sbuffer> || ...)))
    [[nodiscard]] std::expected<msgpack::object_handle, RpcError>
    call(std::string_view method, Args&&... args) {
        msgpack::sbuffer sbuf;
        msgpack::packer<msgpack::sbuffer> pk(&sbuf);
        pk.pack_array(sizeof...(Args));
        (pk.pack(std::forward<Args>(args)), ...);
        return call(method, sbuf);
    }

    // ── AirSim convenience methods ──

    /// Ping the server
    [[nodiscard]] std::expected<bool, RpcError> ping();

    /// Get camera image (Scene=0, DepthPlanar=1, Segmentation=5)
    [[nodiscard]] std::expected<std::vector<uint8_t>, RpcError>
    get_image(std::string_view camera_name, int image_type = 0);

    /// Get vehicle pose (position + orientation)
    struct Pose {
        double x, y, z;       // NED position (meters)
        double qw, qx, qy, qz; // quaternion
    };
    [[nodiscard]] std::expected<Pose, RpcError> get_pose();

    /// Get IMU data
    struct ImuData {
        double angular_velocity_x, angular_velocity_y, angular_velocity_z;
        double linear_acceleration_x, linear_acceleration_y, linear_acceleration_z;
        double orientation_w, orientation_x, orientation_y, orientation_z;
        uint64_t time_stamp;
    };
    [[nodiscard]] std::expected<ImuData, RpcError>
    get_imu_data(std::string_view sensor_name = "Imu");

    /// Get GPS data
    struct GpsData {
        double latitude, longitude, altitude;
        double velocity_x, velocity_y, velocity_z;
        double eph, epv;
        uint8_t fix_type;  // 0=NO_FIX, 2=2D_FIX, 3=3D_FIX
        bool is_valid;
        uint64_t time_stamp;
    };
    [[nodiscard]] std::expected<GpsData, RpcError>
    get_gps_data(std::string_view sensor_name = "Gps");

    /// Get barometer data
    struct BarometerData {
        double altitude, pressure, qnh;
        uint64_t time_stamp;
    };
    [[nodiscard]] std::expected<BarometerData, RpcError>
    get_barometer_data(std::string_view sensor_name = "Barometer");

    /// Get magnetometer data
    struct MagnetometerData {
        double magnetic_field_x, magnetic_field_y, magnetic_field_z;
        uint64_t time_stamp;
    };
    [[nodiscard]] std::expected<MagnetometerData, RpcError>
    get_magnetometer_data(std::string_view sensor_name = "Magnetometer");

    /// Get distance sensor data
    struct DistanceSensorData {
        double distance, min_distance, max_distance;
        uint64_t time_stamp;
    };
    [[nodiscard]] std::expected<DistanceSensorData, RpcError>
    get_distance_sensor_data(std::string_view sensor_name = "Distance");

    /// Set vehicle pose (for external physics)
    [[nodiscard]] std::expected<void, RpcError>
    set_pose(const Pose& pose, bool ignore_collision = true);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fiber_nav_colosseum
