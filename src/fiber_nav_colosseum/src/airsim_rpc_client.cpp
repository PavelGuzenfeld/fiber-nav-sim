#include "fiber_nav_colosseum/airsim_rpc_client.hpp"

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstring>
#include <mutex>

namespace fiber_nav_colosseum {

namespace {

// Helper: find a key in a msgpack map and return its value, or nullptr
const msgpack::object* find_key(const msgpack::object& map_obj, const std::string& key) {
    if (map_obj.type != msgpack::type::MAP) return nullptr;
    auto& map = map_obj.via.map;
    for (uint32_t i = 0; i < map.size; ++i) {
        if (map.ptr[i].key.as<std::string>() == key) {
            return &map.ptr[i].val;
        }
    }
    return nullptr;
}

struct Vec3 { double x, y, z; };
struct Quat { double w, x, y, z; };

Vec3 parse_vector3r(const msgpack::object& obj) {
    Vec3 v{};
    if (auto* p = find_key(obj, "x_val")) v.x = p->as<double>();
    if (auto* p = find_key(obj, "y_val")) v.y = p->as<double>();
    if (auto* p = find_key(obj, "z_val")) v.z = p->as<double>();
    return v;
}

Quat parse_quaternionr(const msgpack::object& obj) {
    Quat q{1.0, 0.0, 0.0, 0.0};
    if (auto* p = find_key(obj, "w_val")) q.w = p->as<double>();
    if (auto* p = find_key(obj, "x_val")) q.x = p->as<double>();
    if (auto* p = find_key(obj, "y_val")) q.y = p->as<double>();
    if (auto* p = find_key(obj, "z_val")) q.z = p->as<double>();
    return q;
}

uint64_t parse_timestamp(const msgpack::object& obj) {
    if (auto* p = find_key(obj, "time_stamp")) return p->as<uint64_t>();
    return 0;
}

} // anonymous namespace

struct AirsimRpcClient::Impl {
    std::string host;
    uint16_t port;
    int sock_fd = -1;
    std::atomic<uint32_t> msg_id{0};
    std::mutex call_mutex;  // Serialize RPC calls (single TCP connection)

    [[nodiscard]] std::expected<void, RpcError> send_all(const void* data, size_t len) {
        auto* ptr = static_cast<const uint8_t*>(data);
        size_t sent = 0;
        while (sent < len) {
            auto n = ::send(sock_fd, ptr + sent, len - sent, MSG_NOSIGNAL);
            if (n <= 0) return std::unexpected(RpcError::connection_lost);
            sent += static_cast<size_t>(n);
        }
        return {};
    }

    [[nodiscard]] std::expected<std::vector<uint8_t>, RpcError> recv_response() {
        std::vector<uint8_t> buf(4096);
        size_t total = 0;

        // Read until we have a complete msgpack response
        while (true) {
            if (total >= buf.size()) {
                buf.resize(buf.size() * 2);
            }

            auto n = ::recv(sock_fd, buf.data() + total, buf.size() - total, 0);
            if (n <= 0) return std::unexpected(RpcError::connection_lost);
            total += static_cast<size_t>(n);

            // Try to unpack — if successful, we have a complete message
            msgpack::object_handle oh;
            try {
                oh = msgpack::unpack(reinterpret_cast<const char*>(buf.data()), total);
                buf.resize(total);
                return buf;
            } catch (const msgpack::insufficient_bytes&) {
                // Need more data
                continue;
            } catch (...) {
                return std::unexpected(RpcError::invalid_response);
            }
        }
    }
};

AirsimRpcClient::AirsimRpcClient(std::string_view host, uint16_t port)
    : impl_(std::make_unique<Impl>()) {
    impl_->host = std::string(host);
    impl_->port = port;
}

AirsimRpcClient::~AirsimRpcClient() { disconnect(); }

AirsimRpcClient::AirsimRpcClient(AirsimRpcClient&&) noexcept = default;
AirsimRpcClient& AirsimRpcClient::operator=(AirsimRpcClient&&) noexcept = default;

std::expected<void, RpcError> AirsimRpcClient::connect() {
    impl_->sock_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (impl_->sock_fd < 0) return std::unexpected(RpcError::connection_failed);

    // TCP_NODELAY for low latency RPC
    int flag = 1;
    ::setsockopt(impl_->sock_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // 5 second receive timeout (prevents blocking on large responses)
    struct timeval tv{5, 0};
    ::setsockopt(impl_->sock_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Larger receive buffer for image data
    int rcvbuf = 512 * 1024;
    ::setsockopt(impl_->sock_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(impl_->port);
    if (::inet_pton(AF_INET, impl_->host.c_str(), &addr.sin_addr) <= 0) {
        ::close(impl_->sock_fd);
        impl_->sock_fd = -1;
        return std::unexpected(RpcError::connection_failed);
    }

    if (::connect(impl_->sock_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(impl_->sock_fd);
        impl_->sock_fd = -1;
        return std::unexpected(RpcError::connection_failed);
    }

    return {};
}

void AirsimRpcClient::disconnect() {
    if (impl_ && impl_->sock_fd >= 0) {
        ::close(impl_->sock_fd);
        impl_->sock_fd = -1;
    }
}

bool AirsimRpcClient::is_connected() const {
    return impl_ && impl_->sock_fd >= 0;
}

std::expected<msgpack::object_handle, RpcError>
AirsimRpcClient::call(std::string_view method, const msgpack::sbuffer& params) {
    if (!is_connected()) return std::unexpected(RpcError::connection_failed);

    std::lock_guard<std::mutex> lock(impl_->call_mutex);

    // msgpack-rpc request: [type=0, msgid, method, params]
    uint32_t id = impl_->msg_id.fetch_add(1);

    msgpack::sbuffer req;
    msgpack::packer<msgpack::sbuffer> pk(&req);
    pk.pack_array(4);
    pk.pack(0);                        // type: request
    pk.pack(id);                       // msgid
    pk.pack(std::string(method));      // method name
    // Append the pre-packed params array
    req.write(params.data(), params.size());

    auto send_result = impl_->send_all(req.data(), req.size());
    if (!send_result) return std::unexpected(send_result.error());

    auto recv_result = impl_->recv_response();
    if (!recv_result) return std::unexpected(recv_result.error());

    try {
        auto oh = msgpack::unpack(
            reinterpret_cast<const char*>(recv_result->data()),
            recv_result->size());

        // msgpack-rpc response: [type=1, msgid, error, result]
        auto& arr = oh.get().via.array;
        if (arr.size != 4 || arr.ptr[0].via.u64 != 1) {
            return std::unexpected(RpcError::invalid_response);
        }

        // Check for server error
        if (arr.ptr[2].type != msgpack::type::NIL) {
            return std::unexpected(RpcError::server_error);
        }

        // Return the result portion
        msgpack::zone* z = new msgpack::zone();
        msgpack::object result;
        result = msgpack::object(arr.ptr[3], *z);
        return msgpack::object_handle(result, std::unique_ptr<msgpack::zone>(z));
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<msgpack::object_handle, RpcError>
AirsimRpcClient::call(std::string_view method) {
    msgpack::sbuffer empty;
    msgpack::packer<msgpack::sbuffer> pk(&empty);
    pk.pack_array(0);
    return call(method, empty);
}

std::expected<bool, RpcError> AirsimRpcClient::ping() {
    auto result = call("ping");
    if (!result) return std::unexpected(result.error());
    return result->get().as<bool>();
}

std::expected<std::vector<uint8_t>, RpcError>
AirsimRpcClient::get_image(std::string_view camera_name, int image_type) {
    // simGetImages([ImageRequest], vehicle_name)
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    pk.pack_array(2);
    // Array of 1 ImageRequest
    pk.pack_array(1);
    pk.pack_map(4);
    pk.pack("camera_name"); pk.pack(std::string(camera_name));
    pk.pack("image_type"); pk.pack(image_type);
    pk.pack("pixels_as_float"); pk.pack(false);
    pk.pack("compress"); pk.pack(true);
    // vehicle_name
    pk.pack(std::string(""));

    auto result = call("simGetImages", sbuf);
    if (!result) return std::unexpected(result.error());

    auto& obj = result->get();
    if (obj.type != msgpack::type::ARRAY || obj.via.array.size == 0)
        return std::unexpected(RpcError::invalid_response);

    auto& img = obj.via.array.ptr[0];
    if (auto* data = find_key(img, "image_data_uint8")) {
        if (data->type == msgpack::type::BIN) {
            auto& bin = data->via.bin;
            return std::vector<uint8_t>(bin.ptr, bin.ptr + bin.size);
        }
        // Cosys-AirSim may pack binary data as STR (raw) type
        if (data->type == msgpack::type::STR) {
            auto& str = data->via.str;
            return std::vector<uint8_t>(
                reinterpret_cast<const uint8_t*>(str.ptr),
                reinterpret_cast<const uint8_t*>(str.ptr) + str.size);
        }
    }

    return std::unexpected(RpcError::invalid_response);
}

std::expected<AirsimRpcClient::Pose, RpcError> AirsimRpcClient::get_pose() {
    auto result = call("simGetVehiclePose", std::string(""));
    if (!result) return std::unexpected(result.error());

    try {
        auto& obj = result->get();
        Pose pose{};
        if (auto* pos = find_key(obj, "position")) {
            auto v = parse_vector3r(*pos);
            pose.x = v.x; pose.y = v.y; pose.z = v.z;
        }
        if (auto* ori = find_key(obj, "orientation")) {
            auto q = parse_quaternionr(*ori);
            pose.qw = q.w; pose.qx = q.x; pose.qy = q.y; pose.qz = q.z;
        }
        return pose;
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<AirsimRpcClient::ImuData, RpcError>
AirsimRpcClient::get_imu_data(std::string_view sensor_name) {
    auto result = call("getImuData",
                       std::string(sensor_name),
                       std::string(""));
    if (!result) return std::unexpected(result.error());

    try {
        auto& obj = result->get();
        ImuData imu{};
        imu.time_stamp = parse_timestamp(obj);

        if (auto* av = find_key(obj, "angular_velocity")) {
            auto v = parse_vector3r(*av);
            imu.angular_velocity_x = v.x;
            imu.angular_velocity_y = v.y;
            imu.angular_velocity_z = v.z;
        }
        if (auto* la = find_key(obj, "linear_acceleration")) {
            auto v = parse_vector3r(*la);
            imu.linear_acceleration_x = v.x;
            imu.linear_acceleration_y = v.y;
            imu.linear_acceleration_z = v.z;
        }
        if (auto* ori = find_key(obj, "orientation")) {
            auto q = parse_quaternionr(*ori);
            imu.orientation_w = q.w;
            imu.orientation_x = q.x;
            imu.orientation_y = q.y;
            imu.orientation_z = q.z;
        }
        return imu;
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<AirsimRpcClient::GpsData, RpcError>
AirsimRpcClient::get_gps_data(std::string_view sensor_name) {
    auto result = call("getGpsData",
                       std::string(sensor_name),
                       std::string(""));
    if (!result) return std::unexpected(result.error());

    try {
        auto& obj = result->get();
        GpsData gps{};
        gps.time_stamp = parse_timestamp(obj);

        if (auto* valid = find_key(obj, "is_valid")) {
            gps.is_valid = valid->as<bool>();
        }

        // gnss → geo_point → {latitude, longitude, altitude}
        if (auto* gnss = find_key(obj, "gnss")) {
            if (auto* geo = find_key(*gnss, "geo_point")) {
                if (auto* p = find_key(*geo, "latitude")) gps.latitude = p->as<double>();
                if (auto* p = find_key(*geo, "longitude")) gps.longitude = p->as<double>();
                if (auto* p = find_key(*geo, "altitude")) gps.altitude = p->as<double>();
            }
            if (auto* eph = find_key(*gnss, "eph")) gps.eph = eph->as<double>();
            if (auto* epv = find_key(*gnss, "epv")) gps.epv = epv->as<double>();
            if (auto* ft = find_key(*gnss, "fix_type")) gps.fix_type = ft->as<uint8_t>();
            if (auto* vel = find_key(*gnss, "velocity")) {
                auto v = parse_vector3r(*vel);
                gps.velocity_x = v.x;
                gps.velocity_y = v.y;
                gps.velocity_z = v.z;
            }
        }
        return gps;
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<AirsimRpcClient::BarometerData, RpcError>
AirsimRpcClient::get_barometer_data(std::string_view sensor_name) {
    auto result = call("getBarometerData",
                       std::string(sensor_name),
                       std::string(""));
    if (!result) return std::unexpected(result.error());

    try {
        auto& obj = result->get();
        BarometerData baro{};
        baro.time_stamp = parse_timestamp(obj);
        if (auto* p = find_key(obj, "altitude")) baro.altitude = p->as<double>();
        if (auto* p = find_key(obj, "pressure")) baro.pressure = p->as<double>();
        if (auto* p = find_key(obj, "qnh")) baro.qnh = p->as<double>();
        return baro;
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<AirsimRpcClient::MagnetometerData, RpcError>
AirsimRpcClient::get_magnetometer_data(std::string_view sensor_name) {
    auto result = call("getMagnetometerData",
                       std::string(sensor_name),
                       std::string(""));
    if (!result) return std::unexpected(result.error());

    try {
        auto& obj = result->get();
        MagnetometerData mag{};
        mag.time_stamp = parse_timestamp(obj);
        if (auto* mf = find_key(obj, "magnetic_field_body")) {
            auto v = parse_vector3r(*mf);
            mag.magnetic_field_x = v.x;
            mag.magnetic_field_y = v.y;
            mag.magnetic_field_z = v.z;
        }
        return mag;
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<AirsimRpcClient::DistanceSensorData, RpcError>
AirsimRpcClient::get_distance_sensor_data(std::string_view sensor_name) {
    auto result = call("getDistanceSensorData",
                       std::string(sensor_name),
                       std::string(""));
    if (!result) return std::unexpected(result.error());

    try {
        auto& obj = result->get();
        DistanceSensorData dist{};
        dist.time_stamp = parse_timestamp(obj);
        if (auto* p = find_key(obj, "distance")) dist.distance = p->as<double>();
        if (auto* p = find_key(obj, "min_distance")) dist.min_distance = p->as<double>();
        if (auto* p = find_key(obj, "max_distance")) dist.max_distance = p->as<double>();
        return dist;
    } catch (...) {
        return std::unexpected(RpcError::invalid_response);
    }
}

std::expected<void, RpcError>
AirsimRpcClient::set_pose(const Pose& pose, bool ignore_collision) {
    // simSetVehiclePose(pose, ignore_collision, vehicle_name="")
    msgpack::sbuffer sbuf;
    msgpack::packer<msgpack::sbuffer> pk(&sbuf);
    pk.pack_array(3);

    // pose (map)
    pk.pack_map(2);
    pk.pack("position");
    pk.pack_map(3);
    pk.pack("x_val"); pk.pack(pose.x);
    pk.pack("y_val"); pk.pack(pose.y);
    pk.pack("z_val"); pk.pack(pose.z);
    pk.pack("orientation");
    pk.pack_map(4);
    pk.pack("w_val"); pk.pack(pose.qw);
    pk.pack("x_val"); pk.pack(pose.qx);
    pk.pack("y_val"); pk.pack(pose.qy);
    pk.pack("z_val"); pk.pack(pose.qz);

    pk.pack(ignore_collision);
    pk.pack(std::string(""));  // vehicle_name

    auto result = call("simSetVehiclePose", sbuf);
    if (!result) return std::unexpected(result.error());
    return {};
}

} // namespace fiber_nav_colosseum
