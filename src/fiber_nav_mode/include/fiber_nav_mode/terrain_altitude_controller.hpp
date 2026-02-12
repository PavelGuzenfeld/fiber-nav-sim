#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <geometry_msgs/msg/point.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace fiber_nav_mode {

struct TerrainFollowConfig
{
    bool enabled = false;
    float target_agl = 30.f;       // [m] desired AGL above terrain
    float kp = 0.5f;               // P-gain: AGL error -> vertical rate
    float max_rate = 3.f;          // [m/s] max vertical rate command
    float filter_tau = 0.5f;       // [s] low-pass filter time constant on dist_bottom
    float fallback_timeout = 5.f;  // [s] invalid dist_bottom before fallback to static alt
    float min_agl = 10.f;          // [m] safety floor
    float max_agl = 200.f;         // [m] safety ceiling
    // Look-ahead parameters
    float lookahead_time = 3.f;    // [s] look-ahead horizon
    float lookahead_max = 100.f;   // [m] max look-ahead distance
    float feedforward_gain = 0.8f; // feed-forward weight for terrain slope correction
    float max_slope = 0.5f;        // max terrain slope (clamp, ~27°)
};

/// Reusable terrain-following altitude controller with GIS-based look-ahead.
///
/// Subscribes to VehicleLocalPosition for dist_bottom, applies a low-pass
/// filter, and computes vertical rate commands via a P-controller to maintain
/// a target AGL (above ground level).
///
/// Look-ahead: alternates queries to /terrain/query for the current and
/// look-ahead positions. Computes terrain slope from the height difference
/// and applies feed-forward correction to anticipate terrain changes.
///
/// Sign conventions:
///   - computeVzCommand():   NED  (positive = descend)
///   - computeHeightRate():  FW   (positive = climb)
///   - computeTargetAmsl():  AMSL altitude for TECS
class TerrainAltitudeController
{
public:
    TerrainAltitudeController(rclcpp::Node& node, const TerrainFollowConfig& config)
        : config_(config)
        , logger_(node.get_logger())
    {
        if (!config_.enabled) {
            return;
        }

        auto qos = rclcpp::QoS(1).best_effort();
        lpos_sub_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "fmu/out/vehicle_local_position_v1", qos,
            [this](px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
                last_dist_bottom_ = msg->dist_bottom;
                last_dist_bottom_valid_ = msg->dist_bottom_valid;
                last_x_ = msg->x;
                last_y_ = msg->y;
            });

        // GIS terrain query: pub Point to /terrain/query, recv Float64 on /terrain/height
        terrain_query_pub_ = node.create_publisher<geometry_msgs::msg::Point>(
            "/terrain/query", 10);
        terrain_height_sub_ = node.create_subscription<std_msgs::msg::Float64>(
            "/terrain/height", 10,
            [this](std_msgs::msg::Float64::UniquePtr msg) {
                onTerrainHeightResponse(static_cast<float>(msg->data));
            });

        RCLCPP_INFO(logger_,
            "Terrain-following enabled: target_agl=%.0fm, kp=%.2f, max_rate=%.1fm/s, "
            "filter_tau=%.2fs, lookahead=%.1fs/%.0fm, ff_gain=%.2f",
            config_.target_agl, config_.kp, config_.max_rate,
            config_.filter_tau, config_.lookahead_time,
            config_.lookahead_max, config_.feedforward_gain);
    }

    /// Call every control cycle. Updates filter and look-ahead queries.
    /// ground_speed: horizontal speed [m/s], vx/vy: NED velocity for look-ahead direction.
    void update(float dt_s, float ground_speed = 0.f, float vx_ned = 0.f, float vy_ned = 0.f)
    {
        if (!config_.enabled) {
            return;
        }

        if (last_dist_bottom_valid_) {
            time_since_valid_ = 0.f;

            if (!filter_primed_) {
                filtered_dist_bottom_ = last_dist_bottom_;
                filter_primed_ = true;
            } else {
                const float alpha = dt_s / (config_.filter_tau + dt_s);
                filtered_dist_bottom_ += alpha * (last_dist_bottom_ - filtered_dist_bottom_);
            }
        } else {
            time_since_valid_ += dt_s;
        }

        updateLookahead(ground_speed, vx_ned, vy_ned);
    }

    /// Compute dynamic AMSL altitude target for FW TECS, with feed-forward correction.
    /// Returns fallback_amsl when controller is inactive.
    [[nodiscard]] float computeTargetAmsl(float current_amsl, float fallback_amsl) const
    {
        if (!isActive()) {
            return fallback_amsl;
        }
        const float terrain_amsl = current_amsl - filtered_dist_bottom_;
        return terrain_amsl + clampedTargetAgl() + feedforwardCorrection();
    }

    /// FW height rate command (positive = climb).
    [[nodiscard]] float computeHeightRate() const
    {
        return -computeVzCommand();
    }

    /// MC vertical velocity command (NED: positive = descend).
    [[nodiscard]] float computeVzCommand() const
    {
        if (!isActive()) {
            return 0.f;
        }
        const float error = filtered_dist_bottom_ - clampedTargetAgl();
        return std::clamp(error * config_.kp, -config_.max_rate, config_.max_rate);
    }

    [[nodiscard]] bool isActive() const
    {
        return config_.enabled && filter_primed_ &&
               time_since_valid_ < config_.fallback_timeout;
    }

    [[nodiscard]] bool isEnabled() const { return config_.enabled; }

    [[nodiscard]] std::optional<float> filteredDistBottom() const
    {
        if (filter_primed_) {
            return filtered_dist_bottom_;
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<float> aglError() const
    {
        if (filter_primed_) {
            return filtered_dist_bottom_ - clampedTargetAgl();
        }
        return std::nullopt;
    }

    [[nodiscard]] float clampedTargetAgl() const
    {
        return std::clamp(config_.target_agl, config_.min_agl, config_.max_agl);
    }

    /// Feed-forward correction from terrain slope look-ahead [m].
    /// Positive = terrain is rising ahead, need more altitude.
    [[nodiscard]] float feedforwardCorrection() const
    {
        if (!lookahead_slope_valid_ || lookahead_dist_ < 1.f) {
            return 0.f;
        }
        const float slope = std::clamp(terrain_slope_, -config_.max_slope, config_.max_slope);
        return slope * lookahead_dist_ * config_.feedforward_gain;
    }

    [[nodiscard]] bool isLookaheadValid() const { return lookahead_slope_valid_; }
    [[nodiscard]] float terrainSlope() const { return terrain_slope_; }
    [[nodiscard]] float lookaheadDist() const { return lookahead_dist_; }

    // --- Test-only accessors ---

    void reset()
    {
        filter_primed_ = false;
        filtered_dist_bottom_ = 0.f;
        time_since_valid_ = 0.f;
        last_dist_bottom_ = 0.f;
        last_dist_bottom_valid_ = false;
        lookahead_slope_valid_ = false;
        lookahead_dist_ = 0.f;
        terrain_slope_ = 0.f;
        current_terrain_gz_ = 0.f;
        lookahead_terrain_gz_ = 0.f;
        has_current_terrain_ = false;
        has_lookahead_terrain_ = false;
    }

    void injectDistBottom(float dist_bottom, bool valid)
    {
        last_dist_bottom_ = dist_bottom;
        last_dist_bottom_valid_ = valid;
    }

    /// Inject look-ahead terrain data for unit testing.
    void injectLookahead(float terrain_here_gz, float terrain_ahead_gz, float dist)
    {
        if (dist < 1.f) {
            lookahead_slope_valid_ = false;
            return;
        }
        current_terrain_gz_ = terrain_here_gz;
        lookahead_terrain_gz_ = terrain_ahead_gz;
        terrain_slope_ = (terrain_ahead_gz - terrain_here_gz) / dist;
        lookahead_dist_ = dist;
        lookahead_slope_valid_ = true;
    }

private:
    enum class QueryType : uint8_t { CURRENT, LOOKAHEAD };

    void onTerrainHeightResponse(float height_gz)
    {
        if (pending_query_ == QueryType::CURRENT) {
            current_terrain_gz_ = height_gz;
            has_current_terrain_ = true;
        } else {
            lookahead_terrain_gz_ = height_gz;
            has_lookahead_terrain_ = true;
        }

        // Compute slope when both values are available
        if (has_current_terrain_ && has_lookahead_terrain_ && lookahead_dist_ > 1.f) {
            terrain_slope_ = (lookahead_terrain_gz_ - current_terrain_gz_) / lookahead_dist_;
            lookahead_slope_valid_ = true;
        }
    }

    void updateLookahead(float ground_speed, float vx_ned, float vy_ned)
    {
        if (ground_speed < 1.f || !terrain_query_pub_) {
            return;
        }

        lookahead_dist_ = std::min(ground_speed * config_.lookahead_time,
                                    config_.lookahead_max);

        // Alternate between querying current position and look-ahead position
        query_tick_ = !query_tick_;

        auto query = geometry_msgs::msg::Point();

        if (query_tick_) {
            // Query look-ahead position (NED to Gazebo ENU: East=NED_y, North=NED_x)
            const float scale = lookahead_dist_ / ground_speed;
            query.x = last_y_ + vy_ned * scale;  // Gazebo East
            query.y = last_x_ + vx_ned * scale;  // Gazebo North
            pending_query_ = QueryType::LOOKAHEAD;
        } else {
            // Query current position
            query.x = last_y_;  // Gazebo East
            query.y = last_x_;  // Gazebo North
            pending_query_ = QueryType::CURRENT;
        }

        terrain_query_pub_->publish(query);
    }

    TerrainFollowConfig config_;
    rclcpp::Logger logger_;

    // Subscriptions
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    float last_dist_bottom_{0.f};
    bool last_dist_bottom_valid_{false};
    float last_x_{0.f};
    float last_y_{0.f};

    // GIS look-ahead
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr terrain_query_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr terrain_height_sub_;
    QueryType pending_query_{QueryType::CURRENT};
    bool query_tick_{false};
    float current_terrain_gz_{0.f};
    float lookahead_terrain_gz_{0.f};
    bool has_current_terrain_{false};
    bool has_lookahead_terrain_{false};
    float lookahead_dist_{0.f};
    float terrain_slope_{0.f};
    bool lookahead_slope_valid_{false};

    // Filter state
    bool filter_primed_{false};
    float filtered_dist_bottom_{0.f};
    float time_since_valid_{0.f};
};

}  // namespace fiber_nav_mode
