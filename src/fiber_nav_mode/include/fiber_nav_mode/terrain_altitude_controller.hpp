#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include <geometry_msgs/msg/point.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
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
    float rate_slew = 1.f;         // [m/s²] max height rate change per second (rate limiter)
    float filter_tau = 0.5f;       // [s] low-pass filter time constant on dist_bottom
    float fallback_timeout = 5.f;  // [s] invalid dist_bottom before fallback to static alt
    float min_agl = 10.f;          // [m] safety floor
    float max_agl = 200.f;         // [m] safety ceiling
    // Look-ahead parameters
    float lookahead_time = 3.f;    // [s] look-ahead horizon
    float lookahead_max = 100.f;   // [m] max look-ahead distance
    float feedforward_gain = 0.8f; // feed-forward weight for terrain slope correction
    float max_slope = 0.5f;        // max terrain slope (clamp, ~27°)
    float slope_tau = 1.0f;        // [s] EMA filter for sensor-derived terrain slope
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

        // Direct distance sensor subscription — bypasses PX4's dist_bottom_valid.
        // sim_distance_sensor.py publishes terrain-aware AGL to /fmu/in/distance_sensor.
        // PX4's EKF2 frequently invalidates dist_bottom during FW flight due to
        // internal kinematic consistency checks, but the raw sensor data is accurate.
        dist_sensor_sub_ = node.create_subscription<px4_msgs::msg::DistanceSensor>(
            "fmu/in/distance_sensor", qos,
            [this](px4_msgs::msg::DistanceSensor::UniquePtr msg) {
                if (msg->orientation == 25 && msg->signal_quality > 0) {  // DOWNWARD
                    direct_agl_ = msg->current_distance;
                    direct_agl_valid_ = true;
                    direct_agl_timestamp_ = std::chrono::steady_clock::now();
                }
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
    /// current_amsl: drone's current AMSL altitude [m] (needed for terrain elevation estimation).
    void update(float dt_s, float ground_speed = 0.f, float vx_ned = 0.f, float vy_ned = 0.f,
                float current_amsl = 0.f)
    {
        if (!config_.enabled) {
            return;
        }

        // Determine AGL source: prefer PX4 dist_bottom, fallback to direct sensor
        bool have_agl = last_dist_bottom_valid_;
        float agl_value = last_dist_bottom_;

        if (!have_agl && direct_agl_valid_) {
            // Check direct sensor freshness (< 2s)
            const auto age = std::chrono::steady_clock::now() - direct_agl_timestamp_;
            if (age < std::chrono::seconds(2)) {
                have_agl = true;
                agl_value = direct_agl_;
            }
        }

        if (have_agl) {
            time_since_valid_ = 0.f;

            if (!filter_primed_) {
                filtered_dist_bottom_ = agl_value;
                filter_primed_ = true;
            } else {
                const float alpha = dt_s / (config_.filter_tau + dt_s);
                filtered_dist_bottom_ += alpha * (agl_value - filtered_dist_bottom_);
            }

            // Filter terrain elevation (AMSL - raw_AGL) separately.
            // Using RAW (unfiltered) AGL here is critical to avoid positive feedback:
            // if the drone descends, both current_amsl and agl_value decrease equally,
            // so terrain_amsl_raw stays constant → no feedback into the altitude target.
            // Filtering the AGL instead would create lag, causing terrain_amsl to
            // temporarily track the drone's altitude changes → positive feedback → crash.
            const float terrain_amsl_raw = current_amsl - agl_value;
            if (!terrain_amsl_primed_) {
                terrain_amsl_filtered_ = terrain_amsl_raw;
                terrain_amsl_primed_ = true;
            } else {
                const float alpha_t = dt_s / (config_.filter_tau + dt_s);
                terrain_amsl_filtered_ += alpha_t * (terrain_amsl_raw - terrain_amsl_filtered_);
            }

            // Sensor-derived terrain slope (spatial, m/m) — GPS-independent
            if (prev_terrain_valid_ && ground_speed > 1.f) {
                const float d_terrain = terrain_amsl_raw - prev_terrain_amsl_raw_;
                const float d_distance = ground_speed * dt_s;
                const float raw_slope = d_terrain / d_distance;
                if (!sensor_slope_primed_) {
                    sensor_terrain_slope_ = raw_slope;
                    sensor_slope_primed_ = true;
                } else {
                    const float alpha_s = dt_s / (config_.slope_tau + dt_s);
                    sensor_terrain_slope_ += alpha_s * (raw_slope - sensor_terrain_slope_);
                }
                terrain_slope_ = std::clamp(sensor_terrain_slope_, -config_.max_slope, config_.max_slope);
                lookahead_slope_valid_ = true;
            }
            prev_terrain_amsl_raw_ = terrain_amsl_raw;
            prev_terrain_valid_ = true;
        } else {
            time_since_valid_ += dt_s;
            // Log every 5s while invalid to track degradation
            if (filter_primed_ && std::fmod(time_since_valid_, 5.f) < dt_s) {
                RCLCPP_WARN(logger_,
                    "AGL invalid for %.1fs/%.0fs, filtered=%.1fm",
                    time_since_valid_, config_.fallback_timeout, filtered_dist_bottom_);
            }
        }

        updateLookahead(ground_speed, vx_ned, vy_ned);
    }

    /// Compute dynamic AMSL altitude target for FW TECS, with feed-forward correction.
    /// Uses terrain_amsl_filtered_ (computed from raw AGL in update()) to avoid
    /// positive feedback: terrain elevation doesn't depend on current drone altitude.
    /// Returns fallback_amsl when controller is inactive.
    [[nodiscard]] float computeTargetAmsl(float fallback_amsl) const
    {
        if (!isActive() || !terrain_amsl_primed_) {
            return fallback_amsl;
        }
        return terrain_amsl_filtered_ + clampedTargetAgl() + feedforwardCorrection();
    }

    /// Rate-limited AMSL altitude target for smooth TECS tracking.
    /// Changes the target at most `rate_slew` m/s, preventing sudden pitch changes
    /// that disturb tailsitter heading control via pitch-yaw coupling.
    /// Primes from fallback_amsl (cruise altitude) — NOT from the raw terrain target —
    /// to avoid a step change when terrain following first activates.
    ///
    /// Two-layer safety floor prevents terrain impact:
    /// 1. Terrain floor: target >= filtered_terrain + min_agl (prevents gradual descent)
    /// 2. Emergency floor: if raw AGL < min_agl, force immediate climb (catches filter lag)
    [[nodiscard]] float computeSmoothedTargetAmsl(float fallback_amsl, float dt_s,
                                                  float current_amsl = NAN)
    {
        const float raw_target = computeTargetAmsl(fallback_amsl);

        if (!amsl_target_primed_) {
            // Start from cruise altitude, not terrain target — avoids step change
            // that would cause pitch perturbation and tailsitter heading instability.
            smoothed_amsl_target_ = fallback_amsl;
            amsl_target_primed_ = true;
        } else {
            const float max_delta = config_.rate_slew * dt_s;
            const float delta = std::clamp(raw_target - smoothed_amsl_target_,
                                           -max_delta, max_delta);
            smoothed_amsl_target_ += delta;
        }

        // Layer 1: Terrain floor — target AMSL must never go below terrain + min_agl.
        // This prevents the rate-limited descent from ever putting the aircraft
        // below the minimum safe AGL. Bypasses rate limiter upward for safety.
        if (terrain_amsl_primed_) {
            const float terrain_floor = terrain_amsl_filtered_ + config_.min_agl;
            if (smoothed_amsl_target_ < terrain_floor) {
                smoothed_amsl_target_ = terrain_floor;
            }
        }

        // Layer 2: Emergency recovery — if raw AGL is already below min_agl,
        // force target above current altitude to command immediate climb.
        // Catches rapid terrain changes that the filter hasn't tracked yet.
        if (filter_primed_ && !std::isnan(current_amsl)) {
            const float raw_agl = getRawAgl();
            if (raw_agl > 0.f && raw_agl < config_.min_agl) {
                const float deficit = config_.min_agl - raw_agl;
                const float emergency_target = current_amsl + deficit;
                smoothed_amsl_target_ = std::max(smoothed_amsl_target_, emergency_target);
                if (!emergency_climb_active_) {
                    RCLCPP_WARN(logger_,
                        "TERRAIN SAFETY: AGL=%.1fm < min=%.0fm, forcing climb "
                        "(target=%.0fm AMSL, deficit=%.1fm)",
                        raw_agl, config_.min_agl, smoothed_amsl_target_, deficit);
                    emergency_climb_active_ = true;
                }
            } else if (emergency_climb_active_) {
                RCLCPP_INFO(logger_,
                    "TERRAIN SAFETY: recovered, AGL=%.1fm >= min=%.0fm",
                    raw_agl, config_.min_agl);
                emergency_climb_active_ = false;
            }
        }

        return smoothed_amsl_target_;
    }

    /// FW height rate command (positive = climb), rate-limited for smooth transitions.
    /// Must be called at the control rate for accurate rate limiting.
    [[nodiscard]] float computeHeightRate(float dt_s)
    {
        const float raw = -computeVzCommand();
        if (!rate_limit_primed_) {
            rate_limited_height_rate_ = raw;
            rate_limit_primed_ = true;
        } else {
            const float max_delta = config_.rate_slew * dt_s;
            const float delta = std::clamp(raw - rate_limited_height_rate_,
                                           -max_delta, max_delta);
            rate_limited_height_rate_ += delta;
        }
        return rate_limited_height_rate_;
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
    [[nodiscard]] bool isEmergencyClimbActive() const { return emergency_climb_active_; }
    [[nodiscard]] float sensorTerrainSlope() const { return sensor_terrain_slope_; }
    [[nodiscard]] bool isSensorSlopePrimed() const { return sensor_slope_primed_; }

    /// Raw (unfiltered) AGL from the best available source.
    [[nodiscard]] float getRawAgl() const
    {
        if (last_dist_bottom_valid_) {
            return last_dist_bottom_;
        }
        if (direct_agl_valid_) {
            const auto age = std::chrono::steady_clock::now() - direct_agl_timestamp_;
            if (age < std::chrono::seconds(2)) {
                return direct_agl_;
            }
        }
        return filtered_dist_bottom_;  // fallback to filtered
    }

    // --- Test-only accessors ---

    void reset()
    {
        filter_primed_ = false;
        filtered_dist_bottom_ = 0.f;
        time_since_valid_ = 0.f;
        last_dist_bottom_ = 0.f;
        last_dist_bottom_valid_ = false;
        direct_agl_ = 0.f;
        direct_agl_valid_ = false;
        rate_limit_primed_ = false;
        rate_limited_height_rate_ = 0.f;
        amsl_target_primed_ = false;
        smoothed_amsl_target_ = 0.f;
        terrain_amsl_primed_ = false;
        terrain_amsl_filtered_ = 0.f;
        emergency_climb_active_ = false;
        lookahead_slope_valid_ = false;
        lookahead_dist_ = 0.f;
        terrain_slope_ = 0.f;
        prev_terrain_amsl_raw_ = 0.f;
        prev_terrain_valid_ = false;
        sensor_terrain_slope_ = 0.f;
        sensor_slope_primed_ = false;
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

    /// Inject sensor-derived slope for unit testing.
    void injectSensorSlope(float slope)
    {
        sensor_terrain_slope_ = slope;
        sensor_slope_primed_ = true;
        terrain_slope_ = std::clamp(slope, -config_.max_slope, config_.max_slope);
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

        // Compute slope when both values are available (only if sensor slope not yet active)
        if (!sensor_slope_primed_ && has_current_terrain_ && has_lookahead_terrain_ && lookahead_dist_ > 1.f) {
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
    rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr dist_sensor_sub_;
    float last_dist_bottom_{0.f};
    bool last_dist_bottom_valid_{false};
    float last_x_{0.f};
    float last_y_{0.f};

    // Direct distance sensor (bypasses PX4 EKF2 validation)
    float direct_agl_{0.f};
    bool direct_agl_valid_{false};
    std::chrono::steady_clock::time_point direct_agl_timestamp_{};

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

    // Sensor-derived terrain slope state
    float prev_terrain_amsl_raw_{0.f};
    bool prev_terrain_valid_{false};
    float sensor_terrain_slope_{0.f};
    bool sensor_slope_primed_{false};

    // Filter state
    bool filter_primed_{false};
    float filtered_dist_bottom_{0.f};
    float time_since_valid_{0.f};

    // Rate limiter state for height rate output
    bool rate_limit_primed_{false};
    float rate_limited_height_rate_{0.f};

    // Rate-limited AMSL target state
    bool amsl_target_primed_{false};
    float smoothed_amsl_target_{0.f};

    // Filtered terrain elevation (AMSL) — computed from raw AGL to avoid feedback
    bool terrain_amsl_primed_{false};
    float terrain_amsl_filtered_{0.f};

    // Emergency climb state (safety floor layer 2)
    bool emergency_climb_active_{false};
};

}  // namespace fiber_nav_mode
