#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

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
};

/// Reusable terrain-following altitude controller.
///
/// Subscribes to VehicleLocalPosition for dist_bottom, applies a low-pass
/// filter, and computes vertical rate commands via a P-controller to maintain
/// a target AGL (above ground level).
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
            });

        RCLCPP_INFO(logger_,
            "Terrain-following enabled: target_agl=%.0fm, kp=%.2f, max_rate=%.1fm/s, "
            "filter_tau=%.2fs, fallback_timeout=%.1fs, min/max=[%.0f, %.0f]m",
            config_.target_agl, config_.kp, config_.max_rate,
            config_.filter_tau, config_.fallback_timeout,
            config_.min_agl, config_.max_agl);
    }

    /// Call every control cycle. Updates filter and validity tracking.
    void update(float dt_s)
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
                // First-order low-pass: filtered += (dt / (tau + dt)) * (raw - filtered)
                const float alpha = dt_s / (config_.filter_tau + dt_s);
                filtered_dist_bottom_ += alpha * (last_dist_bottom_ - filtered_dist_bottom_);
            }
        } else {
            time_since_valid_ += dt_s;
        }
    }

    /// Compute dynamic AMSL altitude target for FW TECS.
    /// terrain_amsl = current_amsl - filtered_dist_bottom
    /// target_amsl  = terrain_amsl + clamped_target_agl
    /// Returns fallback_amsl when controller is inactive.
    [[nodiscard]] float computeTargetAmsl(float current_amsl, float fallback_amsl) const
    {
        if (!isActive()) {
            return fallback_amsl;
        }
        const float terrain_amsl = current_amsl - filtered_dist_bottom_;
        return terrain_amsl + clampedTargetAgl();
    }

    /// FW height rate command (positive = climb).
    /// Inverts the NED P-controller output for FW convention.
    [[nodiscard]] float computeHeightRate() const
    {
        return -computeVzCommand();
    }

    /// MC vertical velocity command (NED: positive = descend).
    /// P-controller: vz = clamp((filtered_dist_bottom - target_agl) * kp, -max, max)
    /// When dist > target (too high), vz > 0 → descend. Correct NED sign.
    [[nodiscard]] float computeVzCommand() const
    {
        if (!isActive()) {
            return 0.f;
        }
        const float error = filtered_dist_bottom_ - clampedTargetAgl();
        return std::clamp(error * config_.kp, -config_.max_rate, config_.max_rate);
    }

    /// True when filter is primed and dist_bottom has been valid within fallback_timeout.
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

    /// AGL error: positive = too high, negative = too low.
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

    // --- Test-only accessors ---

    /// Reset internal state (for unit testing without ROS).
    void reset()
    {
        filter_primed_ = false;
        filtered_dist_bottom_ = 0.f;
        time_since_valid_ = 0.f;
        last_dist_bottom_ = 0.f;
        last_dist_bottom_valid_ = false;
    }

    /// Inject a dist_bottom reading (for unit testing without ROS subscription).
    void injectDistBottom(float dist_bottom, bool valid)
    {
        last_dist_bottom_ = dist_bottom;
        last_dist_bottom_valid_ = valid;
    }

private:
    TerrainFollowConfig config_;
    rclcpp::Logger logger_;

    // Subscription
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr lpos_sub_;
    float last_dist_bottom_{0.f};
    bool last_dist_bottom_valid_{false};

    // Filter state
    bool filter_primed_{false};
    float filtered_dist_bottom_{0.f};
    float time_since_valid_{0.f};
};

}  // namespace fiber_nav_mode
