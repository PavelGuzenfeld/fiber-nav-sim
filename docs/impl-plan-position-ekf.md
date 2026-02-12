# Implementation Plan: Position EKF + GPS-Denied Navigation

Reference: `docs/plan-position-ekf-gps-denied-nav.md`

## Overview

Add a 6-state Extended Kalman Filter (position dead reckoning from spool + optical flow) and upgrade the GPS-denied navigator from open-loop fixed headings to closed-loop position-based steering. Two new packages/files in `fiber_nav_fusion`, plus modifications to the navigator in `fiber_nav_mode`.

## Package: `fiber_nav_fusion` — Position EKF

### New File 1: `include/fiber_nav_fusion/position_ekf.hpp`

**Pure testable EKF math — no ROS dependencies.**

```cpp
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace fiber_nav_fusion {

/// 6-state 2D position EKF for GPS-denied dead reckoning.
/// State: [x, y, vx, vy, wind_x, wind_y] in NED frame.
/// Prediction: spool_speed * body_to_NED(OF_direction) - wind
/// Measurements: OF quality (adaptive R), cable length constraint, speed consistency
struct PositionEkfState {
    Eigen::Vector<float, 6> x = Eigen::Vector<float, 6>::Zero();
    Eigen::Matrix<float, 6, 6> P = Eigen::Matrix<float, 6, 6>::Identity();
    double last_update_time = 0.0;
    bool initialized = false;
};

struct PositionEkfConfig {
    // Process noise
    float q_pos = 0.1f;          // Position process noise [m²/s]
    float q_vel = 1.0f;          // Velocity process noise [m²/s³]
    float q_wind = 0.01f;        // Wind process noise [m²/s³]

    // Measurement noise base values
    float r_velocity = 0.5f;     // Velocity measurement noise [m²/s²]
    float r_speed = 0.1f;        // Speed consistency noise [m²/s²]

    // OF quality → measurement noise scaling
    float of_quality_min = 0.3f; // Below this, use fallback noise
    float of_quality_scale = 5.0f; // R_vel *= scale when quality=min, 1.0 when quality=1.0

    // Cable constraint
    float cable_margin = 0.95f;  // Apply constraint when ||pos|| > margin * cable_length

    // Initial uncertainty
    float p0_pos = 1.0f;         // Initial position variance [m²]
    float p0_vel = 1.0f;         // Initial velocity variance [m²/s²]
    float p0_wind = 4.0f;        // Initial wind variance [m²/s²]
};

/// Predict step: propagate state forward by dt using velocity model.
/// v_ned = spool_speed * R(attitude) * OF_direction_body
/// pos += (v_measured - wind_estimate) * dt
/// wind stays constant (random walk in Q)
PositionEkfState predict(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float dt,
    float vn_measured,   // NED north velocity from spool+OF+attitude
    float ve_measured);  // NED east velocity from spool+OF+attitude

/// Velocity measurement update: fuse spool+OF velocity with adaptive noise.
/// H = [0 0 1 0 0 0; 0 0 0 1 0 0]  (observe vx, vy)
/// R scaled by OF quality: low quality → high noise
PositionEkfState updateVelocity(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float vn_measured,
    float ve_measured,
    float of_quality);   // [0,1] from optical flow node

/// Speed consistency update: |v_state| should match spool_speed.
/// Scalar measurement: ||v|| = spool_speed
/// Prevents velocity direction from drifting when OF quality is marginal.
PositionEkfState updateSpeedConsistency(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float spool_speed);

/// Cable length inequality constraint: ||pos|| <= cable_deployed_length.
/// If ||pos|| > margin * cable_length, project position back onto circle.
/// Implemented as a pseudo-measurement: ||pos|| = cable_length with low noise.
PositionEkfState applyCableConstraint(
    const PositionEkfState& state,
    const PositionEkfConfig& config,
    float cable_deployed_length);

/// Initialize EKF state at (0,0) with zero velocity and zero wind.
/// Called once at takeoff when GPS is available.
PositionEkfState initialize(const PositionEkfConfig& config);

/// Reset position to (0,0) — used if GPS becomes available again.
PositionEkfState resetPosition(
    const PositionEkfState& state,
    float x, float y, float variance);

/// Extract position estimate from state.
inline std::array<float, 2> position(const PositionEkfState& s) {
    return {s.x(0), s.x(1)};
}

/// Extract velocity estimate from state.
inline std::array<float, 2> velocity(const PositionEkfState& s) {
    return {s.x(2), s.x(3)};
}

/// Extract wind estimate from state.
inline std::array<float, 2> wind(const PositionEkfState& s) {
    return {s.x(4), s.x(5)};
}

/// Position uncertainty (1-sigma) in X and Y.
inline std::array<float, 2> positionSigma(const PositionEkfState& s) {
    return {std::sqrt(s.P(0, 0)), std::sqrt(s.P(1, 1))};
}

/// Distance from home.
inline float distanceFromHome(const PositionEkfState& s) {
    return std::sqrt(s.x(0) * s.x(0) + s.x(1) * s.x(1));
}

}  // namespace fiber_nav_fusion
```

**Key design decisions:**
- Pure functions: `predict()`, `updateVelocity()`, etc. take and return state — no side effects
- Eigen for matrix math (already a dependency of `fiber_nav_mode`)
- Config struct with defaults, overridden by YAML params
- Cable constraint as pseudo-measurement (not hard projection) — preserves EKF covariance
- Wind state allows FW course correction

### New File 2: `src/position_ekf.cpp`

Implementation of the functions declared in the header.

**predict():**
```
F = [I2  dt*I2  -dt*I2]   // pos += (vel - wind) * dt
    [0   I2     0     ]   // vel stays (updated by measurement)
    [0   0      I2    ]   // wind random walk

x_pred = F * x
x_pred[0:2] += (v_measured - x[4:6]) * dt   // position from velocity
x_pred[2:4] = v_measured                      // velocity = measurement

Q = diag(q_pos*dt, q_pos*dt, q_vel*dt, q_vel*dt, q_wind*dt, q_wind*dt)
P_pred = F * P * F^T + Q
```

**updateVelocity():**
```
H = [0 0 1 0 0 0]
    [0 0 0 1 0 0]
z = [vn_measured, ve_measured]

// Adaptive R based on OF quality
quality_factor = lerp(of_quality_scale, 1.0, (quality - min) / (1.0 - min))
R = diag(r_velocity * quality_factor, r_velocity * quality_factor)

// Standard Kalman update
S = H * P * H^T + R
K = P * H^T * S^-1
x = x + K * (z - H * x)
P = (I - K * H) * P
```

**updateSpeedConsistency():**
```
// Scalar measurement: ||v|| = spool_speed
speed_state = sqrt(vx² + vy²)
if speed_state < 0.1: skip (near-zero division)

// Jacobian of ||v|| w.r.t. state
H = [0 0 vx/||v|| vy/||v|| 0 0]
z = spool_speed
R = r_speed

// Standard scalar Kalman update
```

**applyCableConstraint():**
```
dist = ||pos||
if dist <= margin * cable_length: return (no constraint)

// Pseudo-measurement: ||pos|| = cable_length
H = [x/dist y/dist 0 0 0 0]
z = cable_length
R = (cable_length * 0.05)²   // 5% of cable length noise

// Standard scalar Kalman update — pulls pos toward circle
```

### New File 3: `test/test_position_ekf.cpp`

Doctest unit tests for the pure EKF functions.

**Test cases:**
1. `initialize` — state is zero, P is diagonal with config values
2. `predict with zero velocity` — position unchanged, P grows
3. `predict with constant velocity` — position advances linearly
4. `updateVelocity high quality` — state converges to measurement
5. `updateVelocity low quality` — high R, slow convergence
6. `updateSpeedConsistency` — corrects speed magnitude
7. `applyCableConstraint inside` — no change when inside radius
8. `applyCableConstraint outside` — pulls position back
9. `wind estimation` — after many updates with offset, wind state converges
10. `full cycle predict+update` — multi-step integration matches expected trajectory
11. `positionSigma decreases with updates` — uncertainty shrinks
12. `distanceFromHome` — basic geometry check

### New File 4: `src/position_ekf_node.cpp`

ROS2 node that runs the Position EKF.

**Subscribers:**
| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `/sensors/fiber_spool/status` | `SpoolStatus` | 10 (RELIABLE) | Scalar speed + deployed length |
| `/sensors/vision_direction` | `Vector3Stamped` | 10 (RELIABLE) | Body-frame direction unit vector |
| `/sensors/optical_flow/quality` | `Float64` | 10 (RELIABLE) | OF quality [0,1] for adaptive R |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | BEST_EFFORT | Body→NED rotation quaternion |
| `/cable/status` | `CableStatus` | BEST_EFFORT | Deployed cable length for constraint |
| `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | BEST_EFFORT | GPS health flag (xy_global) |

**Publishers:**
| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/position_ekf/estimate` | `geometry_msgs/PoseWithCovarianceStamped` | 50 Hz | Position + 2D covariance |
| `/position_ekf/velocity` | `geometry_msgs/TwistStamped` | 50 Hz | Velocity estimate |
| `/position_ekf/wind` | `geometry_msgs/Vector3Stamped` | 50 Hz | Wind estimate (for Foxglove) |
| `/position_ekf/diagnostics` | `std_msgs/String` | 1 Hz | JSON diagnostics |
| `/fmu/in/vehicle_visual_odometry` | `VehicleOdometry` | 50 Hz | Position+velocity to PX4 EKF |

**Parameters** (in `sensor_params.yaml`):
```yaml
position_ekf_node:
  ros__parameters:
    enabled: false                # Master enable — disabled by default
    publish_rate: 50.0            # Hz
    q_pos: 0.1                    # Position process noise
    q_vel: 1.0                    # Velocity process noise
    q_wind: 0.01                  # Wind process noise
    r_velocity: 0.5               # Velocity measurement noise
    r_speed: 0.1                  # Speed consistency noise
    of_quality_min: 0.3           # OF quality threshold
    of_quality_scale: 5.0         # R multiplier at min quality
    cable_margin: 0.95            # Constraint activation margin
    p0_pos: 1.0                   # Initial position uncertainty
    p0_vel: 1.0                   # Initial velocity uncertainty
    p0_wind: 4.0                  # Initial wind uncertainty
    feed_px4: true                # Publish to /fmu/in/vehicle_visual_odometry
    position_variance_to_px4: 10.0  # Position variance sent to PX4
```

**Algorithm (50 Hz timer callback):**
1. Check `enabled` — if false, do nothing (existing `fiber_vision_fusion` handles PX4)
2. Check GPS health from VehicleLocalPosition — if `xy_global` true, don't run EKF (GPS is fine)
3. When `xy_global` goes false → initialize EKF at current PX4 EKF position (last GPS fix)
4. Each tick:
   a. Compute v_NED from spool_speed, OF direction, attitude quaternion (same body→NED as fusion node)
   b. `predict(state, config, dt, vn, ve)`
   c. `updateVelocity(state, config, vn, ve, of_quality)`
   d. `updateSpeedConsistency(state, config, spool_speed)`
   e. If cable status valid: `applyCableConstraint(state, config, deployed_length)`
   f. Publish position estimate, velocity, wind
   g. If `feed_px4`: publish VehicleOdometry with position + velocity

**Relationship with `fiber_vision_fusion`:**
- `fiber_vision_fusion` continues to publish velocity to PX4 (it's well-tested and handles ZUPT, health, etc.)
- `position_ekf_node` adds position estimation on top
- When `feed_px4` is true AND GPS is denied, position_ekf publishes its position estimate to PX4 with moderate variance (overriding fusion node's high-variance echo-back)
- The two nodes don't conflict because:
  - Fusion node echoes EKF position back with variance=1e6 (ignored by PX4)
  - Position EKF publishes with variance=10.0 (PX4 will use this)
  - Both publish velocity, but fusion node's is better-tuned — position_ekf should set velocity_variance higher or skip velocity in its VehicleOdometry

**Decision: position_ekf_node publishes POSITION ONLY to PX4.** Velocity comes from existing fusion node. This prevents conflicting velocity inputs.

```cpp
// In position_ekf_node VehicleOdometry publish:
odom.position[0] = state.x(0);  // north
odom.position[1] = state.x(1);  // east
odom.position[2] = ekf_z_;      // Z from PX4 (baro-controlled)
odom.position_variance[0] = position_variance_to_px4_;
odom.position_variance[1] = position_variance_to_px4_;
odom.position_variance[2] = 1e6f;  // no Z constraint
// Velocity: NaN = don't override fusion node's velocity
odom.velocity[0] = std::nanf("");
odom.velocity[1] = std::nanf("");
odom.velocity[2] = std::nanf("");
```

### Modified File: `CMakeLists.txt` (fiber_nav_fusion)

Add after existing executables:

```cmake
# Position EKF library (header-only functions + implementation)
add_library(position_ekf src/position_ekf.cpp)
target_include_directories(position_ekf PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(position_ekf PUBLIC Eigen3::Eigen)
target_compile_features(position_ekf PUBLIC cxx_std_23)

# Position EKF node
add_executable(position_ekf_node src/position_ekf_node.cpp)
target_link_libraries(position_ekf_node PRIVATE position_ekf)
ament_target_dependencies(position_ekf_node
  rclcpp std_msgs geometry_msgs px4_msgs fiber_nav_sensors tf2)
target_link_libraries(position_ekf_node PRIVATE Eigen3::Eigen)
```

Add `find_package(Eigen3 REQUIRED)` to find_package section.

Add `position_ekf_node` to `install(TARGETS ...)`.

Add test:
```cmake
add_executable(test_position_ekf test/test_position_ekf.cpp)
target_link_libraries(test_position_ekf PRIVATE doctest::doctest position_ekf)
add_test(NAME test_position_ekf COMMAND test_position_ekf)
```

---

## Package: `fiber_nav_mode` — Navigator Upgrade

### Modified File: `include/fiber_nav_mode/vtol_navigation_mode.hpp`

**Changes to `GpsDeniedConfig`:**
```cpp
struct GpsDeniedConfig
{
    bool enabled = false;
    // Existing time-based params (kept as fallbacks)
    float wp_time_s = 30.f;
    float return_time_s = 120.f;
    float descent_time_s = 60.f;
    float altitude_kp = 0.5f;
    float altitude_max_vz = 3.f;
    float fw_speed = 18.f;
    float return_heading = NAN;

    // NEW: Position-based navigation (when position_ekf is available)
    bool use_position_ekf = false;       // Enable position-based steering
    float ekf_wp_accept_radius = 80.f;   // WP acceptance radius [m] (larger than GPS mode)
    float ekf_home_accept_radius = 100.f;// Home acceptance radius [m]
    float ekf_max_uncertainty = 200.f;   // Fallback to time-based if sigma > this [m]
};
```

**New subscription (in constructor):**
```cpp
// Position EKF estimate (for closed-loop GPS-denied navigation)
if (_config.gps_denied.use_position_ekf) {
    _ekf_pos_sub = node.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/position_ekf/estimate", rclcpp::QoS(1).best_effort(),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
            _ekf_pos_x = static_cast<float>(msg->pose.pose.position.x);
            _ekf_pos_y = static_cast<float>(msg->pose.pose.position.y);
            _ekf_pos_sigma_x = static_cast<float>(std::sqrt(msg->pose.covariance[0]));
            _ekf_pos_sigma_y = static_cast<float>(std::sqrt(msg->pose.covariance[7]));
            _ekf_pos_valid = true;
            _ekf_pos_time = node().get_clock()->now();
        });
}
```

**New private members:**
```cpp
// Position EKF subscription
rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _ekf_pos_sub;
float _ekf_pos_x{0.f};
float _ekf_pos_y{0.f};
float _ekf_pos_sigma_x{0.f};
float _ekf_pos_sigma_y{0.f};
bool _ekf_pos_valid{false};
rclcpp::Time _ekf_pos_time;
```

**New helper method:**
```cpp
/// Get dead-reckoned position if available and trustworthy.
/// Returns nullopt if EKF not enabled, not valid, or uncertainty too high.
std::optional<Eigen::Vector2f> drPosition() const
{
    if (!_config.gps_denied.use_position_ekf || !_ekf_pos_valid) {
        return std::nullopt;
    }
    float max_sigma = std::max(_ekf_pos_sigma_x, _ekf_pos_sigma_y);
    if (max_sigma > _config.gps_denied.ekf_max_uncertainty) {
        return std::nullopt;
    }
    return Eigen::Vector2f{_ekf_pos_x, _ekf_pos_y};
}
```

**Changes to `updateFwNavigate()` (GPS-denied branch):**
```cpp
if (_config.gps_denied.enabled) {
    float course;
    bool wp_reached = false;

    auto dr_pos = drPosition();
    if (dr_pos.has_value()) {
        // Position-based steering: course toward WP
        const auto& wp = _waypoints[_current_wp_index];
        course = std::atan2(wp.y - dr_pos->y(), wp.x - dr_pos->x());

        // Distance-based WP acceptance (with time-based fallback)
        const float dx = wp.x - dr_pos->x();
        const float dy = wp.y - dr_pos->y();
        const float dist = std::sqrt(dx * dx + dy * dy);
        wp_reached = (dist < _config.gps_denied.ekf_wp_accept_radius);
    } else {
        // Fallback: fixed heading (current behavior)
        course = _leg_headings[_current_wp_index];
    }

    // Time-based fallback always active as safety net
    _wp_leg_elapsed += 1.f / kUpdateRate;
    if (wp_reached || _wp_leg_elapsed >= _config.gps_denied.wp_time_s) {
        // ... accept WP (existing logic)
    }

    const float height_rate = altitudeHoldVz(...);
    _fw_sp->updateWithHeightRate(height_rate, course);
    logFwStatusPeriodic(pos);
}
```

**Changes to `updateFwReturn()` (GPS-denied branch):**
```cpp
if (_config.gps_denied.enabled) {
    float course;
    bool close_enough = false;

    auto dr_pos = drPosition();
    if (dr_pos.has_value()) {
        // Position-based steering: course toward home (0,0)
        course = std::atan2(-dr_pos->y(), -dr_pos->x());

        // Distance-based MC transition
        float dist_home = dr_pos->norm();
        close_enough = (dist_home < _config.gps_denied.ekf_home_accept_radius);
    } else {
        // Fallback: fixed return heading (current behavior)
        course = _return_heading;
    }

    const float height_rate = altitudeHoldVz(...);
    _fw_sp->updateWithHeightRate(height_rate, course);
    logFwStatusPeriodic(pos);

    // Time-based fallback always active as safety net
    if (close_enough || _state_elapsed >= _config.gps_denied.return_time_s) {
        transitionTo(State::TransitionMc);
    }
}
```

**Changes to `updateMcApproach()` (GPS-denied branch):**
```cpp
if (_config.gps_denied.enabled) {
    auto dr_pos = drPosition();
    if (dr_pos.has_value()) {
        // MC approach toward home using trajectory velocity
        float course = std::atan2(-dr_pos->y(), -dr_pos->x());
        float dist = dr_pos->norm();

        if (dist < kHomeApproachDist * 2.f) {
            // Close enough — land
            transitionTo(State::Done);
        } else {
            // Fly toward home at MC approach speed
            float vn = _config.mc_approach_speed * std::cos(course);
            float ve = _config.mc_approach_speed * std::sin(course);
            float vd = altitudeHoldVz(_config.cruise_alt_m, currentAltAgl(),
                _config.gps_denied.altitude_kp, _config.gps_denied.altitude_max_vz);
            _trajectory_sp->update(
                Eigen::Vector3f{vn, ve, -vd}, std::nullopt, course);
        }
    } else {
        // No position estimate — delegate landing to executor
        transitionTo(State::Done);
    }
}
```

### Modified File: `src/vtol_navigation_node.cpp`

Add parameter declarations for new GPS-denied config:
```cpp
node->declare_parameter<bool>("gps_denied.use_position_ekf", false);
node->declare_parameter<double>("gps_denied.ekf_wp_accept_radius", 80.0);
node->declare_parameter<double>("gps_denied.ekf_home_accept_radius", 100.0);
node->declare_parameter<double>("gps_denied.ekf_max_uncertainty", 200.0);
```

Add config loading:
```cpp
config.gps_denied.use_position_ekf =
    node->get_parameter("gps_denied.use_position_ekf").as_bool();
config.gps_denied.ekf_wp_accept_radius =
    static_cast<float>(node->get_parameter("gps_denied.ekf_wp_accept_radius").as_double());
config.gps_denied.ekf_home_accept_radius =
    static_cast<float>(node->get_parameter("gps_denied.ekf_home_accept_radius").as_double());
config.gps_denied.ekf_max_uncertainty =
    static_cast<float>(node->get_parameter("gps_denied.ekf_max_uncertainty").as_double());
```

Add `#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>` to vtol_navigation_mode.hpp.

### Modified File: `CMakeLists.txt` (fiber_nav_mode)

Add `geometry_msgs` to vtol_navigation_node dependencies (already in find_package, just add to ament_target_dependencies):
```cmake
ament_target_dependencies(vtol_navigation_node
  rclcpp px4_msgs px4_ros2_cpp geometry_msgs std_msgs fiber_nav_sensors)
```
This is already the case — no change needed.

### Modified File: `test/test_vtol_navigation.cpp`

Add test cases for the new position-based steering logic:
1. `drPosition returns nullopt when disabled` — config.use_position_ekf=false
2. `drPosition returns nullopt when uncertainty too high`
3. `drPosition returns position when valid`
4. `FW_NAVIGATE course toward WP` — verify atan2 math with known position
5. `FW_RETURN course toward home` — verify course = atan2(-y, -x)
6. `distance-based WP acceptance` — verify acceptance at threshold
7. `time-based fallback still works` — WP accepted after time even without position

---

## Package: `fiber_nav_bringup` — Launch & Config

### Modified File: `config/sensor_params.yaml`

Add position_ekf_node section:
```yaml
position_ekf_node:
  ros__parameters:
    enabled: false
    publish_rate: 50.0
    q_pos: 0.1
    q_vel: 1.0
    q_wind: 0.01
    r_velocity: 0.5
    r_speed: 0.1
    of_quality_min: 0.3
    of_quality_scale: 5.0
    cable_margin: 0.95
    p0_pos: 1.0
    p0_vel: 1.0
    p0_wind: 4.0
    feed_px4: true
    position_variance_to_px4: 10.0
```

### Modified File: `config/gps_denied_mission.yaml`

Add position EKF fields:
```yaml
    gps_denied:
      enabled: true
      wp_time_s: 25.0
      return_time_s: 100.0
      descent_time_s: 60.0
      altitude_kp: 0.5
      altitude_max_vz: 3.0
      fw_speed: 18.0
      # NEW: position-based navigation
      use_position_ekf: true
      ekf_wp_accept_radius: 80.0
      ekf_home_accept_radius: 100.0
      ekf_max_uncertainty: 200.0
```

Create a new L-shaped mission config `config/gps_denied_l_mission.yaml`:
```yaml
vtol_navigation_node:
  ros__parameters:
    cruise_altitude: 30.0
    climb_rate: 4.0
    fw_accept_radius: 80.0
    mc_transition_dist: 200.0
    mc_approach_speed: 5.0
    fw_transition_timeout: 30.0
    mc_transition_timeout: 60.0

    terrain_follow:
      enabled: false

    cable_monitor:
      enabled: true
      tension_warn_percent: 70.0
      tension_abort_percent: 85.0
      breaking_strength: 50.0
      spool_capacity: 7500.0
      spool_warn_percent: 80.0
      spool_abort_percent: 95.0

    gps_denied:
      enabled: true
      wp_time_s: 30.0
      return_time_s: 120.0
      descent_time_s: 60.0
      altitude_kp: 0.5
      altitude_max_vz: 3.0
      fw_speed: 18.0
      use_position_ekf: true
      ekf_wp_accept_radius: 80.0
      ekf_home_accept_radius: 100.0
      ekf_max_uncertainty: 200.0

    # L-shaped mission: east then north
    waypoints:
      x: [0.0, 0.0, 200.0, 400.0]
      y: [200.0, 400.0, 400.0, 400.0]
```

### Modified File: `launch/sensors.launch.py`

Add position_ekf_node after fusion node:
```python
# Position EKF (GPS-denied dead reckoning)
position_ekf = TimerAction(
    period=5.0,
    actions=[
        Node(
            package='fiber_nav_fusion',
            executable='position_ekf_node',
            name='position_ekf_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(LaunchConfiguration('use_px4'))
        )
    ]
)
```

Add to LaunchDescription return list.

---

## Foxglove Layout

### Modified File: `foxglove/fiber_nav_layout.json`

Add to **Sensors tab** (or new **Navigation tab**):

**Plot!position_xy** — 2D position trajectory:
- `/position_ekf/estimate.pose.pose.position.x` (label: "North [m]", color: blue)
- `/position_ekf/estimate.pose.pose.position.y` (label: "East [m]", color: red)

**Plot!position_uncertainty** — Position uncertainty:
- Computed from covariance: sqrt(cov[0]) and sqrt(cov[7])

**Plot!wind_estimate** — Wind vector:
- `/position_ekf/wind.vector.x` (label: "Wind N", color: cyan)
- `/position_ekf/wind.vector.y` (label: "Wind E", color: magenta)

**Plot!distance_home** — Distance from home vs cable length:
- Computed from position
- `/cable/status.deployed_length` (label: "Cable deployed", color: orange)

---

## Implementation Order

1. **`position_ekf.hpp`** — Pure EKF math header
2. **`position_ekf.cpp`** — Implementation
3. **`test_position_ekf.cpp`** — Unit tests (TDD: write tests, verify they compile+fail, implement)
4. **`CMakeLists.txt` (fusion)** — Build EKF lib + test + node
5. **Build + run tests** — Verify all EKF math tests pass
6. **`position_ekf_node.cpp`** — ROS2 node wrapping the EKF
7. **`sensor_params.yaml`** — Add EKF config
8. **`sensors.launch.py`** — Add EKF node to launch
9. **`vtol_navigation_mode.hpp`** — Add position-based steering
10. **`vtol_navigation_node.cpp`** — Add new parameters
11. **`test_vtol_navigation.cpp`** — Add steering tests
12. **`gps_denied_mission.yaml`** — Enable use_position_ekf
13. **`gps_denied_l_mission.yaml`** — New L-shaped mission config
14. **Build all** — `colcon build --packages-select fiber_nav_fusion fiber_nav_mode fiber_nav_bringup`
15. **Run all tests** — Verify unit tests pass
16. **Foxglove layout** — Add position/uncertainty/wind plots
17. **E2E test: straight line** — Compare DR position vs ground truth
18. **E2E test: L-shaped** — Verify course correction at turn
19. **E2E test: return-to-home** — Measure landing distance from home

## Files Summary

| File | Action | Package |
|------|--------|---------|
| `fiber_nav_fusion/include/fiber_nav_fusion/position_ekf.hpp` | **NEW** | fusion |
| `fiber_nav_fusion/src/position_ekf.cpp` | **NEW** | fusion |
| `fiber_nav_fusion/src/position_ekf_node.cpp` | **NEW** | fusion |
| `fiber_nav_fusion/test/test_position_ekf.cpp` | **NEW** | fusion |
| `fiber_nav_fusion/CMakeLists.txt` | MODIFY | fusion |
| `fiber_nav_mode/include/fiber_nav_mode/vtol_navigation_mode.hpp` | MODIFY | mode |
| `fiber_nav_mode/src/vtol_navigation_node.cpp` | MODIFY | mode |
| `fiber_nav_mode/test/test_vtol_navigation.cpp` | MODIFY | mode |
| `fiber_nav_bringup/config/sensor_params.yaml` | MODIFY | bringup |
| `fiber_nav_bringup/config/gps_denied_mission.yaml` | MODIFY | bringup |
| `fiber_nav_mode/config/gps_denied_l_mission.yaml` | **NEW** | mode |
| `fiber_nav_bringup/launch/sensors.launch.py` | MODIFY | bringup |
| `foxglove/fiber_nav_layout.json` | MODIFY | — |

## Risk Mitigation

1. **Position EKF is disabled by default** (`enabled: false`) — existing system works unchanged
2. **Navigator keeps time-based fallback** — if EKF uncertainty exceeds threshold, falls back to fixed headings
3. **Velocity input to PX4 unchanged** — existing fusion node remains authoritative for velocity
4. **L-shaped mission is a separate config** — doesn't break existing straight-line mission
5. **Cable constraint is soft** (pseudo-measurement, not hard clamp) — won't cause EKF divergence
