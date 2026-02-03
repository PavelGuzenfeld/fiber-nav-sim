# Fiber Navigation Simulation - Implementation Plan

## Overview

GPS-denied fixed-wing navigation using fiber optic cable odometry + monocular vision fusion.

## Project Structure

```
fiber-nav-sim/
├── docker/
│   ├── Dockerfile              # Full simulation environment
│   ├── docker-compose.yml      # Multi-container setup
│   └── entrypoint.sh
├── src/
│   ├── fiber_nav_bringup/      # Launch files, configs
│   ├── fiber_nav_gazebo/       # World, models, URDF
│   ├── fiber_nav_sensors/      # Synthetic sensors
│   ├── fiber_nav_fusion/       # Core algorithm
│   ├── fiber_nav_msgs/         # Custom messages (separate for clean deps)
│   └── fiber_nav_analysis/     # Python plotting
├── config/
│   ├── px4_params/             # PX4 parameter files
│   └── ekf_config.yaml         # EKF tuning
├── test/
│   ├── unit/                   # Unit tests
│   └── integration/            # Full pipeline tests
├── docs/
│   ├── PLAN.md                 # This file
│   └── MATH.md                 # Mathematical derivations
└── README.md
```

---

## Phase 1: Environment & Vehicle

### 1.1 Canyon World (Gazebo)

**File**: `src/fiber_nav_gazebo/worlds/canyon.world`

Features:
- 2km+ procedurally generated canyon
- Texture-rich rock walls (high-frequency features for VO)
- Variable width (50-200m) with gentle curves
- Ground plane with texture
- Ambient lighting simulating overcast sky

Technical approach:
- Use SDF `<heightmap>` or `<mesh>` for terrain
- Apply rock/concrete textures from Gazebo model database
- Add visual noise (debris, vegetation sprites) for feature tracking

### 1.2 Fixed-Wing Vehicle Model

**File**: `src/fiber_nav_gazebo/models/fiber_plane/model.sdf`

Modifications to standard PX4 plane:
- Forward-facing camera sensor (640x480, 60 FOV, 30Hz)
- Camera link at nose (pitched down 10°)
- Fiber attachment point (visual only - no physics)

```xml
<link name="camera_link">
  <pose>0.5 0 0.1 0 0.17 0</pose>  <!-- 10° pitch down -->
  <sensor name="front_camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip><near>0.1</near><far>500</far></clip>
    </camera>
    <update_rate>30</update_rate>
  </sensor>
</link>
```

### 1.3 PX4 SITL Configuration

**File**: `config/px4_params/fiber_nav.params`

Key parameters:
```
# Disable GPS
SYS_HAS_GPS=0
EKF2_GPS_CTRL=0

# Enable external vision
EKF2_EV_CTRL=7          # Velocity fusion
EKF2_EV_DELAY=50        # ms - tune based on pipeline latency
EKF2_EV_NOISE_VD=0.1    # Velocity noise (match spool σ)

# Fixed-wing specific
FW_AIRSPD_MIN=12        # m/s
FW_AIRSPD_TRIM=18       # m/s
```

---

## Phase 2: Synthetic Spool Sensor

### 2.1 Spool Sim Driver Node

**File**: `src/fiber_nav_sensors/src/spool_sim_driver.cpp`

State machine:
```
IDLE → SPOOLING → TENSION_LOSS → RECOVERY
```

Noise model:
```cpp
// Gaussian noise
noise = N(0, σ=0.1 m/s)

// Bias (over-payout)
bias_factor = 1.05  // spool rotates faster than drone flies

// Quantization (12-bit encoder)
quantization = round(speed * 4096) / 4096

// Temperature drift (optional)
temp_drift = 0.001 * (temperature - 20)  // m/s per °C deviation
```

Failure modes to simulate:
- **Slack detection**: When `speed < 0.5 m/s`, spool may have slack
- **Tension spike**: When `d(speed)/dt > 5 m/s²`, cable may be caught
- **Spool limit**: Maximum payout of 3000m

### 2.2 Unit Tests

**File**: `test/unit/test_spool_sensor.cpp`

```cpp
TEST(SpoolSensor, NoiseDistribution) {
  // Verify noise is Gaussian with correct σ
}

TEST(SpoolSensor, BiasApplication) {
  // Input: 10 m/s true
  // Expected: 10.5 m/s (±noise)
}

TEST(SpoolSensor, ZeroVelocity) {
  // Should not produce negative values
}

TEST(SpoolSensor, SlackDetection) {
  // Low velocity should trigger slack warning
}
```

---

## Phase 3: Synthetic Vision Sensor

### 3.1 Vision Direction Node

**File**: `src/fiber_nav_sensors/src/vision_direction_sim.cpp`

Drift model (random walk):
```cpp
// Continuous-time random walk
drift_rate = 0.001 rad/s  // ~0.06°/s

// Per timestep
Δdrift = N(0, drift_rate * √dt)
total_drift += Δdrift

// Apply as rotation to unit vector
direction_drifted = Rz(drift_yaw) * Ry(drift_pitch) * direction_true
```

Edge cases:
- **Low velocity**: Direction undefined when `||v|| < 0.5 m/s`
- **Pure vertical**: Gimbal lock handling
- **180° ambiguity**: Forward/backward detection

### 3.2 Unit Tests

**File**: `test/unit/test_vision_sensor.cpp`

```cpp
TEST(VisionSensor, UnitVector) {
  // Output must have magnitude 1.0 (±ε)
}

TEST(VisionSensor, DriftBounds) {
  // After 1000s, drift should be bounded (√1000 * rate)
}

TEST(VisionSensor, LowVelocityReject) {
  // Should not publish when speed < threshold
}
```

---

## Phase 4: Fusion Node

### 4.1 Core Algorithm

**File**: `src/fiber_nav_fusion/src/fiber_vision_fusion.cpp`

Algorithm:
```
1. Receive spool_velocity (scalar S)
2. Receive vision_direction (unit vector û in body frame)
3. Receive attitude (quaternion q from IMU)

4. Reconstruct body velocity:
   v_body = (S / slack_factor) * û

5. Rotate to NED frame:
   v_ned = q ⊗ v_body ⊗ q*

6. Publish to PX4 EKF:
   /fmu/in/vehicle_visual_odometry
   - velocity = v_ned
   - position = NaN (unknown)
   - covariance = diag(σ_spool², σ_spool², σ_spool²)
```

### 4.2 Slack Factor Estimation (Advanced)

Optional adaptive estimation:
```cpp
// Compare fused velocity to IMU-integrated velocity
// Adjust slack_factor to minimize divergence

error = ||v_fused - v_imu_integrated||
slack_factor += learning_rate * error * sign(divergence)
```

### 4.3 Timing Synchronization

Critical: Sensors have different latencies

```cpp
// Use message_filters for time sync
sync.registerCallback([](const SpoolMsg& spool,
                         const DirectionMsg& dir,
                         const AttitudeMsg& att) {
  // All messages within 20ms of each other
});
```

### 4.4 Unit Tests

**File**: `test/unit/test_fusion.cpp`

```cpp
TEST(Fusion, VelocityReconstruction) {
  // spool=10, dir=(1,0,0) → v=(10,0,0)
}

TEST(Fusion, SlackCorrection) {
  // spool=10.5, slack=1.05 → corrected=10.0
}

TEST(Fusion, FrameRotation) {
  // Body velocity + 90° yaw → NED rotated 90°
}

TEST(Fusion, StaleDataRejection) {
  // Old data should not be used
}

TEST(Fusion, CovarianceCorrect) {
  // Output covariance matches sensor noise
}
```

---

## Phase 5: Analysis & Metrics

### 5.1 Trajectory Recording

**File**: `src/fiber_nav_analysis/scripts/record_trajectory.py`

Record to CSV/ROS2 bag:
- Ground truth: `/gazebo/model_states`
- Estimated: `/fmu/out/vehicle_local_position`
- Timestamps: synchronized

### 5.2 Plotting Script

**File**: `src/fiber_nav_analysis/scripts/plot_trajectory.py`

Plots:
1. **2D trajectory** (X-Y plane): GT vs Estimated
2. **3D trajectory**: Full path with drift visualization
3. **Velocity comparison**: Time series
4. **Error over distance**: Drift accumulation
5. **Allan variance**: Characterize noise

### 5.3 Metrics

```python
def drift_per_1000m(gt_trajectory, est_trajectory):
    """
    Calculate position error per 1000m traveled
    """
    total_distance = path_length(gt_trajectory)
    final_error = ||gt_trajectory[-1] - est_trajectory[-1]||
    return (final_error / total_distance) * 1000

def velocity_rmse(gt_vel, est_vel):
    """
    Root mean square velocity error
    """
    return sqrt(mean((gt_vel - est_vel)²))
```

### 5.4 Baseline Comparisons

Run 3 scenarios:
1. **IMU only** (no fusion): Expect massive drift (>100m/km)
2. **Spool only** (no direction): Dead reckoning, heading drift
3. **Full fusion**: Target <5m/km drift

---

## Phase 6: Testing Infrastructure

### 6.1 Unit Test Framework

Using `ament_cmake_gtest`:

```cmake
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_spool test/unit/test_spool_sensor.cpp)
ament_add_gtest(test_vision test/unit/test_vision_sensor.cpp)
ament_add_gtest(test_fusion test/unit/test_fusion.cpp)
```

### 6.2 Integration Tests

**File**: `test/integration/test_full_pipeline.py`

```python
def test_straight_flight():
    """
    Fly straight 1km, verify drift < 10m
    """
    launch_simulation()
    command_straight_flight(distance=1000)
    wait_for_completion()

    gt = get_ground_truth()
    est = get_estimate()

    assert drift_per_1000m(gt, est) < 10.0

def test_canyon_navigation():
    """
    Navigate full canyon with curves
    """
    launch_simulation()
    command_waypoint_mission()
    wait_for_completion()

    assert final_position_error() < 50.0  # meters
```

### 6.3 CI Pipeline

**File**: `.github/workflows/ci.yml`

```yaml
jobs:
  build:
    runs-on: ubuntu-22.04
    container: ghcr.io/thebandofficial/fiber-nav-sim:latest
    steps:
      - uses: actions/checkout@v4
      - name: Build
        run: colcon build
      - name: Unit Tests
        run: colcon test
      - name: Integration Tests
        run: ros2 launch fiber_nav_bringup test_integration.launch.py
```

---

## Phase 7: Docker Environment

### 7.1 Dockerfile

**File**: `docker/Dockerfile`

Base image: `px4io/px4-dev-ros2-humble`

Additions:
- Gazebo Classic/Garden
- PX4 SITL
- Project dependencies
- Headless rendering (VirtualGL/Mesa)

### 7.2 Docker Compose

**File**: `docker/docker-compose.yml`

Services:
- `gazebo`: Simulation environment
- `px4`: SITL autopilot
- `ros2`: ROS 2 nodes
- `analysis`: Jupyter notebook for post-processing

---

## Implementation Order

### Week 1: Foundation
- [x] Project structure
- [x] Basic sensor nodes (spool, vision)
- [x] Basic fusion node
- [ ] Unit tests for sensor math

### Week 2: Simulation Environment
- [ ] Canyon world generation
- [ ] Vehicle model with camera
- [ ] PX4 SITL integration
- [ ] Dockerfile

### Week 3: Integration
- [ ] Full pipeline launch files
- [ ] Time synchronization
- [ ] PX4 EKF configuration
- [ ] Integration tests

### Week 4: Analysis & Tuning
- [ ] Recording/plotting scripts
- [ ] Baseline comparisons
- [ ] Parameter tuning
- [ ] Documentation

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| PX4 EKF rejects velocity | Start with loose covariance, tighten gradually |
| Gazebo ground truth lag | Use Gazebo transport directly, not ROS bridge |
| Body→NED rotation error | Extensive unit testing, compare to IMU integration |
| Drift accumulation | Implement loop closure detection (future) |
| Real-time performance | Profile nodes, use intra-process communication |

---

## Success Criteria

| Metric | Target | Stretch |
|--------|--------|---------|
| Drift per 1000m | <10m | <5m |
| Velocity RMSE | <0.5 m/s | <0.2 m/s |
| CPU usage (fusion node) | <5% | <2% |
| Latency (sensor→EKF) | <50ms | <20ms |
| Build time | <5min | <2min |
