# Fiber Navigation Simulation

**GPS-denied fixed-wing navigation using fiber optic cable odometry and monocular vision fusion.**

A ROS 2 / Gazebo simulation environment for testing a novel navigation algorithm where a tethered fixed-wing drone navigates through GPS-denied environments (tunnels, canyons) using only:
- **Cable Spool Sensor**: Measures scalar velocity magnitude from fiber payout rate
- **Monocular Camera**: Provides direction of motion (unit vector) without scale

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Requirements](#requirements)
- [Installation](#installation)
  - [Docker (Recommended)](#docker-recommended)
  - [Native Installation](#native-installation)
- [Quick Start](#quick-start)
- [Package Documentation](#package-documentation)
- [Configuration](#configuration)
- [Testing](#testing)
- [Analysis & Metrics](#analysis--metrics)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

### The Problem

Fixed-wing drones in GPS-denied environments (tunnels, canyons, indoor) cannot rely on satellite navigation. Traditional solutions like Visual-Inertial Odometry (VIO) suffer from scale drift over long distances.

### The Solution

A fiber-tethered drone with two complementary sensors:

| Sensor | Provides | Limitation |
|--------|----------|------------|
| **Fiber Spool** | Absolute velocity magnitude (m/s) | No direction information |
| **Monocular Camera** | Direction of motion (unit vector) | No scale/magnitude |

By fusing these sensors, we reconstruct the full 3D velocity vector and feed it to the PX4 EKF, replacing GPS velocity measurements.

### Key Innovation

```
v_estimated = (spool_velocity / slack_factor) × direction_unit_vector
```

This decouples the hard problem (physical rope simulation) from the navigation algorithm, allowing rapid iteration on the fusion logic.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Gazebo Simulation                               │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────────────┐ │
│  │  Fixed-Wing  │  │    Canyon    │  │       Ground Truth             │ │
│  │    Model     │  │    World     │  │  /gazebo/model_states          │ │
│  └──────────────┘  └──────────────┘  └───────────────┬────────────────┘ │
└──────────────────────────────────────────────────────┼──────────────────┘
                                                       │
                    ┌──────────────────────────────────┼──────────────────────────────┐
                    │                                  │                              │
                    ▼                                  ▼                              ▼
        ┌─────────────────────┐          ┌─────────────────────┐          ┌─────────────────────┐
        │   spool_sim_driver  │          │ vision_direction_sim│          │      PX4 SITL       │
        │                     │          │                     │          │                     │
        │  • Extract |v|      │          │  • Normalize to û   │          │  • IMU data         │
        │  • Add noise σ=0.1  │          │  • Add drift walk   │          │  • Attitude q       │
        │  • Apply slack 1.05 │          │  • Body frame       │          │  • EKF fusion       │
        └──────────┬──────────┘          └──────────┬──────────┘          └──────────┬──────────┘
                   │                                │                                │
                   │ /sensors/fiber_spool           │ /sensors/vision               │ /fmu/out/*
                   │     /velocity                  │     _direction                │
                   │                                │                                │
                   └───────────────┬────────────────┴────────────────────────────────┘
                                   │
                                   ▼
                   ┌───────────────────────────────────┐
                   │      fiber_vision_fusion          │
                   │                                   │
                   │  1. v_body = (S/slack) × û        │
                   │  2. v_ned = q ⊗ v_body ⊗ q*       │
                   │  3. Publish to PX4 EKF            │
                   └───────────────┬───────────────────┘
                                   │
                                   ▼
                   ┌───────────────────────────────────┐
                   │  /fmu/in/vehicle_visual_odometry  │
                   │                                   │
                   │  • velocity = v_ned               │
                   │  • position = NaN (unknown)       │
                   │  • covariance = sensor noise      │
                   └───────────────────────────────────┘
```

---

## Requirements

### Hardware
- 8GB+ RAM (16GB recommended for Gazebo)
- GPU with OpenGL support (for visualization)

### Software
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (11.x) or Gazebo Garden
- PX4 Autopilot v1.14+
- Docker (optional but recommended)

---

## Installation

### Docker (Recommended)

The Docker environment includes all dependencies pre-configured.

```bash
# Clone repository
cd ~/workspace
git clone <repository-url> fiber-nav-sim
cd fiber-nav-sim

# Build Docker image
cd docker
docker-compose build

# Run simulation
docker-compose up simulation
```

**Docker Services:**

| Service | Description | Port |
|---------|-------------|------|
| `simulation` | Full simulation (Gazebo + PX4 + ROS 2) | 14540 (MAVLink) |
| `analysis` | Jupyter notebook for post-processing | 8888 |
| `ci` | Headless testing for CI/CD | - |

### Native Installation

#### 1. Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs
```

#### 2. Install PX4

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch v1.14.0
cd PX4-Autopilot
make px4_sitl_default
```

#### 3. Build px4_msgs

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git --branch release/1.14
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
```

#### 4. Build Fiber Nav Simulation

```bash
cd ~/workspace/fiber-nav-sim
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start

### Terminal 1: Launch PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_plane
```

### Terminal 2: Launch ROS 2 Nodes

```bash
source ~/workspace/fiber-nav-sim/install/setup.bash
ros2 launch fiber_nav_bringup simulation.launch.py
```

### Terminal 3: Monitor Topics

```bash
# Spool velocity
ros2 topic echo /sensors/fiber_spool/velocity

# Vision direction
ros2 topic echo /sensors/vision_direction

# Fused odometry
ros2 topic echo /fmu/in/vehicle_visual_odometry
```

### Terminal 4: Record Data

```bash
ros2 run fiber_nav_analysis record_trajectory
```

---

## Package Documentation

### fiber_nav_sensors

Synthetic sensor nodes that simulate real hardware behavior.

#### spool_sim_driver

Simulates a fiber optic spool encoder measuring payout velocity.

**Subscriptions:**
- `/gazebo/model_states` (gazebo_msgs/ModelStates)

**Publications:**
- `/sensors/fiber_spool/velocity` (std_msgs/Float32)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | string | "plane" | Gazebo model name |
| `noise_stddev` | double | 0.1 | Gaussian noise σ (m/s) |
| `slack_factor` | double | 1.05 | Over-payout bias |

**Noise Model:**
```
S_measured = (|v_true| + N(0, σ)) × slack_factor
```

#### vision_direction_sim

Simulates a visual odometry pipeline providing motion direction.

**Subscriptions:**
- `/gazebo/model_states` (gazebo_msgs/ModelStates)

**Publications:**
- `/sensors/vision_direction` (geometry_msgs/Vector3Stamped)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_name` | string | "plane" | Gazebo model name |
| `drift_rate` | double | 0.001 | Random walk rate (rad/s) |
| `min_velocity` | double | 0.5 | Minimum velocity for valid direction (m/s) |

**Drift Model:**
```
Δdrift = N(0, drift_rate × √dt)
û_drifted = Rz(drift_yaw) × Ry(drift_pitch) × û_true
```

---

### fiber_nav_fusion

Core fusion algorithm that reconstructs velocity and publishes to PX4.

#### fiber_vision_fusion

**Subscriptions:**
- `/sensors/fiber_spool/velocity` (std_msgs/Float32)
- `/sensors/vision_direction` (geometry_msgs/Vector3Stamped)
- `/fmu/out/vehicle_attitude` (px4_msgs/VehicleAttitude)

**Publications:**
- `/fmu/in/vehicle_visual_odometry` (px4_msgs/VehicleOdometry)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `slack_factor` | double | 1.05 | Slack correction factor |
| `publish_rate` | double | 50.0 | Output rate (Hz) |
| `max_data_age` | double | 0.1 | Maximum sensor data age (s) |

**Algorithm:**
```cpp
// 1. Reconstruct body-frame velocity
v_body = (spool_velocity / slack_factor) × direction_unit_vector

// 2. Rotate to NED frame using attitude quaternion
v_ned = quaternion_rotate(attitude_q, v_body)

// 3. Publish to PX4 EKF
odometry.velocity = v_ned
odometry.position = NaN  // Unknown
odometry.velocity_variance = [0.01, 0.01, 0.01]  // From sensor noise
```

---

### fiber_nav_gazebo

Gazebo worlds and vehicle models.

#### Canyon World

A 2km+ canyon environment with:
- Texture-rich rock walls (for visual features)
- Variable width (50-200m)
- Gentle curves for heading drift testing
- Visual markers at 200m, 500m, 800m

**File:** `worlds/canyon.world`

---

### fiber_nav_bringup

Launch files and configuration.

#### Launch Files

| File | Description |
|------|-------------|
| `simulation.launch.py` | Full simulation stack |
| `test_sensors.launch.py` | Sensors only (requires external Gazebo) |

#### Configuration

**File:** `config/sensor_params.yaml`

```yaml
spool_sim_driver:
  ros__parameters:
    model_name: "plane"
    noise_stddev: 0.1
    slack_factor: 1.05

vision_direction_sim:
  ros__parameters:
    model_name: "plane"
    drift_rate: 0.001
    min_velocity: 0.5

fiber_vision_fusion:
  ros__parameters:
    slack_factor: 1.05
    publish_rate: 50.0
    max_data_age: 0.1
```

---

### fiber_nav_analysis

Python tools for trajectory analysis.

#### record_trajectory

Records ground truth and estimated trajectories to CSV.

```bash
ros2 run fiber_nav_analysis record_trajectory \
  --ros-args -p model_name:=plane -p output_dir:=/tmp/data
```

**Output Files:**
- `ground_truth_<timestamp>.csv`
- `estimated_<timestamp>.csv`

#### plot_trajectory

Generates trajectory comparison plots.

```bash
ros2 run fiber_nav_analysis plot_trajectory \
  --gt /tmp/data/ground_truth_*.csv \
  --est /tmp/data/estimated_*.csv \
  --output-dir /tmp/plots
```

**Output Plots:**
- `trajectory_2d.png` - X-Y plane comparison
- `trajectory_3d.png` - 3D visualization
- `error_vs_distance.png` - Drift accumulation
- `velocity_comparison.png` - Time series
- `report.txt` - Text summary

#### compute_metrics

Computes navigation metrics.

```bash
ros2 run fiber_nav_analysis compute_metrics \
  --gt ground_truth.csv --est estimated.csv --format json
```

**Metrics:**
- Drift per 1000m (target: <10m)
- Velocity RMSE (target: <0.5 m/s)
- Maximum position error
- Mean position error

---

## Configuration

### PX4 Parameters

For GPS-denied operation, configure PX4 EKF:

```bash
# Disable GPS
param set SYS_HAS_GPS 0
param set EKF2_GPS_CTRL 0

# Enable external vision velocity
param set EKF2_EV_CTRL 7
param set EKF2_EV_DELAY 50
param set EKF2_EV_NOISE_VD 0.1
```

**File:** `config/px4_params/fiber_nav.params`

### Sensor Tuning

| Parameter | Effect | Tuning |
|-----------|--------|--------|
| `noise_stddev` | Spool noise | Match real sensor spec |
| `slack_factor` | Over-payout bias | Measure from real spool |
| `drift_rate` | Vision drift | Match VIO performance |

---

## Testing

### Unit Tests

```bash
# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Run all tests
colcon test

# View results
colcon test-result --verbose
```

### Test Coverage

| Test File | Coverage |
|-----------|----------|
| `test_spool_sensor.cpp` | Noise distribution, bias, clamping |
| `test_vision_sensor.cpp` | Unit vector, drift, low velocity |
| `test_fusion.cpp` | Reconstruction, rotation, covariance |

### Integration Tests

```bash
ros2 launch fiber_nav_bringup test_integration.launch.py
```

---

## Analysis & Metrics

### Success Criteria

| Metric | Target | Stretch Goal |
|--------|--------|--------------|
| Drift per 1000m | <10m | <5m |
| Velocity RMSE | <0.5 m/s | <0.2 m/s |
| Latency | <50ms | <20ms |

### Baseline Comparison

Run three scenarios to validate fusion:

1. **IMU Only** (GPS disabled)
   - Expected: >100m drift per 1000m

2. **Spool Only** (no direction)
   - Expected: Heading drift accumulation

3. **Full Fusion** (spool + vision)
   - Expected: <10m drift per 1000m

### Generate Report

```bash
python3 -m fiber_nav_analysis.plot_trajectory \
  --gt ground_truth.csv \
  --est estimated.csv \
  --output-dir ./results
```

---

## Development

### Code Style

- **C++23** standard
- `snake_case` for functions/variables
- `PascalCase` for types
- `std::expected` over exceptions
- No raw pointers in interfaces

### Adding New Sensors

1. Create node in `fiber_nav_sensors/src/`
2. Add to `CMakeLists.txt`
3. Write unit tests in `test/`
4. Add launch configuration

### Debugging

```bash
# Verbose logging
ros2 launch fiber_nav_bringup simulation.launch.py \
  --ros-args --log-level debug

# Topic monitoring
ros2 topic hz /sensors/fiber_spool/velocity
ros2 topic delay /fmu/in/vehicle_visual_odometry
```

---

## Troubleshooting

### PX4 EKF Rejects Velocity

**Symptom:** EKF status shows vision rejected

**Solution:**
1. Check covariance values (too low = rejected)
2. Verify timestamp synchronization
3. Increase `EKF2_EV_NOISE_VD`

### No Ground Truth Data

**Symptom:** Spool/vision nodes show warnings

**Solution:**
1. Verify Gazebo is publishing `/gazebo/model_states`
2. Check `model_name` parameter matches Gazebo

### High CPU Usage

**Symptom:** Real-time factor <1.0

**Solution:**
1. Reduce sensor publish rates
2. Use headless Gazebo: `gzserver` instead of `gazebo`
3. Disable unnecessary visualizations

### Docker Display Issues

**Symptom:** Gazebo window doesn't appear

**Solution:**
```bash
xhost +local:docker
docker-compose up simulation
```

---

## Project Structure

```
fiber-nav-sim/
├── docker/
│   ├── Dockerfile              # Full simulation environment
│   ├── docker-compose.yml      # Multi-service configuration
│   └── entrypoint.sh           # Container initialization
├── docs/
│   └── PLAN.md                 # Detailed implementation plan
├── src/
│   ├── fiber_nav_sensors/      # Synthetic sensor nodes
│   │   ├── src/
│   │   │   ├── spool_sim_driver.cpp
│   │   │   └── vision_direction_sim.cpp
│   │   ├── test/
│   │   │   ├── test_spool_sensor.cpp
│   │   │   └── test_vision_sensor.cpp
│   │   ├── msg/
│   │   │   └── SpoolVelocity.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── fiber_nav_fusion/       # Core fusion algorithm
│   │   ├── src/
│   │   │   └── fiber_vision_fusion.cpp
│   │   ├── test/
│   │   │   └── test_fusion.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── fiber_nav_gazebo/       # Simulation environment
│   │   ├── worlds/
│   │   │   └── canyon.world
│   │   ├── models/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── fiber_nav_bringup/      # Launch and config
│   │   ├── launch/
│   │   │   ├── simulation.launch.py
│   │   │   └── test_sensors.launch.py
│   │   ├── config/
│   │   │   └── sensor_params.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── fiber_nav_analysis/     # Analysis tools
│       ├── fiber_nav_analysis/
│       │   ├── __init__.py
│       │   ├── record_trajectory.py
│       │   ├── plot_trajectory.py
│       │   └── compute_metrics.py
│       ├── setup.py
│       └── package.xml
├── .gitignore
├── LICENSE
└── README.md
```

---

## License

**Proprietary Software** - Copyright (c) 2026 Pavel Guzenfeld. All Rights Reserved.

See [LICENSE](LICENSE) for details.

---

## Contact

Pavel Guzenfeld - pavelguzenfeld@gmail.com
