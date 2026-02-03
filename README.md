# Fiber Navigation Simulation

**GPS-denied fixed-wing navigation using fiber optic cable odometry and monocular vision fusion.**

A fully Dockerized ROS 2 Jazzy / Gazebo Harmonic simulation environment for testing a novel navigation algorithm where a tethered fixed-wing drone navigates through GPS-denied environments (tunnels, canyons) using only:
- **Cable Spool Sensor**: Measures scalar velocity magnitude from fiber payout rate
- **Monocular Camera**: Provides direction of motion (unit vector) without scale

---

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Docker Services](#docker-services)
- [Architecture](#architecture)
- [Package Documentation](#package-documentation)
- [Configuration](#configuration)
- [Testing](#testing)
- [Development](#development)
- [Troubleshooting](#troubleshooting)

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

## Quick Start

### Prerequisites

- Docker and Docker Compose
- X11 server (for GUI) - Linux native or XQuartz on macOS

### Run Standalone Simulation (Recommended)

```bash
# Clone repository
git clone <repository-url> fiber-nav-sim
cd fiber-nav-sim

# Allow X11 access
xhost +local:docker

# Build and run
cd docker
docker compose build simulation
docker compose up standalone
```

This launches Gazebo Harmonic with the canyon world, sensor simulators, and fusion node using a mock attitude publisher (no PX4 required).

### Interactive Shell

```bash
docker compose run --rm simulation bash

# Inside container:
./scripts/run_standalone.sh

# In another terminal:
docker exec -it fiber-nav-sim bash
ros2 topic echo /sensors/fiber_spool/velocity
```

---

## Docker Services

All functionality is containerized. No native installation required.

| Service | Command | Description |
|---------|---------|-------------|
| `simulation` | `docker compose up simulation` | Full Gazebo GUI + ROS 2 |
| `standalone` | `docker compose up standalone` | Without PX4 (mock attitude) |
| `test` | `docker compose up test` | Build and run unit tests |
| `ci` | `docker compose up ci` | Headless CI testing |
| `foxglove` | `docker compose up foxglove` | Foxglove visualization bridge |
| `analysis` | `docker compose up analysis` | Jupyter notebook (port 8888) |

### Build Only

```bash
docker compose build simulation
```

### Run Tests

```bash
docker compose up test
# Or interactively:
docker compose run --rm simulation ./scripts/run_tests.sh
```

---

## Visualization

For real-time visualization (works on WSL/Windows without X11):

```bash
# Start simulation (headless)
docker compose up standalone -d

# Start Foxglove bridge
docker compose up foxglove -d
```

Then open [Foxglove Studio](https://studio.foxglove.dev/) in your browser:
1. Click "Open connection"
2. Select "Foxglove WebSocket"
3. Enter URL: `ws://localhost:8765`

**Useful panels:**
- **Plot**: Add `/sensors/fiber_spool/velocity.data` for speed over time
- **Raw Messages**: View `/fmu/in/vehicle_visual_odometry` fusion output
- **3D**: Visualize pose (requires TF setup)

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     Gazebo Harmonic (canyon_harmonic.sdf)                │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  plane model (OdometryPublisher plugin @ 50Hz)                    │   │
│  └────────────────────────────┬─────────────────────────────────────┘   │
└───────────────────────────────┼─────────────────────────────────────────┘
                                │
                                ▼  /model/plane/odometry (gz.msgs.Odometry)
                    ┌───────────────────────────┐
                    │      ros_gz_bridge        │
                    └───────────────┬───────────┘
                                    │
                    ▼  /model/plane/odometry (nav_msgs/Odometry)
                    │
    ┌───────────────┴───────────────┐
    │                               │
    ▼                               ▼
┌─────────────────────┐   ┌─────────────────────┐
│   spool_sim_driver  │   │ vision_direction_sim│
│                     │   │                     │
│  • Extract |v|      │   │  • Normalize to û   │
│  • Add noise σ=0.1  │   │  • Add drift walk   │
│  • Apply slack 1.05 │   │  • Body frame       │
└──────────┬──────────┘   └──────────┬──────────┘
           │                         │
           │ /sensors/fiber_spool    │ /sensors/vision
           │     /velocity           │     _direction
           │                         │
           └───────────┬─────────────┘
                       │
                       ▼
       ┌───────────────────────────────────┐
       │      fiber_vision_fusion          │
       │                                   │
       │  1. v_body = (S/slack) × û        │◄── /fmu/out/vehicle_attitude
       │  2. v_ned = q ⊗ v_body ⊗ q*       │    (PX4 or mock_attitude)
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

## Package Documentation

### fiber_nav_sensors

Synthetic sensor nodes that simulate real hardware behavior.

#### spool_sim_driver

Simulates a fiber optic spool encoder measuring payout velocity.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_topic` | string | `/model/plane/odometry` | Gazebo odometry topic |
| `noise_stddev` | double | 0.1 | Gaussian noise σ (m/s) |
| `slack_factor` | double | 1.05 | Over-payout bias |

**Noise Model:**
```
S_measured = (|v_true| + N(0, σ)) × slack_factor
```

#### vision_direction_sim

Simulates a visual odometry pipeline providing motion direction.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_topic` | string | `/model/plane/odometry` | Gazebo odometry topic |
| `drift_rate` | double | 0.001 | Random walk rate (rad/s) |
| `min_velocity` | double | 0.5 | Minimum velocity for valid direction (m/s) |

**Drift Model:**
```
Δdrift = N(0, drift_rate × √dt)
û_drifted = Rz(drift_yaw) × Ry(drift_pitch) × û_true
```

#### mock_attitude_publisher

Publishes a fixed attitude quaternion for testing without PX4.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate` | double | 50.0 | Output rate (Hz) |
| `roll` | double | 0.0 | Roll angle (rad) |
| `pitch` | double | 0.0 | Pitch angle (rad) |
| `yaw` | double | 0.0 | Yaw angle (rad) |

---

### fiber_nav_fusion

Core fusion algorithm that reconstructs velocity and publishes to PX4.

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

Gazebo Harmonic worlds and vehicle models.

**World:** `worlds/canyon_harmonic.sdf`
- SDF 1.10 format for Gazebo Harmonic
- DART physics engine
- 3000m × 500m ground plane
- 2500m canyon walls (150m high)
- Visual markers every 200m

**Model:** `models/plane/model.sdf`
- Simple fixed-wing geometry
- `gz-sim-odometry-publisher-system` plugin
- 50Hz odometry output

---

## Configuration

### PX4 Parameters

For GPS-denied operation with external vision, apply these parameters:

**File:** `src/fiber_nav_bringup/config/px4_params.txt`

```bash
# Disable GPS
param set EKF2_GPS_CTRL 0

# Enable external vision velocity
param set EKF2_EV_CTRL 4

# Vision noise settings
param set EKF2_EVV_NOISE 0.15
```

### Sensor Tuning

| Parameter | Effect | Tuning |
|-----------|--------|--------|
| `noise_stddev` | Spool noise | Match real sensor spec |
| `slack_factor` | Over-payout bias | Measure from real spool |
| `drift_rate` | Vision drift | Match VIO performance |

---

## Testing

### Unit Tests

All tests run in Docker:

```bash
# Run all tests
docker compose up test

# Or interactively
docker compose run --rm simulation ./scripts/run_tests.sh

# View results
docker compose run --rm simulation bash -c "colcon test-result --verbose"
```

### Topic Verification

```bash
# Start simulation
docker compose up standalone

# In another terminal
docker exec -it fiber-nav-standalone bash

# Check topic rates
ros2 topic hz /model/plane/odometry          # Should be ~50Hz
ros2 topic hz /sensors/fiber_spool/velocity  # Should be ~50Hz
ros2 topic hz /sensors/vision_direction      # Should be ~50Hz (when moving)
ros2 topic hz /fmu/in/vehicle_visual_odometry # Should be ~50Hz
```

### Apply Motion to Plane

```bash
# Inside container
./scripts/apply_thrust.sh 10.0 5  # 10N force for 5 seconds
```

---

## Development

### Code Style

- **C++23** standard
- `snake_case` for functions/variables
- `PascalCase` for types
- `std::expected` over exceptions
- No raw pointers in interfaces

### Rebuild in Container

```bash
docker compose run --rm simulation bash

# Inside container
cd /root/ws
colcon build --symlink-install --packages-skip px4_msgs
source install/setup.bash
```

### Adding New Sensors

1. Create node in `fiber_nav_sensors/src/`
2. Add to `CMakeLists.txt`
3. Write unit tests in `test/`
4. Add launch configuration

---

## Troubleshooting

### Gazebo Window Doesn't Appear

```bash
# On host
xhost +local:docker

# Then restart the container
docker compose down
docker compose up standalone
```

### No Odometry Data

1. Verify Gazebo is publishing: `gz topic -l | grep odometry`
2. Check bridge: `ros2 topic list | grep odometry`
3. Verify world loaded: Look for `canyon_world` in Gazebo

### Build Errors

```bash
# Clean rebuild
docker compose run --rm simulation bash -c "
  cd /root/ws
  rm -rf build install log
  colcon build --symlink-install --packages-skip px4_msgs
"
```

### High CPU / Slow Simulation

```bash
# Run headless
docker compose run --rm simulation bash -c "
  ros2 launch fiber_nav_bringup simulation.launch.py headless:=true
"
```

---

## Project Structure

```
fiber-nav-sim/
├── docker/
│   ├── Dockerfile              # ROS 2 Jazzy + Gazebo Harmonic
│   ├── docker-compose.yml      # All services
│   └── entrypoint.sh           # Environment setup
├── scripts/
│   ├── run_demo.sh             # Full demo
│   ├── run_standalone.sh       # Without PX4
│   ├── run_tests.sh            # Unit tests
│   └── apply_thrust.sh         # Test motion
├── src/
│   ├── fiber_nav_sensors/      # Sensor nodes
│   ├── fiber_nav_fusion/       # Fusion algorithm
│   ├── fiber_nav_gazebo/       # World and models
│   ├── fiber_nav_bringup/      # Launch files
│   └── fiber_nav_analysis/     # Python tools
├── docs/
│   └── PLAN.md                 # Implementation plan
└── README.md
```

---

## License

**Proprietary Software** - Copyright (c) 2026 Pavel Guzenfeld. All Rights Reserved.

---

## Contact

Pavel Guzenfeld - me@pavelguzenfeld.com
