# Fiber Navigation Simulation

**GPS-denied VTOL navigation using fiber optic cable odometry and monocular vision fusion.**

A fully Dockerized ROS 2 Jazzy / Gazebo Harmonic simulation environment for testing a novel navigation algorithm where a tethered quad-tailsitter VTOL drone navigates through GPS-denied environments (tunnels, canyons) using only:
- **Cable Spool Sensor**: Measures scalar velocity magnitude from fiber payout rate
- **Monocular Camera**: Provides direction of motion (unit vector) without scale

---

## Benchmark Results

### Short Flight (15s, ~300m)
| Method | Velocity RMSE | Position Error | Drift Rate |
|--------|---------------|----------------|------------|
| GPS | 0.26 m/s | 2.3m | ~0 (bounded) |
| **Fiber+Vision** | **0.96 m/s** | 14.5m | 48 m/km |
| IMU-only | 0.20 m/s | 2.7m | quadratic |

### Long Distance (20km, ~17 minutes)
| Method | Velocity RMSE | Position Error | vs IMU |
|--------|---------------|----------------|--------|
| GPS | 0.26 m/s | 1.6m | reference |
| **Fiber+Vision** | 0.96 m/s | 952m (4.8%) | **12x better** |
| IMU-only | 13.82 m/s | 11,979m (60%) | - |

**Key findings:**
- Fiber+Vision achieves **12x less position error** than IMU-only over 20km
- Position drift is linear (Fiber+Vision) vs quadratic (IMU)
- Viable for GPS-denied long-distance navigation

See `scripts/compare_three_way.py` and `scripts/generate_20km_flight.py` for analysis.

---

## Project Status

| Component | Status | Notes |
|-----------|--------|-------|
| Gazebo simulation | Done | Canyon world, quadtailsitter VTOL model |
| Sensor simulation | Done | Spool + vision + IMU/baro/mag |
| Fusion algorithm | Done | Body-to-NED transform, 0.13 m/s RMSE |
| Stabilized flight | Done | PD wrench controller, 50m altitude, racetrack waypoints |
| PX4 integration | Done | Custom airframe 4251, sensor bridges, px4-ros2-interface-lib |
| Custom flight modes | Done | HoldMode + CanyonMission via px4-ros2-interface-lib |
| Foxglove visualization | Done | Integrated in launch file (default enabled, port 8765) |
| Unit tests | Done | 60 tests passing (spool, vision, fusion, flight controller, waypoints, integration, analysis) |
| Integration tests | Done | Stabilized flight stability verification in Gazebo |
| Benchmarking | Done | 20km 3-way comparison |

---

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Docker Services](#docker-services)
- [Architecture](#architecture)
- [Vehicle Model](#vehicle-model)
- [Package Documentation](#package-documentation)
- [PX4 Custom Flight Modes](#px4-custom-flight-modes)
- [Visualization](#visualization)
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
v_estimated = (spool_velocity / slack_factor) * direction_unit_vector
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

This launches Gazebo Harmonic with the canyon world, quadtailsitter model, stabilized flight controller (PD wrench control at 50m altitude), sensor simulators, fusion node, and Foxglove bridge — no PX4 required.

### Run with PX4 SITL

PX4 requires a real TTY and must be started manually. See [Integration Testing](#integration-testing) below.

```bash
# Terminal 1: Start Gazebo + sensors (headless)
docker compose run --rm px4-sitl bash

# Inside: follow the multi-terminal instructions in Integration Testing section
```

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
| `standalone` | `docker compose up standalone` | Headless, no PX4 (mock attitude, auto-fly) |
| `px4-sitl` | `docker compose up px4-sitl` | Headless Gazebo for PX4 SITL mode |
| `test` | `docker compose up test` | Build and run unit tests |
| `ci` | `docker compose up ci` | Headless CI testing |
| `foxglove` | `docker compose up foxglove` | Standalone Foxglove bridge (now integrated in launch) |
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

Foxglove bridge is integrated into the launch file (default enabled on port 8765). No separate service needed:

```bash
# Start simulation — Foxglove bridge starts automatically
docker compose up standalone -d
```

Then open [Foxglove Studio](https://studio.foxglove.dev/) in your browser:
1. Click "Open connection"
2. Select "Foxglove WebSocket"
3. Enter URL: `ws://localhost:8765`
4. Import layout from `foxglove/fiber_nav_layout.json`

To disable Foxglove: add `foxglove:=false` to the launch command.

The layout shows:
- **Follow camera**: 3rd person chase view attached to the vehicle
- **Forward camera**: Nose-mounted forward-looking view
- **Down camera**: Downward-facing ground tracking view
- **Plots**: Spool velocity vs ground truth, vision direction, position, velocity
- **Raw data**: Fusion output and odometry messages

---

## Architecture

### Standalone Mode (wrench-based flight)

```
+-----------------------------------------------------------------------------+
|                   Gazebo Harmonic (canyon_harmonic.sdf)                      |
|  +-----------------------------------------------------------------------+  |
|  |  quadtailsitter model (fixed joints, wrench-controlled)               |  |
|  |  OdometryPublisher @ 50Hz, IMU @ 250Hz, Baro/Mag @ 50Hz             |  |
|  |  Cameras: forward, down, follow (attached to base_link)              |  |
|  +---------------------------+-------------------------------------------+  |
+------------------------------+----------------------------------------------+
                               |
                               v  /model/quadtailsitter/odometry
                 +----------------------------+
                 |       ros_gz_bridge        |
                 +-+------------+----------+--+
                   |            |          |
                   v            v          v
+------------------+--+  +-----+------+  +----+---------------------+
|  spool_sim_driver   |  | vision_    |  | stabilized_flight_       |
|  * Extract |v|      |  | direction  |  | controller               |
|  * Add noise s=0.1  |  | _sim       |  |                          |
|  * Apply slack 1.05 |  | * Unit dir |  |  * PD attitude control   |
+----------+----------+  | * Drift    |  |  * Altitude hold (50m)   |
           |              +-----+------+  |  * Racetrack waypoints   |
           |                    |         |  * One-time gz wrench    |
           v                    v         +-------+------------------+
  /sensors/fiber_spool  /sensors/vision           |
      /velocity           _direction              | /world/.../wrench
           |                    |                  | (gz transport)
           +--------+-----------+                  v
                    |                        Gazebo Physics
                    v
    +-----------------------------------+
    |      fiber_vision_fusion          |
    |                                   |
    |  1. v_body = (S/slack) * u        |<-- /fmu/out/vehicle_attitude
    |  2. v_ned = q * v_body * q*       |    (mock_attitude_publisher)
    |  3. Publish visual odometry       |
    +---------------+-------------------+
                    |
                    v
    +-----------------------------------+
    |  /fmu/in/vehicle_visual_odometry  |       +-------------------+
    |  * velocity = v_ned               |       | foxglove_bridge   |
    |  * position = NaN (unknown)       |       | ws://localhost:   |
    |  * covariance = sensor noise      |       |         8765      |
    +-----------------------------------+       +-------------------+
```

### PX4 Mode (with custom flight modes)

```
+-----------------------------------+
|  PX4 SITL (airframe 4251)        |
|  * GPS-denied (SYS_HAS_GPS=0)    |
|  * Vision velocity (EKF2_EV_CTRL) |
+-------+-------+------------------+
        |       ^
  motors|       | /fmu/in/vehicle_visual_odometry
        v       |
+-----------------------------------+     +----------------------------+
| Gazebo Harmonic (quadtailsitter) |---->| fiber_vision_fusion        |
| * Revolute joints + motor plugins|     +----------------------------+
| * AdvancedLiftDrag aerodynamics  |
| * IMU, Baro, Mag, Cameras       |     +----------------------------+
+-----------------------------------+     | px4-ros2-interface-lib     |
                                          | * HoldMode (FiberNav Hold) |
                                          | * CanyonMission (executor) |
                                          +----------------------------+
```

> **Note:** The quadtailsitter model has two configurations. In standalone mode (wrench-based),
> motor and aero plugins are disabled and rotor joints are fixed — flight is controlled via
> the `stabilized_flight_controller` applying one-time wrenches. In PX4 mode, motors and
> AdvancedLiftDrag are re-enabled for PX4-controlled flight. See
> [Gazebo Wrench Lessons](docs/GAZEBO_WRENCH_LESSONS.md) for details.

---

## Vehicle Model

### Quadtailsitter VTOL

The simulation uses a **quadtailsitter** VTOL model (`models/quadtailsitter/model.sdf`):

| Property | Value |
|----------|-------|
| Mass | 1.635 kg (base 1.6 + rotors + airspeed) |
| Inertia | Ixx=0.113, Iyy=0.030, Izz=0.084 |
| Rotor joints | Fixed (standalone) / Revolute (PX4) |
| IMU | 250 Hz, Gaussian noise |
| Barometer | 50 Hz |
| Magnetometer | 50 Hz |
| Forward camera | 640x480, 30 Hz |
| Down camera | 640x480, 30 Hz |
| Follow camera | 1280x720, 30 Hz (5m behind, 2m above) |

The model has two operating modes:
- **Standalone mode**: Fixed rotor joints, no aero/motor plugins. Flight controlled by `stabilized_flight_controller` applying one-time wrenches via Gazebo transport
- **PX4 mode**: Revolute rotor joints, AdvancedLiftDrag aerodynamics, 4x MulticopterMotorModel. PX4 controls motors directly
- PX4-compatible sensor suite (IMU, baro, mag)
- 3 cameras attached to base_link (forward, down, follow)

---

## Package Documentation

### fiber_nav_sensors

Synthetic sensor nodes that simulate real hardware behavior.

#### spool_sim_driver

Simulates a fiber optic spool encoder measuring payout velocity.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_topic` | string | `/model/quadtailsitter/odometry` | Gazebo odometry topic |
| `noise_stddev` | double | 0.1 | Gaussian noise (m/s) |
| `slack_factor` | double | 1.05 | Over-payout bias |

#### vision_direction_sim

Simulates a visual odometry pipeline providing motion direction.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_topic` | string | `/model/quadtailsitter/odometry` | Gazebo odometry topic |
| `drift_rate` | double | 0.001 | Random walk rate (rad/s) |
| `min_velocity` | double | 0.5 | Minimum velocity for valid direction (m/s) |

#### mock_attitude_publisher

Publishes a fixed attitude quaternion for testing without PX4.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate` | double | 50.0 | Output rate (Hz) |
| `roll` | double | 0.0 | Roll angle (rad) |
| `pitch` | double | 0.0 | Pitch angle (rad) |
| `yaw` | double | 0.0 | Yaw angle (rad) |

#### stabilized_flight_controller

PD-stabilized wrench-based flight controller. Maintains altitude, tracks racetrack waypoints, and applies one-time wrenches via Gazebo transport. See [Gazebo Wrench Lessons](docs/GAZEBO_WRENCH_LESSONS.md) for key implementation insights.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `world_name` | string | `canyon_world` | Gazebo world name |
| `model_name` | string | `quadtailsitter` | Target model name |
| `target_altitude` | double | 50.0 | Target altitude (m) |
| `target_speed` | double | 15.0 | Target forward speed (m/s) |
| `auto_enable` | bool | true | Auto-enable after valid odometry |
| `acceptance_radius` | double | 20.0 | Waypoint acceptance radius (m) |
| `kp_roll/kd_roll` | double | 8.0/2.0 | Roll PD gains |
| `kp_pitch/kd_pitch` | double | 3.0/0.8 | Pitch PD gains |
| `kp_yaw/kd_yaw` | double | 3.0/1.0 | Yaw PD gains |
| `kp_alt/kd_alt` | double | 8.0/6.0 | Altitude PD gains |

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
v_body = (spool_velocity / slack_factor) * direction_unit_vector

// 2. Rotate to NED frame using attitude quaternion
v_ned = quaternion_rotate(attitude_q, v_body)

// 3. Publish to PX4 EKF
odometry.velocity = v_ned
odometry.position = NaN  // Unknown
odometry.velocity_variance = [0.01, 0.01, 0.01]
```

---

### fiber_nav_gazebo

Gazebo Harmonic worlds and vehicle models.

**World:** `worlds/canyon_harmonic.sdf`
- SDF 1.10 format for Gazebo Harmonic
- DART physics engine with bullet collision
- 3000m x 500m ground plane
- 2500m canyon walls (150m high)
- Visual markers every 200m (color-coded)

**Model:** `models/quadtailsitter/model.sdf`
- Quad-tailsitter VTOL with fixed rotor joints (standalone wrench mode)
- Motor/aero plugins available for PX4 mode (see model comments)
- Full sensor suite (IMU, barometer, magnetometer)
- 3 cameras: forward, downward, follow (all attached to base_link)
- OdometryPublisher at 50Hz

---

### fiber_nav_mode

PX4 custom flight modes using [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib).

#### HoldMode (`hold_mode_node`)
- Registers as "FiberNav Hold" in PX4
- Captures NED position + heading on activation
- Streams `MulticopterGotoSetpointType` at 50Hz to hold position

#### CanyonMission (`canyon_mission_node`)
- `CanyonWaypointMode`: Navigates waypoint sequence with acceptance radius
- `CanyonMissionExecutor`: State machine (arm -> takeoff -> waypoints -> RTL -> disarm)
- Configurable waypoints via ROS params (`waypoints_x`, `waypoints_y`, `waypoints_heading`)

---

## PX4 Custom Flight Modes

The `fiber_nav_mode` package provides custom PX4 flight modes built with the px4-ros2-interface-lib:

| Mode | Class | Description |
|------|-------|-------------|
| FiberNav Hold | `HoldMode` | Position hold at current location |
| Canyon Mission | `CanyonMissionExecutor` | Automated waypoint navigation through canyon |

These modes register with PX4 as external modes and can be activated via QGroundControl or MAVLink commands.

---

## Configuration

### PX4 Custom Airframe

Custom airframe `4251_gz_quadtailsitter_vision`:

```bash
# GPS-denied operation
SYS_HAS_GPS=0
EKF2_GPS_CTRL=0

# External vision velocity fusion
EKF2_EV_CTRL=4
EKF2_EVV_NOISE=0.15

# Gazebo lockstep
SIM_GZ_EN=1
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

**C++ Tests** (sensor models, flight controller math, fusion algorithm, flight modes):
```bash
# Run all tests in Docker
docker compose up test

# Or interactively
docker compose run --rm simulation ./scripts/run_tests.sh
```

**Python Tests** (analysis scripts):
```bash
cd scripts
python3 -m pytest test_analysis.py -v
```

### Test Coverage

| Component | Tests | Coverage |
|-----------|-------|----------|
| Spool sensor | 5 | Noise, bias, clamping |
| Vision sensor | 5 | Direction, drift, threshold |
| Fusion algorithm | 8 | Rotation, slack, edge cases |
| Flight controller | 22 | Quaternion math, rotation, wrap, clamp, waypoints, PD control |
| Canyon waypoints | 9 | Geometry, heading, distance |
| Integration (Gazebo) | 1 | Stabilized flight stability, altitude, forward progress |
| Analysis scripts | 10 | RMSE, drift, 3-way comparison |

### Topic Verification

```bash
# Start simulation
docker compose up standalone

# In another terminal
docker exec -it fiber-nav-standalone bash

# Check topic rates
ros2 topic hz /model/quadtailsitter/odometry    # Should be ~50Hz
ros2 topic hz /sensors/fiber_spool/velocity      # Should be ~50Hz
ros2 topic hz /sensors/vision_direction          # Should be ~50Hz (when moving)
ros2 topic hz /fmu/in/vehicle_visual_odometry    # Should be ~50Hz
```

### Standalone Auto-Fly Testing

The stabilized flight controller can run without PX4. Foxglove bridge is included in the launch file by default:

```bash
# Start headless simulation with auto-fly and integrated Foxglove
docker compose up standalone

# Open Foxglove Studio -> Connect -> ws://localhost:8765
```

To disable Foxglove in the launch: `foxglove:=false`

### Integration Testing

PX4 requires a real TTY. Run in **separate terminals**:

**Terminal 1 - Gazebo + sensors:**
```bash
docker compose run --rm px4-sitl bash
source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
ros2 launch fiber_nav_bringup simulation.launch.py use_px4:=true headless:=true
```

**Terminal 2 - PX4 SITL:**
```bash
docker exec -it <container> bash
cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
rm -f dataman parameters*.bson
PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4
```

**Terminal 3 - DDS + Custom mode:**
```bash
docker exec -it <container> bash
MicroXRCEAgent udp4 -p 8888 &
sleep 3
source /root/ws/install/setup.bash
ros2 run fiber_nav_mode hold_mode_node
```

**Success:** `cs_ev_vel: true` in `/fmu/out/estimator_status_flags`

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
colcon build --symlink-install --packages-skip px4_msgs px4_ros2_cpp px4_ros2_py --packages-skip-regex 'example_.*'
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
  colcon build --symlink-install --packages-skip px4_msgs px4_ros2_cpp px4_ros2_py --packages-skip-regex 'example_.*'
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
+-- docker/
|   +-- Dockerfile              # ROS 2 Jazzy + Gazebo Harmonic + px4-ros2-interface-lib
|   +-- docker-compose.yml      # All services (simulation, standalone, px4-sitl, etc.)
|   +-- airframes/              # PX4 custom airframes (4251_gz_quadtailsitter_vision)
|   +-- entrypoint.sh           # Environment setup
+-- scripts/
|   +-- run_demo.sh             # Full demo (auto-fly)
|   +-- run_standalone.sh       # Without PX4
|   +-- run_tests.sh            # Unit tests
|   +-- record_test_flight.py   # Flight data recorder
|   +-- analyze_flight.py       # Performance analysis
|   +-- compare_three_way.py    # GPS/Fiber/IMU comparison
|   +-- generate_20km_flight.py # Synthetic benchmark data
|   +-- test_analysis.py        # Python unit tests
+-- src/
|   +-- fiber_nav_sensors/      # Sensor nodes + flight controller + tests
|   +-- fiber_nav_fusion/       # Fusion algorithm + tests
|   +-- fiber_nav_gazebo/       # World and models (canyon, quadtailsitter)
|   +-- fiber_nav_bringup/      # Launch files + integration tests
|   +-- fiber_nav_mode/         # PX4 custom flight modes (hold, canyon mission)
|   +-- fiber_nav_analysis/     # Python analysis tools
+-- foxglove/
|   +-- fiber_nav_layout.json   # Foxglove Studio layout
|   +-- README.md               # Visualization guide
+-- docs/
|   +-- PLAN.md                 # Implementation plan
|   +-- PX4_GAZEBO_INTEGRATION_PLAN.md  # EKF integration
|   +-- GAZEBO_WRENCH_LESSONS.md  # Wrench control lessons learned
+-- README.md
```

---

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/model/quadtailsitter/odometry` | nav_msgs/Odometry | Ground truth from Gazebo |
| `/sensors/fiber_spool/velocity` | std_msgs/Float64 | Scalar spool velocity (m/s) |
| `/sensors/vision_direction` | geometry_msgs/Vector3Stamped | Unit direction vector |
| `/fmu/in/vehicle_visual_odometry` | px4_msgs/VehicleOdometry | Fusion output to PX4 |
| `/fmu/out/vehicle_attitude` | px4_msgs/VehicleAttitude | PX4 attitude for transforms |
| `/camera` | sensor_msgs/Image | Forward camera feed |
| `/camera_down` | sensor_msgs/Image | Downward camera feed |
| `/follow_camera` | sensor_msgs/Image | 3rd person follow camera |

---

## License

**Proprietary Software** - Copyright (c) 2026 Pavel Guzenfeld. All Rights Reserved.

---

## Contact

Pavel Guzenfeld - pavelguzenfeld@gmail.com
