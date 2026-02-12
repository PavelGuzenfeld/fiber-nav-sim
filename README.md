# Fiber Navigation Simulation

**GPS-denied VTOL navigation using fiber optic cable odometry and monocular vision fusion.**

A fully Dockerized ROS 2 Jazzy / Gazebo Harmonic simulation environment for testing a novel navigation algorithm where a tethered quad-tailsitter VTOL drone navigates through GPS-denied environments (tunnels, canyons) using only:
- **Cable Spool Sensor**: Measures scalar velocity magnitude from fiber payout rate
- **Monocular Camera**: Provides direction of motion (unit vector) without scale

---

## Benchmark Results

### PX4 SITL — Canyon Mission (3.3km, 10 minutes)

Full PX4 EKF2 integration with GPS + fiber+vision velocity fusion, tested on a 4-waypoint canyon back-and-forth mission at 15m altitude. GT velocity properly converted from body frame (FLU) to NED via quaternion rotation.

| Metric | Value | Target | Status |
|--------|-------|--------|--------|
| EKF Position (mean) | **0.85 m** | < 5 m | PASS |
| EKF Position (max) | **1.94 m** | — | — |
| EKF Position (final) | **0.22 m** | — | — |
| EKF Drift | **0.06 m/1000m** | < 10 m/1000m | PASS |
| Speed RMSE (EKF vs GT) | **0.187 m/s** | < 0.5 m/s | PASS |
| Speed RMSE (Fusion vs GT) | **0.133 m/s** | < 0.5 m/s | PASS |
| 3D Velocity RMSE (EKF vs GT) | **0.210 m/s** | < 0.5 m/s | PASS |

See `scripts/record_test_flight.py` and `scripts/analyze_flight.py` for recording and analysis.

### Standalone Fusion — Short Flight (15s, ~300m)
| Method | Velocity RMSE | Position Error | Drift Rate |
|--------|---------------|----------------|------------|
| GPS | 0.26 m/s | 2.3m | ~0 (bounded) |
| **Fiber+Vision** | **0.96 m/s** | 14.5m | 48 m/km |
| IMU-only | 0.20 m/s | 2.7m | quadratic |

### Standalone Fusion — Long Distance (20km, ~17 minutes)
| Method | Velocity RMSE | Position Error | vs IMU |
|--------|---------------|----------------|--------|
| GPS | 0.26 m/s | 1.6m | reference |
| **Fiber+Vision** | 0.96 m/s | 952m (4.8%) | **12x better** |
| IMU-only | 13.82 m/s | 11,979m (60%) | - |

**Key findings:**
- PX4 EKF2 with GPS+fusion: sub-meter position accuracy over 3.7km, negligible drift
- Fiber+Vision achieves **12x less position error** than IMU-only over 20km
- Position drift is linear (Fiber+Vision) vs quadratic (IMU)
- Viable for GPS-denied long-distance navigation

See `scripts/compare_three_way.py` and `scripts/generate_20km_flight.py` for standalone analysis.

---

## Project Status

| Component | Status | Notes |
|-----------|--------|-------|
| Gazebo simulation | Done | Canyon + terrain worlds, quadtailsitter VTOL model |
| Terrain mapping | Done | SRTM DEM + satellite imagery → heightmap + GIS service |
| Sensor simulation | Done | Spool + vision + IMU/baro/mag + terrain-aware distance sensor |
| Fusion algorithm | Done | Body-to-NED transform, adaptive variance, flight mode awareness, health scaling, cross-validation, heading check, adaptive ZUPT, slack calibration |
| Stabilized flight | Done | PD wrench controller, 50m altitude, racetrack waypoints |
| PX4 integration | Done | Custom airframe 4251, sensor bridges, 7-phase orchestrator |
| PX4 offboard flight | Done | MC takeoff, VTOL FW mission, GPS-denied navigation |
| VTOL navigation mode | Done | C++ 7-state machine (MC_CLIMB → FW_NAVIGATE → MC_APPROACH → DONE) |
| Terrain-follow lookahead | Done | GIS look-ahead + feed-forward altitude controller |
| Sensor fusion enhancements | Done | Health scaling, staleness, cross-validation, heading check |
| GPS-denied improvements | Done | Online slack calibration, adaptive ZUPT, heading cross-check |
| Custom flight modes | Done | HoldMode + CanyonMission + VtolNavigationMode |
| Foxglove visualization | Done | Integrated in launch file (default enabled, port 8765) |
| ZUPT + Position Clamping | Done | Wire ZUPT, drag bow 1D position, SpoolStatus message |
| Unit tests | Done | 209 tests (163 C++ + 31 terrain pipeline + 15 analysis) |
| Integration tests | Done | Stabilized flight + PX4 SITL + terrain E2E |
| PX4 SITL perf test | Done | 3.7km canyon mission, sub-meter EKF accuracy |
| Benchmarking | Done | 20km 3-way comparison |
| O3DE migration | Phase 0 | GPU/Vulkan works (DZN), Atom renderer crashes on WSL2 |

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
  /velocity + /status    _direction              | /world/.../wrench
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
    |  * velocity = v_ned (or ZUPT 0)   |       | foxglove_bridge   |
    |  * position = drag bow estimate   |       | ws://localhost:   |
    |  * covariance = sensor noise      |       |         8765      |
    +-----------------------------------+       +-------------------+
```

### PX4 Mode (offboard flight control)

PX4 communicates on **two separate buses**: gz transport for Gazebo physics (motors, sensors)
and ROS 2 via MicroXRCEAgent for all other nodes (fusion, offboard scripts, flight modes).

```
                    +---------------------------------------+
                    |        PX4 SITL (airframe 4251)       |
                    |  * GPS home position (SYS_HAS_GPS=1)  |
                    |  * Vision vel+pos (EKF2_EV_CTRL=5)    |
                    |  * Range finder (EKF2_RNG_CTRL=1)     |
                    |  * Airspeed disabled (FW_ARSP_MODE=1)  |
                    +--------+------------------+-----------+
                             |                  ^
         gz transport        |                  |  MicroXRCEAgent
     (motors, IMU, baro,     |                  |  (UDP 8888)
      mag, navsat, lockstep) |                  |
                             |                  |
+----------------------------+-+                |
| Gazebo Harmonic              |                |
| (quadtailsitter model)      |                |
| * Revolute joints            |                |
| * MulticopterMotorModel x4   |                |
| * IMU, Baro, Mag, NavSat    |                |
| * Cameras: fwd, down, follow |                |
+-----------+------------------+                |
            |                                   |
            | ros_gz_bridge                     |
            | (odometry, cameras)               |
            v                                   v
+=================================================================+
|                     ROS 2 (Jazzy) Topic Bus                     |
|=================================================================|
| /model/.../odometry       /fmu/out/vehicle_attitude             |
| /camera, /camera_down     /fmu/out/vehicle_status               |
| /follow_camera            /fmu/out/vehicle_local_position       |
|                           /fmu/in/vehicle_visual_odometry       |
|                           /fmu/in/trajectory_setpoint           |
|                           /fmu/in/offboard_control_mode         |
+=+=========+=========+=========+=========+=========+============+
  |         |         |         |         |         |
  v         v         v         v         v         v
+------+ +------+ +--------+ +--------+ +--------+ +---------+
|spool | |vision| |fiber   | |sim     | |offboard| |foxglove |
|_sim  | |_dir  | |_vision | |_dist   | |_mission| |_bridge  |
|driver| |_sim  | |_fusion | |_sensor | |.py     | |ws://    |
|      | |      | |        | |.py     | |        | |...:8765 |
+--+---+ +--+---+ +--+--+-+ +----+---+ +--------+ +---------+
   |        |        |  ^        |
   |        |        |  |        |
   |        |        v  |        v
   |        |  /fmu/in/ |   /fmu/in/
   |        |  vehicle_ |   distance_
   |        |  visual_  |   sensor
   v        v  odometry |
/sensors/  /sensors/     |
fiber_     vision_       |
spool/     _direction    |
velocity        |        |
+ status        |        |
   |            |        |
   +-----+------+       |
         |               |
   fiber_vision_fusion --+
   reads /fmu/out/vehicle_attitude
   from PX4 for body-to-NED rotation
   ZUPT: zero velocity when stopped
   Drag bow: 1D position from spool length
```

> **Note:** The quadtailsitter model has two configurations. In standalone mode (wrench-based),
> motor plugins are disabled and rotor joints are fixed — flight is controlled via
> the `stabilized_flight_controller` applying one-time wrenches. In PX4 mode, motors are
> enabled with revolute joints, MulticopterMotorModel x4, and AdvancedLiftDrag aerodynamics
> for full VTOL MC/FW flight. See [Gazebo Wrench Lessons](docs/GAZEBO_WRENCH_LESSONS.md) for details.
>
> **Critical:** When sending `TrajectorySetpoint` in offboard mode, ALL unused fields must be
> set to `NaN`. The ROS2 message defaults to `[0,0,0]`, not NaN. PX4 treats non-NaN values
> as valid constraints, which causes hard altitude ceilings and other unexpected behavior.

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
- **PX4 mode**: Revolute rotor joints, 4x MulticopterMotorModel. PX4 controls motors directly (flies as quadcopter; AdvancedLiftDrag not yet enabled)
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
| `moving_threshold` | double | 0.05 | Threshold for is_moving in SpoolStatus |

Publishes:
- `/sensors/fiber_spool/velocity` (Float32) — backward-compatible scalar velocity
- `/sensors/fiber_spool/status` (SpoolStatus) — velocity + total_length + is_moving

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
| `zupt_threshold` | double | 0.05 | Speed below this triggers ZUPT (m/s) |
| `zupt_velocity_variance` | double | 0.001 | Velocity variance during ZUPT |
| `enable_position_clamping` | bool | true | Enable drag bow position estimate |
| `k_drag` | double | 0.0005 | Drag bow coefficient |
| `tunnel_heading_deg` | double | 90.0 | NED heading of tunnel axis (deg) |
| `position_variance_longitudinal` | double | 1.0 | Position variance along tunnel |
| `position_variance_lateral` | double | 100.0 | Position variance perpendicular |

**Algorithm:**
```cpp
// 1. ZUPT: If spool velocity < threshold, hard-reset to zero
if (spool_velocity < zupt_threshold) {
    odometry.velocity = [0, 0, 0]  // Zero-velocity update
    odometry.velocity_variance = [0.001, 0.001, 0.001]
} else {
    // 2. Reconstruct body-frame velocity
    v_body = (spool_velocity / slack_factor) * direction_unit_vector
    // 3. Rotate to NED frame using attitude quaternion
    v_ned = quaternion_rotate(attitude_q, v_body)
    odometry.velocity = v_ned
    odometry.velocity_variance = [0.01, 0.01, 0.01]
}

// 4. Drag bow position estimate (1D along tunnel axis)
x_est = total_spool_length * (1 - k_drag * v^2)
odometry.position = rotate_to_NED(x_est, tunnel_heading)
odometry.position_variance = anisotropic(longitudinal, lateral, heading)
```

---

### fiber_nav_gazebo

Gazebo Harmonic worlds and vehicle models.

**Worlds:**

| World | File | Description |
|-------|------|-------------|
| `terrain_world` | `worlds/terrain_world.sdf` | Real terrain from SRTM DEM (Negev, 6km x 6km, satellite texture) |
| `canyon_harmonic` | `worlds/canyon_harmonic.sdf` | Procedural canyon (3km x 0.5km, visual markers) |

The terrain world is generated by `scripts/generate_terrain.py` from a template (`terrain_world.sdf.template`). It includes:
- 513x513 heightmap visual with satellite texture (Bing Maps)
- Flat ground plane at terrain center height for DART collision (heightmap collision not supported)
- Spherical coordinates centered on terrain GPS origin
- Atmosphere tuned to terrain MSL elevation

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
# GPS for home position + global origin
SYS_HAS_GPS=1
EKF2_GPS_CTRL=7        # position + velocity + altitude

# External vision velocity + position fusion
EKF2_EV_CTRL=5
EKF2_EVV_NOISE=0.15
EKF2_EVP_NOISE=1.0

# Range finder for terrain estimation
EKF2_RNG_CTRL=1
EKF2_RNG_A_HMAX=50

# Land detector tuning (prevents ground_contact deadlock)
LNDMC_ALT_MAX=1.0

# SITL workarounds
CBRK_SUPPLY_CHK=894281  # No power supply sensor
CBRK_USB_CHK=197848     # Container environment
CBRK_AIRSPD_CHK=162128  # No airspeed sensor in SITL
FW_ARSP_MODE=1           # Disable airspeed sensing

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
| Spool sensor | 10 | Noise, bias, clamping, total length, is_moving |
| Vision sensor | 5 | Direction, drift, threshold |
| Fusion algorithm | 69 | Rotation, slack, ZUPT, drag bow, health scaling, staleness, cross-val, heading check, adaptive ZUPT, slack calibration |
| Flight controller | 22 | Quaternion math, rotation, wrap, clamp, waypoints, PD control |
| Canyon waypoints | 9 | Geometry, heading, distance |
| VTOL navigation | 24 | State machine, course geometry, FW setpoints, config, transitions |
| Terrain altitude ctrl | 24 | P-controller, filter, look-ahead, feed-forward, AMSL, clamping |
| Terrain pipeline (Python) | 31 | Heightmap gen, texture, coordinate transforms, GIS queries |
| Analysis scripts (Python) | 15 | RMSE, drift, 3-way comparison, fusion position error |

Total: 209 (163 C++ + 46 Python)

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

PX4 requires a real TTY for interactive use. The offboard scripts can start PX4 in the background.

**Terminal 1 - Start simulation container:**
```bash
docker compose -f docker/docker-compose.yml run -d --name fiber-nav-px4-sitl --rm \
  px4-sitl bash -c "source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash && \
  ros2 launch fiber_nav_bringup simulation.launch.py use_px4:=true headless:=true foxglove:=true"
```

**Terminal 2 - Start supporting services:**
```bash
# Copy latest airframe (baked into image at build time)
docker exec fiber-nav-px4-sitl cp /root/ws/src/fiber-nav-sim/docker/airframes/4251_gz_quadtailsitter_vision \
  /root/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/

# Start DDS agent
docker exec -d fiber-nav-px4-sitl bash -c "MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1"

# Start distance sensor simulator
docker exec -d fiber-nav-px4-sitl bash -c "source /opt/ros/jazzy/setup.bash && \
  source /root/ws/install/setup.bash && \
  python3 /root/ws/src/fiber-nav-sim/scripts/sim_distance_sensor.py > /dev/null 2>&1"

# Start PX4 (output to /dev/null — NEVER redirect to a file, it creates multi-GB logs)
docker exec -d fiber-nav-px4-sitl bash -c "cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs && \
  rm -f dataman parameters*.bson && source /opt/ros/jazzy/setup.bash && \
  PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4 > /dev/null 2>&1"
```

**Terminal 3 - Run offboard takeoff:**
```bash
docker exec -it fiber-nav-px4-sitl bash -c "source /opt/ros/jazzy/setup.bash && \
  source /root/ws/install/setup.bash && \
  python3 /root/ws/src/fiber-nav-sim/scripts/offboard_takeoff.py 10.0"
```

**Foxglove:** Open https://app.foxglove.dev → Connect → `ws://localhost:8765`

**Success criteria:**
- `pre_flight_checks_pass: true`
- Vehicle arms and climbs to target altitude at ~2 m/s
- Holds position for 30s, then lands

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
|   +-- Dockerfile.o3de         # O3DE + Mesa DZN Vulkan (Phase 0)
|   +-- docker-compose.yml      # All services (simulation, standalone, px4-sitl, etc.)
|   +-- airframes/              # PX4 custom airframes (4251_gz_quadtailsitter_vision)
|   +-- entrypoint.sh           # Environment setup
|   +-- px4-sitl-entrypoint.sh  # 7-phase PX4 SITL orchestrator
|   +-- gpu_test.sh             # GPU/Vulkan stack verification
|   +-- test_headless.sh        # O3DE headless GO/NO-GO test
+-- scripts/
|   +-- generate_terrain.py     # Terrain pipeline (DEM -> heightmap -> texture -> SDF)
|   +-- terrain_gis_node.py     # ROS 2 terrain height query service
|   +-- sim_distance_sensor.py  # Terrain-aware AGL distance sensor
|   +-- offboard_takeoff.py     # PX4 offboard takeoff (arm, climb, hold, land)
|   +-- offboard_mission.py     # PX4 offboard mission (MC/VTOL/GPS-denied)
|   +-- offboard_transition_test.py  # MC<->FW transition test
|   +-- record_test_flight.py   # Flight data recorder
|   +-- analyze_flight.py       # Performance analysis
|   +-- compare_three_way.py    # GPS/Fiber/IMU comparison
|   +-- test_terrain.py         # Terrain pipeline unit tests (31 tests)
|   +-- test_analysis.py        # Analysis script unit tests
+-- src/
|   +-- fiber_nav_sensors/      # Sensor nodes + flight controller + tests
|   +-- fiber_nav_fusion/       # Fusion algorithm + tests
|   +-- fiber_nav_gazebo/       # Worlds, models, terrain data
|   |   +-- worlds/             # canyon_harmonic.sdf, terrain_world.sdf
|   |   +-- models/             # quadtailsitter (model.sdf, model_px4.sdf)
|   |   +-- terrain/            # heightmap, texture, terrain_data.json
|   +-- fiber_nav_bringup/      # Launch files (dispatcher + backends)
|   +-- fiber_nav_mode/         # PX4 custom flight modes (hold, canyon, VTOL nav)
|   +-- fiber_nav_analysis/     # Python analysis tools
|   +-- fiber_nav_o3de/         # O3DE gems (px4_bridge, vtol_dynamics) — scaffolds
+-- foxglove/
|   +-- fiber_nav_layout.json   # Foxglove Studio layout
+-- docs/
|   +-- PLAN.md                 # Implementation plan
|   +-- PX4_GAZEBO_INTEGRATION_PLAN.md  # EKF integration (10 phases)
|   +-- O3DE_MIGRATION_PLAN.md  # O3DE migration plan (Phase 0 partial)
|   +-- VTOL_NAVIGATION_MODE.md # C++ VTOL nav mode docs
|   +-- GAZEBO_WRENCH_LESSONS.md  # Wrench control lessons
|   +-- ROADMAP_ZUPT_POSITION.md  # ZUPT + 1D position clamping
+-- README.md
```

---

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/model/quadtailsitter/odometry` | nav_msgs/Odometry | Ground truth from Gazebo |
| `/sensors/fiber_spool/velocity` | std_msgs/Float32 | Scalar spool velocity (m/s) |
| `/sensors/fiber_spool/status` | fiber_nav_sensors/SpoolStatus | Velocity + total length + is_moving |
| `/sensors/vision_direction` | geometry_msgs/Vector3Stamped | Unit direction vector |
| `/fmu/in/vehicle_visual_odometry` | px4_msgs/VehicleOdometry | Fusion output to PX4 |
| `/fmu/in/distance_sensor` | px4_msgs/DistanceSensor | Terrain-aware AGL |
| `/fmu/out/vehicle_attitude` | px4_msgs/VehicleAttitude | PX4 attitude for transforms |
| `/terrain/query` | geometry_msgs/Point | Query terrain height at (x, y) |
| `/terrain/height` | std_msgs/Float64 | Terrain height response |
| `/terrain/info` | std_msgs/String | Terrain metadata JSON (latched) |
| `/camera` | sensor_msgs/Image | Forward camera feed |
| `/camera_down` | sensor_msgs/Image | Downward camera feed |
| `/follow_camera` | sensor_msgs/Image | 3rd person follow camera |

---

## License

**Proprietary Software** - Copyright (c) 2026 Pavel Guzenfeld. All Rights Reserved.

---

## Contact

Pavel Guzenfeld - pavelguzenfeld@gmail.com
