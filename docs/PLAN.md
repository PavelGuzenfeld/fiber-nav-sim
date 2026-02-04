# Fiber Navigation Simulation - Implementation Plan

## Overview

GPS-denied fixed-wing navigation using fiber optic cable odometry + monocular vision fusion.

## Current Implementation Status

### Completed

- **Phase 1: Environment & Vehicle**
  - [x] Canyon world (Gazebo Harmonic SDF 1.10) - `worlds/canyon_harmonic.sdf`
  - [x] Plane model with odometry publisher - `models/plane/model.sdf`
  - [x] ros_gz_bridge configuration - `config/ros_gz_bridge.yaml`
  - [x] PX4 parameters for GPS-denied operation - `config/px4_params.txt`

- **Phase 2: Synthetic Spool Sensor**
  - [x] Spool sim driver node - `spool_sim_driver.cpp`
  - [x] Noise model (Gaussian + bias)
  - [x] Unit tests - `test_spool_sensor.cpp`

- **Phase 3: Synthetic Vision Sensor**
  - [x] Vision direction simulator - `vision_direction_sim.cpp`
  - [x] Drift model (random walk)
  - [x] Unit tests - `test_vision_sensor.cpp`

- **Phase 4: Fusion Node**
  - [x] Core algorithm - `fiber_vision_fusion.cpp`
  - [x] Body-to-NED rotation
  - [x] PX4 visual odometry output
  - [x] Unit tests - `test_fusion.cpp`

- **Phase 5: Docker Environment**
  - [x] Dockerfile with ROS 2 Jazzy + Gazebo Harmonic
  - [x] docker-compose.yml with multiple services
  - [x] MicroXRCE-DDS agent for PX4 communication

- **Phase 6: Launch Files**
  - [x] Standalone simulation - `simulation.launch.py`
  - [x] PX4 SITL launch - `px4_sitl.launch.py`
  - [x] Full simulation - `full_simulation.launch.py`
  - [x] Mock attitude publisher for testing

### In Progress

- [x] PX4 SITL integration testing (SIH mode working, UXRCE-DDS connected)
- [x] EKF fusion verification (EKF using vision velocity, GPS disabled)

### Completed Recently

- [x] Fixed plane_controller to discover entity ID automatically
- [x] Headless simulation mode for WSL compatibility
- [x] Foxglove visualization support for real-time monitoring
- [x] Camera sensors (forward + downward) with image streaming
- [x] Analysis scripts (record, plot, metrics)
- [x] CI/CD pipeline (GitHub Actions)

### Planned

- [ ] Full PX4 EKF integration test flight
- [ ] Performance benchmarking

---

## Project Structure

```
fiber-nav-sim/
├── docker/
│   ├── Dockerfile              # ROS 2 Jazzy + Gazebo Harmonic + MicroXRCE-DDS
│   ├── docker-compose.yml      # simulation, standalone, ci, test services
│   └── entrypoint.sh           # Environment setup
├── scripts/
│   ├── run_demo.sh             # Full simulation demo
│   ├── run_standalone.sh       # Without PX4
│   ├── run_tests.sh            # Unit tests
│   └── apply_thrust.sh         # Test plane motion
├── src/
│   ├── fiber_nav_bringup/      # Launch files, configs
│   │   ├── launch/
│   │   │   ├── simulation.launch.py       # Main simulation
│   │   │   ├── px4_sitl.launch.py         # PX4 SITL
│   │   │   ├── full_simulation.launch.py  # Combined
│   │   │   └── test_sensors.launch.py     # Sensor testing
│   │   └── config/
│   │       ├── ros_gz_bridge.yaml         # Gazebo-ROS bridge
│   │       ├── px4_params.txt             # PX4 EKF config
│   │       └── sensor_params.yaml         # Sensor parameters
│   ├── fiber_nav_gazebo/       # World, models
│   │   ├── worlds/
│   │   │   ├── canyon_harmonic.sdf        # Gazebo Harmonic world
│   │   │   └── canyon.world               # Legacy (Gazebo Classic)
│   │   └── models/
│   │       └── plane/
│   │           ├── model.sdf              # SDF 1.10 plane model
│   │           └── model.config           # Model metadata
│   ├── fiber_nav_sensors/      # Synthetic sensors
│   │   ├── src/
│   │   │   ├── spool_sim_driver.cpp       # Fiber spool sensor
│   │   │   ├── vision_direction_sim.cpp   # Vision sensor
│   │   │   └── mock_attitude_publisher.cpp # Test without PX4
│   │   └── test/
│   │       ├── test_spool_sensor.cpp
│   │       └── test_vision_sensor.cpp
│   ├── fiber_nav_fusion/       # Core algorithm
│   │   ├── src/
│   │   │   └── fiber_vision_fusion.cpp
│   │   └── test/
│   │       └── test_fusion.cpp
│   └── fiber_nav_analysis/     # Python plotting
│       └── fiber_nav_analysis/
│           ├── record_trajectory.py
│           ├── plot_trajectory.py
│           └── compute_metrics.py
├── docs/
│   └── PLAN.md                 # This file
└── README.md
```

---

## Data Flow Architecture

```
Gazebo Harmonic (canyon_harmonic.sdf)
    │
    ▼
plane model (OdometryPublisher @ 50Hz)
    │
    ▼  /model/plane/odometry (gz.msgs.Odometry)
ros_gz_bridge
    │
    ▼  /model/plane/odometry (nav_msgs/Odometry)
    │
    ├──► spool_sim_driver ──► /sensors/fiber_spool/velocity
    │
    └──► vision_direction_sim ──► /sensors/vision_direction
                                        │
                                        ▼
                              fiber_vision_fusion ◄── /fmu/out/vehicle_attitude
                                        │               (PX4 or mock)
                                        ▼
                              /fmu/in/vehicle_visual_odometry
                                        │
                              ┌─────────┴─────────┐
                              ▼                   ▼
                        MicroXRCE-DDS       PX4 SITL EKF
                              │                   │
                              └───────────────────┘
```

---

## Docker Services

| Service | Command | Description |
|---------|---------|-------------|
| `simulation` | `docker compose up simulation` | Full Gazebo GUI simulation |
| `standalone` | `docker compose up standalone` | Without PX4, uses mock attitude |
| `test` | `docker compose up test` | Build and run unit tests |
| `ci` | `docker compose up ci` | Headless CI testing |
| `analysis` | `docker compose up analysis` | Jupyter notebook |

---

## Quick Start

```bash
# Build Docker image
cd docker
docker compose build simulation

# Run standalone mode (no PX4)
docker compose up standalone

# Or interactive shell
docker compose run --rm simulation bash

# Inside container:
./scripts/run_standalone.sh
```

---

## Sensor Node Details

### spool_sim_driver

Subscribes to: `/model/plane/odometry`
Publishes to: `/sensors/fiber_spool/velocity`

Noise model:
```
S_measured = (|v_true| + N(0, σ=0.1)) × slack_factor(1.05)
```

### vision_direction_sim

Subscribes to: `/model/plane/odometry`
Publishes to: `/sensors/vision_direction`

Drift model:
```
Δdrift = N(0, drift_rate × √dt)
û_drifted = Rz(drift_yaw) × Ry(drift_pitch) × û_true
```

### fiber_vision_fusion

Subscribes to:
- `/sensors/fiber_spool/velocity`
- `/sensors/vision_direction`
- `/fmu/out/vehicle_attitude`

Publishes to: `/fmu/in/vehicle_visual_odometry`

Algorithm:
```
v_body = (spool_velocity / slack_factor) × direction_unit_vector
v_ned = quaternion_rotate(attitude_q, v_body)
```

---

## PX4 Configuration

Key parameters in `config/px4_params.txt`:

```
# Disable GPS
EKF2_GPS_CTRL=0

# Enable external vision velocity
EKF2_EV_CTRL=4    # Velocity only

# Vision noise
EKF2_EVV_NOISE=0.15
```

---

## Testing

### Unit Tests

```bash
# In Docker
./scripts/run_tests.sh

# Or manually
colcon test --packages-skip px4_msgs
colcon test-result --verbose
```

### Verify Topics

```bash
# In Docker
ros2 topic hz /model/plane/odometry          # ~50Hz
ros2 topic hz /sensors/fiber_spool/velocity  # ~50Hz
ros2 topic hz /sensors/vision_direction      # ~50Hz (when moving)
ros2 topic hz /fmu/in/vehicle_visual_odometry # ~50Hz
```

---

## Success Criteria

| Metric | Target | Stretch | Measured |
|--------|--------|---------|----------|
| Drift per 1000m | <10m | <5m | 28.3m* |
| Velocity RMSE | <0.5 m/s | <0.2 m/s | **0.127 m/s** ✓ |
| CPU usage (fusion node) | <5% | <2% | TBD |
| Latency (sensor→EKF) | <50ms | <20ms | TBD |

*Note: Drift measured over 294m at ~10 m/s using dead-reckoning integration. Higher drift is expected without loop closure or external position corrections.

---

## Benchmark Results (2024-02-04)

```
Flight Statistics:
  Duration: 30s, Distance: 294.6m, Avg Speed: 10.1 m/s

Velocity RMSE (Fusion vs Ground Truth): 0.127 m/s  [PASS]
Position Drift: 8.34m over 294.6m → 28.3m per 1000m
```

The fusion algorithm achieves excellent instantaneous velocity accuracy. Position drift accumulates due to:
- Synthetic sensor noise (spool: σ=0.1 m/s, vision: random walk drift)
- Slack factor correction uncertainty
- No loop closure or external position aiding

---

## Known Limitations

**PX4 SIH Integration**: Current setup uses PX4 SIH (Simulator-in-Hardware) which runs its own internal physics simulation separate from Gazebo. The EKF will reject visual odometry velocities that differ significantly from SIH's internal state. For full EKF integration testing, PX4 needs to be built with native Gazebo Harmonic support.

---

## Next Steps

1. ~~**Integration Testing**: Verify PX4 EKF accepts fused velocity~~ (Limited by SIH architecture)
2. ~~**Flight Testing**: Apply forces to plane, verify sensor pipeline~~ ✓
3. ~~**Analysis Tools**: Complete recording and plotting scripts~~ ✓
4. **CI Pipeline**: GitHub Actions for automated testing
5. **PX4 Gazebo Integration**: Build PX4 with `gz_x500` for native Gazebo support
