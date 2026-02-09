# Fiber Navigation Simulation - Implementation Plan

## Overview

GPS-denied VTOL navigation using fiber optic cable odometry + monocular vision fusion.

## Current Implementation Status

### Completed

- **Phase 1: Environment & Vehicle**
  - [x] Canyon world (Gazebo Harmonic SDF 1.10) - `worlds/canyon_harmonic.sdf`
  - [x] Quadtailsitter VTOL model with AdvancedLiftDrag - `models/quadtailsitter/model.sdf`
  - [x] ros_gz_bridge configuration
  - [x] PX4 parameters for GPS-denied operation - custom airframe 4251

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
  - [x] docker-compose.yml with multiple services (simulation, standalone, px4-sitl, ci, test, foxglove, analysis)
  - [x] MicroXRCE-DDS agent for PX4 communication
  - [x] px4-ros2-interface-lib for custom flight modes

- **Phase 6: Launch Files**
  - [x] Standalone simulation - `simulation.launch.py` (always uses quadtailsitter)
  - [x] PX4 SITL launch - `px4_sitl.launch.py`
  - [x] Full simulation - `full_simulation.launch.py`
  - [x] Custom mode launch - `custom_mode.launch.py`
  - [x] Mock attitude publisher for testing

- **Phase 7: PX4 Custom Flight Modes**
  - [x] HoldMode (FiberNav Hold) - position hold via MulticopterGotoSetpointType
  - [x] CanyonMission (ModeExecutorBase) - waypoint navigation through canyon
  - [x] Unit tests - `test_canyon_waypoints.cpp`

- **Phase 8: Visualization**
  - [x] Foxglove Studio layout with 3 cameras + plots
  - [x] Follow camera attached to vehicle base_link
  - [x] Forward + downward cameras for navigation

### Completed Recently

- [x] Switched from plane model to quadtailsitter VTOL (proper aerodynamics + motors)
- [x] Follow camera attached to model base_link (was static world camera)
- [x] PX4 custom flight modes via px4-ros2-interface-lib
- [x] Updated all documentation and architecture diagrams
- [x] 72 tests passing (spool, vision, fusion, flight controller, waypoints, integration, analysis)
- [x] ZUPT (Zero-Velocity Update) — hard-reset velocity to zero when spool reports no motion
- [x] 1D Position Clamping via drag bow model — position estimate from accumulated spool length
- [x] SpoolStatus message — velocity + total_length + is_moving
- [x] EKF2_EV_CTRL=5 — position + velocity fusion active (cs_ev_pos + cs_ev_vel)
- [x] VTOL Fixed-Wing Transition — AdvancedLiftDrag plugin, tuned transition params
- [x] FW Canyon Mission — `offboard_mission.py --vtol` with 4 FW waypoints at 18-23 m/s
- [x] MC→FW→MC transition test — `offboard_transition_test.py`
- [x] Camera orientations verified for both hover and FW flight modes

---

## Data Flow Architecture

```
Gazebo Harmonic (canyon_harmonic.sdf)
    |
    v
quadtailsitter model (OdometryPublisher @ 50Hz)
    |
    v  /model/quadtailsitter/odometry (gz.msgs.Odometry)
ros_gz_bridge
    |
    v  /model/quadtailsitter/odometry (nav_msgs/Odometry)
    |
    +-->  spool_sim_driver --> /sensors/fiber_spool/velocity + /status
    |
    +-->  vision_direction_sim --> /sensors/vision_direction
                                        |
                                        v
                              fiber_vision_fusion <-- /fmu/out/vehicle_attitude
                                        |               (PX4 or mock)
                                        v
                              /fmu/in/vehicle_visual_odometry
                                        |
                              +---------+---------+
                              v                   v
                        MicroXRCE-DDS       PX4 SITL EKF
                              |                   |
                              +-------------------+
```

---

## Docker Services

| Service | Command | Description |
|---------|---------|-------------|
| `simulation` | `docker compose up simulation` | Full Gazebo GUI simulation |
| `standalone` | `docker compose up standalone` | Headless, mock attitude, auto-fly |
| `px4-sitl` | `docker compose up px4-sitl` | Headless for PX4 SITL mode |
| `test` | `docker compose up test` | Build and run unit tests |
| `ci` | `docker compose up ci` | Headless CI testing |
| `foxglove` | `docker compose up foxglove` | Foxglove visualization bridge |
| `analysis` | `docker compose up analysis` | Jupyter notebook |

---

## Sensor Node Details

### spool_sim_driver

Subscribes to: `/model/quadtailsitter/odometry`
Publishes to: `/sensors/fiber_spool/velocity`

### vision_direction_sim

Subscribes to: `/model/quadtailsitter/odometry`
Publishes to: `/sensors/vision_direction`

### fiber_vision_fusion

Subscribes to:
- `/sensors/fiber_spool/status` (SpoolStatus: velocity + total_length + is_moving)
- `/sensors/vision_direction`
- `/fmu/out/vehicle_attitude`

Publishes to: `/fmu/in/vehicle_visual_odometry`

---

## PX4 Configuration

Custom airframe `4251_gz_quadtailsitter_vision`:

```
SYS_HAS_GPS=1        # GPS for home position
EKF2_GPS_CTRL=7      # Position + velocity + altitude
EKF2_EV_CTRL=5       # External vision velocity + position
EKF2_EVV_NOISE=0.15  # Vision velocity noise
EKF2_EVP_NOISE=1.0   # Vision position noise
SIM_GZ_EN=1          # Gazebo lockstep
```

---

## Success Criteria

| Metric | Target | Stretch | Measured |
|--------|--------|---------|----------|
| Drift per 1000m | <10m | <5m | 28.3m* |
| Velocity RMSE | <0.5 m/s | <0.2 m/s | **0.127 m/s** |
| CPU usage (fusion node) | <5% | <2% | TBD |
| Latency (sensor->EKF) | <50ms | <20ms | TBD |

*Note: Drift measured over 294m using dead-reckoning integration.

---

## References

- [PX4 Gazebo Harmonic Docs](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Gazebo Sensor Plugins](https://gazebosim.org/docs/harmonic/sensors)
- [EKF2 External Vision](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
