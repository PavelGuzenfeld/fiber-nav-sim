# PX4-Gazebo Native Integration Plan

## Problem Statement

GPS-denied navigation requires PX4's EKF to accept external vision velocity from the fiber+vision fusion node. The EKF will reject visual odometry if sensor data is inconsistent with its internal state.

## Solution: Native Gazebo Integration

PX4 reads IMU/baro/mag directly from Gazebo via native GZBridge, while the fusion node provides external vision velocity. All sensor data comes from the same physics simulation, ensuring consistency.

```
Target Architecture:
+-------------------------------------------+
|           Gazebo Physics                  |
|   (IMU, Baro, Mag + quadtailsitter)      |
+------------------+------------------------+
                   |
    +--------------+---------------+
    v              v               v
+--------+    +--------+    +-------------+
|  IMU   |    |  Baro  |    | Fusion Node |
|  Mag   |    |        |    | (vis odom)  |
+---+----+    +---+----+    +------+------+
    |             |                |
    v             v                v
+-------------------------------------------+
|      PX4 EKF (consistent state)          |
|   IMU matches visual odometry: ACCEPT    |
+-------------------------------------------+
```

---

## Implementation Status

### Phase 1: Model Preparation - COMPLETE
- Quadtailsitter model with IMU (250Hz), barometer (50Hz), magnetometer (50Hz)
- Forward, downward, and follow cameras attached to base_link
- AdvancedLiftDrag aerodynamics + MulticopterMotorModel x4
- OdometryPublisher at 50Hz

### Phase 2: PX4 Build Configuration - COMPLETE
- Custom airframe `4251_gz_quadtailsitter_vision`
- Vision velocity fusion (`EKF2_EV_CTRL=4`) as primary nav source
- GPS provides global origin/home position (`EKF2_GPS_CTRL=1`); in real GPS-denied flight, GPS loss triggers fallback to vision-only
- Gazebo physics lockstep (`SIM_GZ_EN=1`)
- PX4-Autopilot built with `GZ_DISTRO=harmonic make px4_sitl_default` (NOT `gz_x500` which hangs)
- gz_bridge compiled via ROS Jazzy vendor packages (gz-transport13)

### Phase 3: Bridge Configuration - COMPLETE
- ros_gz_bridge maps all sensor topics from Gazebo to ROS 2
- Topic paths: `/world/canyon_world/model/quadtailsitter/link/base_link/sensor/...`
- Camera topics also bridged for visualization
- Odometry bridged at `/model/quadtailsitter/odometry`

### Phase 4: Custom Flight Modes - COMPLETE
- px4-ros2-interface-lib integrated
- HoldMode: Position hold via MulticopterGotoSetpointType
- CanyonMission: Waypoint navigation (arm -> takeoff -> waypoints -> RTL -> disarm)
- Registered as external PX4 modes

### Phase 5: Integration Testing - COMPLETE
**Goal:** Verify full end-to-end with PX4 SITL

PX4 requires a real TTY and must be started manually in separate terminals.
GPU support required (nvidia-container-toolkit) to prevent Gazebo from pegging CPU.

**Terminal 1 - Gazebo + sensors + foxglove:**
```bash
cd ~/workspace/fiber-nav-sim
docker compose -f docker/docker-compose.yml run --rm -p 8765:8765 px4-sitl bash
# Inside container:
source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
cd /root/ws && colcon build --symlink-install --packages-skip px4_msgs px4_ros2_cpp px4_ros2_py --packages-skip-regex 'example_.*' --cmake-args -DCMAKE_CXX_STANDARD=23
ros2 launch fiber_nav_bringup simulation.launch.py use_px4:=true headless:=true foxglove:=true
```

**Terminal 2 - DDS Agent:**
```bash
docker exec -it fiber-nav-px4-sitl bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3 - PX4 SITL:**
```bash
docker exec -it fiber-nav-px4-sitl bash
source /opt/ros/jazzy/setup.bash
cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
rm -f dataman parameters*.bson
PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4
```

**Foxglove:** Open https://studio.foxglove.dev -> Connect -> `ws://localhost:8765`

**Verified success criteria (2026-02-07):**
- `cs_ev_vel: true` — EKF fusing vision velocity
- `cs_gnss_pos: true` — GPS providing global reference
- `home_position_invalid: false` — Home position set
- `global_position_invalid: false` — Global position valid
- `local_position_invalid: false` — Local position valid
- gz_bridge connected: `world: canyon_world, model: quadtailsitter`
- DDS bridge: all PX4 topics published on ROS2

### Phase 6: Arm & Takeoff Verification - PENDING
**Goal:** Verify vehicle can arm, take off, and hover stably
- Motors respond (`/fmu/out/actuator_motors` non-zero)
- Quadtailsitter hovers stably in hold mode
- Custom flight modes (HoldMode, CanyonMission) operational

---

## File Summary

| File | Description |
|------|-------------|
| `models/quadtailsitter/model.sdf` | VTOL model (fixed joints, no motor plugins) |
| `models/quadtailsitter/model_px4.sdf` | PX4 variant (revolute joints + MulticopterMotorModel plugins) |
| `worlds/canyon_harmonic.sdf` | World with NavSat, IMU, Baro, Mag system plugins |
| `docker/airframes/4251_gz_quadtailsitter_vision` | PX4 custom airframe |
| `docker/Dockerfile` | Builds PX4 with `GZ_DISTRO=harmonic make px4_sitl_default` |
| `docker/docker-compose.yml` | Services with GPU support (nvidia-container-toolkit) |
| `src/fiber_nav_mode/` | Custom flight modes (hold, canyon mission) |
| `src/fiber_nav_bringup/launch/simulation.launch.py` | Main launch (selects model_px4.sdf when use_px4:=true) |
| `src/fiber_nav_bringup/launch/custom_mode.launch.py` | Custom mode launch |

---

## References

- [PX4 Gazebo Harmonic Docs](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Gazebo Sensor Plugins](https://gazebosim.org/docs/harmonic/sensors)
- [EKF2 External Vision](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
