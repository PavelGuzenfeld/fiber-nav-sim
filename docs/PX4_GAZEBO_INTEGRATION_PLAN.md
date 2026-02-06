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
- GPS-denied with vision velocity fusion (`EKF2_EV_CTRL=4`)
- Gazebo physics lockstep (`SIM_GZ_EN=1`)
- PX4-Autopilot built with `px4_sitl` target

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

### Phase 5: Integration Testing - PENDING
**Goal:** Verify full end-to-end with PX4 SITL

PX4 requires a real TTY and must be started manually in separate terminals:

**Terminal 1 - Gazebo + sensors:**
```bash
docker compose run --rm px4-sitl bash
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

**Success criteria:**
- `cs_ev_vel: true` in `/fmu/out/estimator_status_flags`
- Quadtailsitter hovers stably in hold mode
- Motors respond (`/fmu/out/actuator_motors` non-zero)

---

## File Summary

| File | Description |
|------|-------------|
| `models/quadtailsitter/model.sdf` | VTOL model with full sensor suite |
| `docker/airframes/4251_gz_quadtailsitter_vision` | PX4 custom airframe |
| `src/fiber_nav_mode/` | Custom flight modes (hold, canyon mission) |
| `src/fiber_nav_bringup/launch/simulation.launch.py` | Main launch (always quadtailsitter) |
| `src/fiber_nav_bringup/launch/custom_mode.launch.py` | Custom mode launch |

---

## References

- [PX4 Gazebo Harmonic Docs](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Gazebo Sensor Plugins](https://gazebosim.org/docs/harmonic/sensors)
- [EKF2 External Vision](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
