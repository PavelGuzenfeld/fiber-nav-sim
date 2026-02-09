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
- AdvancedLiftDrag aerodynamics (PX4 reference coefficients) + MulticopterMotorModel x4
- OdometryPublisher at 50Hz
- Camera orientations: forward looks along body +Z (FW flight forward), down looks along body +X (FW flight ground)

### Phase 2: PX4 Build Configuration - COMPLETE
- Custom airframe `4251_gz_quadtailsitter_vision`
- Vision velocity + position fusion (`EKF2_EV_CTRL=5`) as primary nav source
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

**Verified success criteria (2026-02-09):**
- `cs_ev_vel: true` — EKF fusing vision velocity
- `cs_ev_pos: true` — EKF fusing vision position (drag bow model)
- `cs_gnss_pos: true` — GPS providing global reference
- `home_position_invalid: false` — Home position set
- `global_position_invalid: false` — Global position valid
- `local_position_invalid: false` — Local position valid
- gz_bridge connected: `world: canyon_world, model: quadtailsitter`
- DDS bridge: all PX4 topics published on ROS2

### Phase 6: Arm & Takeoff Verification - COMPLETE
**Goal:** Verify vehicle can arm, take off, and hover stably
- Motors respond (`/fmu/out/actuator_motors` non-zero) — verified
- Offboard takeoff: arm → climb 10m → hold 30s → land — verified
- Canyon mission: 3.3km back-and-forth at 15m altitude — verified
- ZUPT active during hover (velocity zeroed, position held) — verified
- Fusion position estimate (drag bow model) active — verified

### Phase 7: VTOL Fixed-Wing Transition - COMPLETE
**Goal:** Enable MC→FW→MC transition cycle and FW canyon mission

**Model changes:**
- Added AdvancedLiftDrag plugin to model_px4.sdf (PX4 reference quadtailsitter coefficients)
- Aero forces: forward=Z, upward=-X (tailsitter body frame), area=0.4m², no control surfaces
- Forces scale with v² — negligible in hover, dominant in FW cruise

**Airframe tuning:**
- `VT_F_TRANS_DUR=5.0` — transition pitch-over time (was 1.5, too fast)
- `VT_F_TR_OL_TM=5.0` — open-loop transition complete timer (no airspeed sensor in SITL)
- `VT_FW_MIN_ALT=10` — safety back-transition altitude threshold
- `FW_THR_TRIM=0.65` — FW cruise throttle for altitude maintenance (was 0.35)

**Scripts:**
- `offboard_transition_test.py` — standalone MC→FW→MC transition test
- `offboard_mission.py --vtol` — FW canyon mission (one-way east 100→400m)

**Verified results (2026-02-09):**
- Hover regression: AdvancedLiftDrag has no effect at hover speeds ✓
- FW transition: completes at ~12s, 15-16 m/s ✓
- FW cruise: 4 waypoints at 18-23 m/s, altitude 28-33m ✓
- MC back-transition: completes at ~16s ✓
- RTL and landing in MC mode ✓
- Camera orientations correct in both hover and FW flight ✓

### Phase 8: Performance Comparison - COMPLETE
**Goal:** Compare EKF accuracy between MC and VTOL FW missions

**Test setup:**
- MC: `offboard_mission.py` — back-and-forth at 15m, ~8 m/s, 839m total
- VTOL: `offboard_mission.py --vtol` — FW mission at 30m, 18-23 m/s, 710m total
- Recorder: `record_test_flight.py` at 10Hz (GT vs EKF position + speed)

**Results (2026-02-09):**

| Metric | MC Canyon | VTOL FW | Change |
|--------|-----------|---------|--------|
| Position RMSE | 1.00 m | 0.49 m | **-51%** |
| Position max | 1.99 m | 0.95 m | **-52%** |
| Speed RMSE | 0.117 m/s | 0.066 m/s | **-44%** |
| Drift/km | 2.4 m/km | 1.3 m/km | **-43%** |
| FW cruise (>10 m/s) pos RMSE | N/A | 0.65 m | — |
| Hover (<0.5 m/s) pos RMSE | 0.47 m | 0.09 m | **-81%** |

**Key findings:**
- AdvancedLiftDrag did not degrade EKF accuracy — improved across all metrics
- FW cruise at 18-23 m/s maintains sub-meter position accuracy (0.65m RMSE)
- Stronger velocity signals during FW flight give EKF better observability
- All stretch targets met (drift <5 m/km, speed RMSE <0.2 m/s)

**Caveats:**
- GT velocity from Gazebo Odometry `twist.twist.linear` is body-frame; EKF velocity is NED — direct velocity comparison invalid, speed magnitude used instead
- VTOL recorder captured 300 of 537s (missed back-transition + landing)

---

## File Summary

| File | Description |
|------|-------------|
| `models/quadtailsitter/model.sdf` | VTOL model (fixed joints, no motor plugins) |
| `models/quadtailsitter/model_px4.sdf` | PX4 variant (revolute joints + MulticopterMotorModel + AdvancedLiftDrag) |
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
