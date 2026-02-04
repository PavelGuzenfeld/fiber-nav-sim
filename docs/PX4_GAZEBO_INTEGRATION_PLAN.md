# PX4-Gazebo Native Integration Plan

## Problem Statement

Current setup uses PX4 SIH (Simulator-in-Hardware) which has its **own internal physics simulation** separate from Gazebo. The EKF rejects visual odometry because SIH's internal state differs from Gazebo's actual plane state.

```
Current (Broken):
┌─────────────────┐     ┌─────────────────┐
│ Gazebo Physics  │     │   PX4 SIH       │
│ (plane @ 10m/s) │     │ (internal: 0m/s)│
└────────┬────────┘     └────────┬────────┘
         │                       │
         v                       v
┌─────────────────┐     ┌─────────────────┐
│ Fusion Node     │────▶│ EKF: REJECT!    │
│ (sends 10m/s)   │     │ (expects ~0m/s) │
└─────────────────┘     └─────────────────┘
```

## Solution: Native Gazebo Integration

Replace SIH with PX4's native Gazebo support where sensors come from Gazebo physics:

```
Target (Working):
┌─────────────────────────────────────────┐
│              Gazebo Physics             │
│   (IMU, Baro, Mag + plane odometry)     │
└────────────────┬────────────────────────┘
                 │
    ┌────────────┼────────────┐
    v            v            v
┌────────┐  ┌────────┐  ┌─────────────┐
│  IMU   │  │  Baro  │  │ Fusion Node │
│  Mag   │  │        │  │ (vis odom)  │
└───┬────┘  └───┬────┘  └──────┬──────┘
    │           │              │
    v           v              v
┌─────────────────────────────────────────┐
│        PX4 EKF (consistent state)       │
│   IMU matches visual odometry: ACCEPT   │
└─────────────────────────────────────────┘
```

---

## Implementation Phases

### Phase 1: Model Preparation (1-2 hours)
**Goal:** Add required sensor plugins to plane model SDF

1. **Add IMU sensor plugin** to `plane/model.sdf`:
```xml
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
  <topic>/imu</topic>
</plugin>

<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>250</update_rate>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.009</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></x>
      <y><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></y>
      <z><noise type="gaussian"><mean>0</mean><stddev>0.021</stddev></noise></z>
    </linear_acceleration>
  </imu>
</sensor>
```

2. **Add air pressure (barometer) sensor**:
```xml
<sensor name="air_pressure_sensor" type="air_pressure">
  <always_on>true</always_on>
  <update_rate>50</update_rate>
  <air_pressure>
    <pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure>
  </air_pressure>
</sensor>
```

3. **Add magnetometer sensor**:
```xml
<sensor name="magnetometer_sensor" type="magnetometer">
  <always_on>true</always_on>
  <update_rate>50</update_rate>
  <magnetometer>
    <x><noise type="gaussian"><mean>0</mean><stddev>0.006</stddev></noise></x>
    <y><noise type="gaussian"><mean>0</mean><stddev>0.006</stddev></noise></y>
    <z><noise type="gaussian"><mean>0</mean><stddev>0.006</stddev></noise></z>
  </magnetometer>
</sensor>
```

### Phase 2: PX4 Build Configuration (2-3 hours)
**Goal:** Build PX4 with native Gazebo Harmonic support

1. **Update Dockerfile** to build PX4 with Gazebo:
```dockerfile
# Build PX4 with Gazebo support
RUN cd /root/PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl gz_x500
```

2. **Create custom airframe** for fixed-wing with vision:
```bash
# /etc/init.d-posix/airframes/4250_gz_plane_vision
#!/bin/sh
. ${R}etc/init.d/rc.fw_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=plane}

# Enable Gazebo lockstep
param set-default SIM_GZ_EN 1

# GPS-denied with vision velocity
param set-default SYS_HAS_GPS 0
param set-default EKF2_GPS_CTRL 0
param set-default EKF2_EV_CTRL 4
param set-default EKF2_EVV_NOISE 0.15
param set-default EKF2_EV_DELAY 20
param set-default COM_ARM_WO_GPS 1
```

3. **Register airframe** in CMakeLists or airframes config

### Phase 3: Bridge Configuration (1 hour)
**Goal:** Bridge Gazebo sensors to PX4

1. **Update ros_gz_bridge** to include sensor topics:
```yaml
# config/ros_gz_bridge.yaml
- topic_name: /world/canyon_world/model/plane/link/base_link/sensor/imu_sensor/imu
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS

- topic_name: /world/canyon_world/model/plane/link/base_link/sensor/air_pressure_sensor/air_pressure
  ros_type_name: sensor_msgs/msg/FluidPressure
  gz_type_name: gz.msgs.FluidPressure
  direction: GZ_TO_ROS
```

2. **Alternative: Use GZBridge plugin** (PX4's native approach):
   - PX4's `GZBridge` module directly subscribes to Gazebo topics
   - No need for ros_gz_bridge for sensor data
   - Only need ros_gz_bridge for odometry (ground truth) and cameras

### Phase 4: Integration Testing (1-2 hours)
**Goal:** Verify EKF accepts visual odometry

1. **Start simulation** with new configuration
2. **Verify sensor data flow**:
```bash
# Check Gazebo publishes IMU
gz topic -l | grep imu
gz topic -e -t /world/canyon_world/model/plane/link/base_link/sensor/imu_sensor/imu

# Check PX4 receives sensor data
ros2 topic echo /fmu/out/sensor_combined --once
```

3. **Verify EKF status**:
```bash
ros2 topic echo /fmu/out/estimator_status_flags --qos-reliability best_effort | grep ev_vel
# Should show: cs_ev_vel: true
```

4. **Run benchmark**:
```bash
python3 scripts/record_test_flight.py 30
python3 scripts/analyze_flight.py /tmp/flight_data.csv
```

---

## File Changes Summary

| File | Change |
|------|--------|
| `src/fiber_nav_gazebo/models/plane/model.sdf` | Add IMU, baro, mag sensors |
| `docker/Dockerfile` | Build PX4 with `gz_x500` target |
| `src/fiber_nav_bringup/config/px4_params.txt` | Add `SIM_GZ_EN=1` |
| `src/fiber_nav_bringup/launch/px4_sitl.launch.py` | Use gz airframe instead of sihsim |

---

## Expected Results After Integration

| Metric | Before (SIH) | After (Native GZ) | Notes |
|--------|--------------|-------------------|-------|
| cs_ev_vel | false | **true** | EKF accepts vision |
| Velocity RMSE | 0.127 m/s | ~0.127 m/s | Fusion unchanged |
| EKF vs GT RMSE | N/A | <0.2 m/s | New metric |
| Position drift | 28 m/1000m | <15 m/1000m | EKF helps reduce |

---

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Gazebo Harmonic compatibility | Use `gz-harmonic` package, verify plugin availability |
| PX4 build complexity | Start with `gz_x500` quadrotor first, then port to plane |
| Sensor topic naming | Use `gz topic -l` to discover actual topic names |
| Lockstep timing issues | Ensure `SIM_GZ_EN=1` and proper world plugin |

---

## Quick Start Commands (After Implementation)

```bash
# Terminal 1: Start Gazebo with new model
ros2 launch fiber_nav_bringup simulation.launch.py

# Terminal 2: Start PX4 with native Gazebo
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4250 PX4_GZ_MODEL=plane make px4_sitl gz_plane_vision

# Terminal 3: Start MicroXRCE-DDS
MicroXRCEAgent udp4 -p 8888

# Terminal 4: Start fusion node
ros2 run fiber_nav_fusion fiber_vision_fusion

# Terminal 5: Apply thrust and record
ros2 run fiber_nav_sensors plane_controller --ros-args -p thrust:=15.0
python3 scripts/record_test_flight.py 30
```

---

## References

- [PX4 Gazebo Harmonic Docs](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [PX4 Gazebo Models Repo](https://github.com/PX4/PX4-gazebo-models)
- [Gazebo Sensor Plugins](https://gazebosim.org/docs/harmonic/sensors)
- [EKF2 External Vision](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
