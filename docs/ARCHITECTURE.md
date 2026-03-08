# Fiber Navigation Simulation — Architecture

## System Overview

GPS-denied VTOL navigation using fiber optic cable odometry, monocular vision fusion, terrain-aided dead reckoning (TERCOM), and position EKF. Fully Dockerized ROS 2 Jazzy / Gazebo Harmonic / PX4 SITL stack.

---

## Standalone Mode (wrench-based flight)

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

---

## PX4 Mode (offboard flight control)

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
| * 2-axis gimbal (yaw+pitch)  |                |
+-----------+------------------+                |
            |                                   |
            | ros_gz_bridge                     |
            | (odometry, cameras, joints)       |
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
+=+======+======+======+======+======+======+======+======+======+
  |      |      |      |      |      |      |      |      |
  v      v      v      v      v      v      v      v      v
+----+ +----+ +-----+ +----+ +-----+ +----+ +----+ +----+ +-----+
|spol| |vis | |fiber| |sim | |pos  | |terc| |gimb| |cabl| |foxgl|
|_sim| |_dir| |_vis | |_dst| |_ekf | |_om | |_ctl| |_dyn| |_brdg|
|_drv| |_sim| |_fus | |_sns| |_node| |_nod| |_nod| |_nod| |     |
+--+-+ +--+-+ +-+--++ +--+-+ +--+--+ +-+--+ +-+--+ +-+--+ +-----+
   |      |     | ^      |      |       |      |      |
   |      |     | |      |      |       |      |      |
   |      |     v |      v      v       v      |      v
   |      |  /fmu/in/  /fmu/in/  /position_ekf/ |  /world/.../
   |      |  vehicle_  distance_ /tercom/    /gimbal/ wrench
   |      |  visual_   sensor    position    cmd_pos  (gz)
   v      v  odometry
/sensors/ /sensors/
fiber_    vision_
spool/    _direction
   |         |
   +----+----+-------> position_ekf_node
   |    |    |         * 6-state EKF (pos, vel, wind)
   |    |    |         * GPS-denied dead reckoning
   |    |    |         * terrain-anchored altitude
   |    |    |         * cable length constraint
   |    |    |
   |    |    +-------> tercom_node
   |    |              * 2D terrain profile matching
   |    |              * anisotropic covariance
   |    |              * coarse-to-fine search
   |    |
   +----+------------> fiber_vision_fusion
                       * body-to-NED rotation
                       * ZUPT + drag bow position
                       * terrain Z from EKF
```

> **Note:** The quadtailsitter model has two configurations. In standalone mode (wrench-based),
> motor plugins are disabled and rotor joints are fixed — flight is controlled via
> the `stabilized_flight_controller` applying one-time wrenches. In PX4 mode, motors are
> enabled with revolute joints, MulticopterMotorModel x4, and AdvancedLiftDrag aerodynamics
> for full VTOL MC/FW flight.

---

## Data Pipeline

### Fusion Pipeline

```
Gazebo Odometry (50Hz)
    |
    +---> spool_sim_driver          +---> vision_direction_sim
    |     |v| + noise + slack       |     unit vector + drift
    |     /sensors/fiber_spool/     |     /sensors/vision_direction
    |     velocity + status         |
    |                               |
    +----------- + -----------------+
                 |
                 v
        fiber_vision_fusion
        1. Sensor health (ring buffer, 100-sample window)
        2. Adaptive ZUPT (noise floor tracking -> dynamic threshold)
        3. Online slack calibration (EKF cross-validation when GPS healthy)
        4. v_body = (spool_speed / slack) * direction_unit_vector
        5. v_ned = attitude_q * v_body_frd * attitude_q^-1
        6. Heading cross-check (fused vs EKF heading, penalize >30deg divergence)
        7. Cross-validation scaling (spool vs EKF speed agreement)
        8. Attitude staleness scaling (decay to zero over 1s)
        9. Flight-mode variance (MC: 0.01, FW: 0.01, transitions: 0.04)
        10. Drag bow 1D position estimate (along tunnel axis)
                 |
                 v
        /fmu/in/vehicle_visual_odometry -> PX4 EKF2
```

### Terrain Pipeline (offline generation + runtime queries)

```
SRTM DEM (30m resolution)
    |
    v
generate_terrain.py -> heightmap_513x513.png + satellite texture + terrain_data.json
    |                                          |
    v                                          v
terrain_world.sdf (Gazebo)              terrain_gis_node.py (runtime)
                                        /terrain/query -> /terrain/height
                                              |
                                              v
                                   TerrainAltitudeController (C++)
                                   look-ahead + feed-forward + P-ctrl
                                   filtered AMSL target for FW terrain-following
```

### TERCOM Terrain Matching

```
Baro altitude + AGL rangefinder -> measured terrain profile (tercom_node)
DEM heightmap + candidate positions -> reference profiles
Coarse grid search (2x step) -> top-N refinement -> best match
NCC Hessian -> anisotropic 2x2 covariance (direction-dependent error)
/tercom/position -> position_ekf_node (measurement update)
```

### Position EKF (GPS-denied dead reckoning)

```
                     +------------------+
                     | position_ekf_node|
                     | 6-state EKF      |
                     +--------+---------+
                              |
              +-------+-------+-------+-------+
              |       |       |       |       |
              v       v       v       v       v
           Predict  Vel.   TERCOM  Cable   Terrain
           (spool+  Update  Fix    Range   Altitude
            OF+att)  (NED)  (2D)   (meas)  (DEM+RF)
              |       |       |       |       |
              v       v       v       v       v
    State: [pos_n, pos_e, vel_n, vel_e, wind_n, wind_e]

Inputs:
  Spool velocity + OF direction + attitude -> NED velocity (predict)
  TERCOM position fix -> position measurement update (anisotropic)
  Cable deployed length -> range measurement (continuous, not gated)
  DEM + rangefinder -> terrain-anchored NED Z -> fiber_vision_fusion -> PX4

Outputs:
  /position_ekf/estimate    -> PoseWithCovarianceStamped (position + XY cov)
  /position_ekf/terrain_z   -> Float64 (NED Z for fusion node)
  /position_ekf/state       -> PointStamped (search center for TERCOM)
  /fmu/in/vehicle_visual_odometry -> VehicleOdometry (position to PX4)

Features:
  * GPS->GPS-denied transition with auto-initialization
  * Wind estimation with pump-up mitigation
  * Adaptive OF quality-based measurement noise
  * Path prior: terrain discriminability cross-track constraint
```

### Gimbal Controller

```
Attitude quaternion -> gravity in body frame -> g_body = R^T * [0,0,-1]
Yaw target = atan2(gy, gx)           (roll compensation)
Pitch target = -atan2(sqrt(gx^2+gy^2), -gz)
Low-pass + rate limiting -> /gimbal/cmd_pos, /gimbal/pitch_cmd_pos
Saturation ratio -> VTOL nav mode throttles turn rate + altitude rate
```

---

## VTOL Navigation State Machine

```
MC_CLIMB -> TRANSITION_FW -> FW_NAVIGATE -> FW_RETURN -> TRANSITION_MC -> MC_APPROACH -> DONE
```

### GPS-Denied Navigation (v3 — Position-Based)

| Feature | Description |
|---------|-------------|
| PX4 position navigation | Uses PX4 EKF position (velocity-integrated, no GPS) instead of manual dead reckoning |
| Position-based WP acceptance | Projects position onto leg vector — immune to sim/wall time-base issues |
| L1 cross-track correction | Steers back toward planned path using `leg_heading - atan2(cross_track, lookahead)` |
| Wind correction | Wind triangle from EKF wind estimate applied to FW course |
| Distance-based return | FW_RETURN steers toward home using `atan2(-pos.y, -pos.x)`, completes at d < 200m |
| GPS re-enable for landing | Re-enables EKF2_GPS_CTRL=7 for MC approach, keeps rangefinder height ref |
| Gimbal accommodation | Throttles turn rate when gimbal yaw saturated, altitude rate when pitch saturated |
| Terrain following | Filtered AMSL target from TerrainAltitudeController with feed-forward |
| Cable monitoring | Warn/abort thresholds on cable tension |
| Ground truth logging | Gazebo odometry subscribed for diagnostic comparison only (never in control loop) |

### GPS-Denied Configuration (`gps_denied_mission.yaml`)

```yaml
gps_denied:
  enabled: true
  wp_time_s: 25.0                    # Default time per WP leg (safety fallback)
  return_time_s: 600.0               # Safety timeout (distance-based is primary)
  descent_time_s: 600.0              # MC descent safety timeout
  fw_speed: 18.0                     # FW cruise speed (m/s)
  use_position_ekf: true             # Subscribe to EKF position
  use_position_ekf_navigate: false   # EKF drifts too much for cross-track
  ekf_cross_track_lookahead: 200.0   # L1 lookahead distance (m)
  turn_heading_tolerance_deg: 10.0

# TERCOM-optimized route: 8 waypoints through discriminability corridors
# All turns <= 18.6 deg (within 30 deg tailsitter limit)
# Total outbound: 1257m
waypoints:
  x: [32.4, 64.3, 110.9, 172.4, 594.3, 648.2, 695.2, 800.0]
  y: [291.4, 360.0, 416.4, 460.5, 590.4, 620.8, 661.1, 800.0]
  wp_time_s: [19.3, 7.2, 7.1, 7.2, 27.5, 6.4, 6.4, 12.7]
```

---

## Node Inventory

| Node | Package | Description |
|------|---------|-------------|
| `spool_sim_driver` | fiber_nav_sensors | Fiber spool velocity simulation |
| `vision_direction_sim` | fiber_nav_sensors | Optical flow direction simulation |
| `mock_attitude_publisher` | fiber_nav_sensors | Fixed attitude for standalone mode |
| `stabilized_flight_controller` | fiber_nav_sensors | PD wrench flight controller |
| `cable_dynamics_node` | fiber_nav_sensors | Virtual fiber force model |
| `gimbal_controller_node` | fiber_nav_sensors | 2-axis nadir tracking gimbal |
| `fiber_vision_fusion` | fiber_nav_fusion | Velocity fusion (spool + vision -> VehicleOdometry) |
| `position_ekf_node` | fiber_nav_fusion | 6-state position EKF for GPS-denied |
| `tercom_node` | fiber_nav_fusion | Terrain profile matching (TERCOM) |
| `hold_mode_node` | fiber_nav_mode | PX4 position hold mode |
| `canyon_mission_node` | fiber_nav_mode | PX4 canyon waypoint mission |
| `vtol_navigation_node` | fiber_nav_mode | PX4 VTOL GPS-denied mission |
| `terrain_gis_node.py` | scripts | Terrain height query service |
| `map_bridge_node.py` | scripts | NavSatFix + ground truth + GeoJSON for Foxglove Map |
| `sim_distance_sensor.py` | scripts | Terrain-aware AGL distance sensor |

---

## Docker Services

| Service | Container | Description |
|---------|-----------|-------------|
| `simulation` | fiber-nav-sim | Full Gazebo GUI + ROS 2 |
| `standalone` | fiber-nav-standalone | Headless, no PX4 (mock attitude, auto-fly) |
| `px4-sitl` | fiber-nav-px4-sitl | 9-phase orchestrator (Gazebo + PX4 + sensors + mission) |
| `dev` | fiber-nav-dev | Interactive shell (same image, no entrypoint) |
| `test` | fiber-nav-test | Build + run unit tests |
| `ci` | fiber-nav-ci | Headless CI testing |
| `analysis` | fiber-nav-analysis | Jupyter notebook (port 8888) |

### PX4 SITL Orchestrator (9 phases)

```
Phase 0: Rebuild workspace (colcon build --symlink-install)
Phase 1: MicroXRCE-DDS Agent (UDP 8888)
Phase 2: Gazebo + sensors + Foxglove (simulation.launch.py)
Phase 3: sim_distance_sensor.py (terrain AGL)
Phase 4: terrain_gis_node.py (GIS queries)
Phase 5: PX4 SITL (airframe 4251, output to /dev/null)
Phase 6: map_bridge_node.py (Foxglove Map)
Phase 7: cable_dynamics_node (fiber dynamics)
Phase 8: Mission auto-launch (MISSION env var)
Phase 9: Ready
```

---

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/model/quadtailsitter/odometry` | nav_msgs/Odometry | Ground truth from Gazebo |
| `/sensors/fiber_spool/velocity` | std_msgs/Float32 | Scalar spool velocity (m/s) |
| `/sensors/fiber_spool/status` | SpoolStatus | Velocity + total length + is_moving |
| `/sensors/vision_direction` | Vector3Stamped | Unit direction vector |
| `/fmu/in/vehicle_visual_odometry` | VehicleOdometry | Fusion output to PX4 |
| `/fmu/in/distance_sensor` | DistanceSensor | Terrain-aware AGL |
| `/fmu/out/vehicle_attitude` | VehicleAttitude | PX4 attitude |
| `/fmu/out/vehicle_local_position` | VehicleLocalPosition | PX4 position |
| `/fmu/out/estimator_status_flags` | EstimatorStatusFlags | GPS health |
| `/position_ekf/estimate` | PoseWithCovarianceStamped | EKF position + XY covariance |
| `/position_ekf/velocity` | TwistStamped | EKF NED velocity |
| `/position_ekf/wind` | Vector3Stamped | Wind vector estimate |
| `/position_ekf/terrain_z` | Float64 | Terrain-anchored NED Z |
| `/position_ekf/distance_home` | Float64 | Distance from home (m) |
| `/position_ekf/diagnostics` | String | EKF JSON status |
| `/tercom/position` | PoseWithCovarianceStamped | TERCOM fix (anisotropic cov) |
| `/tercom/quality` | Float64 | Peak ambiguity ratio |
| `/gimbal/cmd_pos` | Float64 | Gimbal yaw command (rad) |
| `/gimbal/pitch_cmd_pos` | Float64 | Gimbal pitch command (rad) |
| `/gimbal/saturation` | Float64 | Gimbal yaw saturation [0..1] |
| `/cable/status` | CableStatus | Tension, lengths, forces, break state |
| `/cable/tension` | Float64 | Scalar cable tension (N) |
| `/terrain/query` | Point | Query terrain height at (x, y) |
| `/terrain/height` | Float64 | Terrain height response |
| `/vehicle/nav_sat_fix` | NavSatFix | Vehicle GPS for Foxglove Map |
| `/vehicle/ground_truth_fix` | NavSatFix | Gazebo ground truth for Foxglove Map (diagnostic) |
| `/map/terrain_overlay` | GeoJSON | Terrain elevation heatmap |
| `/map/mission_plan` | GeoJSON | Mission waypoints + path |
| `/camera` | Image | Forward camera feed |
| `/camera_down` | Image | Downward camera feed |
| `/follow_camera` | Image | 3rd person follow camera |
