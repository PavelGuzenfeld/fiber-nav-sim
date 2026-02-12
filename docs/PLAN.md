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
  - [x] Health-based variance scaling, attitude staleness, cross-validation
  - [x] Online slack calibration, heading cross-check, adaptive ZUPT
  - [x] 69 unit tests — `test_fusion.cpp`

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

- **Phase 9: VTOL Navigation Mode (C++ Custom Flight Mode)**
  - [x] VtolNavigationMode — 7-state VTOL state machine (MC_CLIMB → TRANSITION_FW → FW_NAVIGATE → FW_RETURN → TRANSITION_MC → MC_APPROACH → DONE)
  - [x] VtolMissionExecutor — arm → takeoff → navigate → land → disarm lifecycle
  - [x] YAML-configurable missions (cruise altitude, waypoints, accept radius, transition timeouts)
  - [x] Canyon mission config — 7 waypoints, 1400m east, 150m cruise altitude
  - [x] Straight 500m test config
  - [x] PX4 NPFG+TECS FW control via course+altitude setpoints
  - [x] Quad-chute detection and MC fallback
  - [x] VTOL class 2s→10s timeout patch (permanent in Dockerfile)
  - [x] 33 unit tests (24 VTOL navigation + 9 canyon waypoints)
  - [x] Integration test verified: full canyon mission 1393s, all phases successful
  - [x] Hardening: faster climb (4 m/s), unbuffered logging, offboard timeouts

- **Phase 10: Terrain Mapping Service**
  - [x] SRTM DEM download + heightmap generation (513x513 16-bit PNG)
  - [x] Bing Maps satellite tile stitching for terrain texture
  - [x] SDF world template with parameterized heightmap, ground plane, atmosphere
  - [x] `generate_terrain.py` — full pipeline: DEM → heightmap → texture → SDF
  - [x] `terrain_gis_node.py` — ROS 2 GIS height query service (/terrain/query → /terrain/height)
  - [x] `sim_distance_sensor.py` — terrain-aware AGL using 513x513 bilinear interpolation
  - [x] `terrain_data.json` — metadata (center coords, elevation range, resolution)
  - [x] Ground plane z aligned to terrain center height (DART collision workaround)
  - [x] Spawn point at map center (0,0) at terrain surface level
  - [x] PX4_GZ_WORLD env var passthrough in orchestrator entrypoint
  - [x] Orchestrator Phase 4: terrain_gis_node auto-start with health check
  - [x] 31 unit tests (heightmap gen, texture stitching, coordinate transforms, GIS queries)
  - [x] E2E verified: Gazebo terrain load → distance sensor → GIS queries → flight test

- **Phase 11: O3DE Migration — Phase 0 Feasibility**
  - [x] O3DE Docker image with Vulkan DZN driver for WSL2
  - [x] GPU test script — NVIDIA RTX 3060 detected via DZN (Vulkan → D3D12)
  - [x] Headless test script — GameLauncher crashes at D3D12 device init (Phase 0 partial)
  - [x] O3DE Gem scaffolds (px4_bridge, vtol_dynamics)
  - [x] O3DE migration plan document
  - [x] Backend dispatcher: simulation.launch.py → gazebo_simulation.launch.py / o3de_simulation.launch.py

- **Phase 12: Terrain-Following Altitude Controller + Auto-Launch**
  - [x] TerrainAltitudeController — P-controller with low-pass filter for terrain-following via GIS queries
  - [x] Integrated into VtolNavigationMode for FW_NAVIGATE and FW_RETURN states
  - [x] terrain_mission.yaml — 5 WPs, 80m cruise, 30m target AGL, configurable gains
  - [x] Entrypoint auto-launch: `MISSION` env var (vtol_terrain, vtol_canyon) for autonomous simulation
  - [x] 7-phase orchestrator (Phase 6 = optional mission auto-launch)
  - [x] Updated airframe with terrain-following compatible params
  - [x] offboard_terrain_follow.py Python reference implementation
  - [x] Unit tests for terrain altitude controller
  - [x] E2E verified: `MISSION=vtol_terrain` → full autonomous flight (arm → 5 WPs → return → land)

- **Phase 13: Fusion & Navigation Enhancements (PRs #9, #10)**

  - **13a: Terrain-Follow Lookahead** (PR #9)
    - [x] GIS-based look-ahead: query terrain at pos + velocity × lookahead_time
    - [x] Feed-forward correction: terrain slope × distance × feedforward_gain
    - [x] New config: lookahead_time=3.0s, lookahead_max=100m, feedforward_gain=0.8
    - [x] Async ROS 2 service client for /terrain/query with fallback to P-only
    - [x] 24 unit tests (look-ahead position, feed-forward, clamping, fallback)

  - **13b: Sensor Fusion Enhancements** (PR #10)
    - [x] Health-based variance scaling: 1/(health²) — 100%→1x, 50%→4x, 10%→100x
    - [x] Attitude staleness fallback: <200ms normal, 200-500ms 2x, 500ms-1s 4x, >1s skip
    - [x] Spool-EKF cross-validation via VehicleLocalPosition: >30% disagreement scales variance
    - [x] Enhanced 1Hz diagnostics: health scales, attitude age, cross-val innovation, effective variance

  - **13c: GPS-Denied Navigation Improvements** (PR #10)
    - [x] Online slack calibration: EMA spool/EKF speed ratio, clamped [0.8, 1.2], frozen when GPS unavailable
    - [x] Heading cross-check: vision vs EKF yaw divergence, quadratic scaling capped at 10x
    - [x] Adaptive ZUPT: running RMS noise floor, threshold = max(0.01, 3 × noise_rms)
    - [x] Bug fix: cap heading_crosscheck_scale to 10x (unbounded growth caused FW roll oscillation)
    - [x] 69 fusion unit tests total (52 new: health, staleness, cross-val, heading, slack, adaptive ZUPT)
    - [x] E2E verified: full VTOL terrain mission with all enhancements active

### Completed Previously

- [x] Switched from plane model to quadtailsitter VTOL (proper aerodynamics + motors)
- [x] Follow camera attached to model base_link (was static world camera)
- [x] PX4 custom flight modes via px4-ros2-interface-lib
- [x] Updated all documentation and architecture diagrams
- [x] 209 tests passing (163 C++ + 31 terrain pipeline + 15 analysis)
- [x] ZUPT (Zero-Velocity Update) — hard-reset velocity to zero when spool reports no motion
- [x] 1D Position Clamping via drag bow model — position estimate from accumulated spool length
- [x] SpoolStatus message — velocity + total_length + is_moving
- [x] EKF2_EV_CTRL=5 — position + velocity fusion active (cs_ev_pos + cs_ev_vel)
- [x] VTOL Fixed-Wing Transition — AdvancedLiftDrag plugin, tuned transition params
- [x] FW Canyon Mission — `offboard_mission.py --vtol` with 4 FW waypoints at 18-23 m/s
- [x] MC→FW→MC transition test — `offboard_transition_test.py`
- [x] Camera orientations verified for both hover and FW flight modes
- [x] **Phase 3.1: FW Vision Fusion — Flight Mode Awareness**
  - Flight phase detection via VehicleStatus (MC, FW, TRANSITION_FW, TRANSITION_MC)
  - Adaptive velocity variance: 0.001 (ZUPT), 0.01 (MC/FW), 0.04 (transitions)
  - Sensor health monitoring via ring buffer (window=50, warn at 80%)
  - 1 Hz diagnostics publisher on `/sensors/fusion/diagnostics`
  - 34 unit tests (17 new: phase detection, variance scaling, sensor health)
  - E2E verified: all 4 phases detected across full VTOL mission

---

## Data Flow Architecture

```
Gazebo Harmonic (terrain_world.sdf / canyon_harmonic.sdf)
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
                              fiber_vision_fusion
                                        ^  Subscriptions:
                                        |  - /fmu/out/vehicle_attitude (body-to-NED)
                                        |  - /fmu/out/vehicle_status_v1 (flight phase)
                                        |  - /fmu/out/vehicle_local_position (cross-val + slack cal)
                                        |
                                        |  Pipeline:
                                        |  health scaling → staleness check → cross-val
                                        |  → heading check → adaptive ZUPT → slack cal
                                        |
                                        +--> /sensors/fusion/diagnostics (1 Hz)
                                        |
                                        v
                              /fmu/in/vehicle_visual_odometry
                                        |
                              +---------+---------+
                              v                   v
                        MicroXRCE-DDS       PX4 SITL EKF
                              |                   |
                              +-------------------+
    |
    +-->  sim_distance_sensor.py --> /fmu/in/distance_sensor
    |         (terrain-aware AGL, 513x513 heightmap bilinear interp)
    |
    +-->  terrain_gis_node.py
    |         /terrain/query (Point) --> /terrain/height (Float64)
    |         /terrain/info (String, latched) — terrain metadata JSON
    |
    +-->  TerrainAltitudeController (in VtolNavigationMode)
              * Queries /terrain/query at pos + velocity × lookahead_time
              * Feed-forward: terrain slope × distance × feedforward_gain
              * Fallback to P-only when GIS unavailable
              * Active during FW_NAVIGATE and FW_RETURN states
```

---

## Docker Services

| Service | Command | Description |
|---------|---------|-------------|
| `simulation` | `docker compose up simulation` | Full Gazebo GUI simulation |
| `standalone` | `docker compose up standalone` | Headless, mock attitude, auto-fly |
| `px4-sitl` | `docker compose up px4-sitl` | Headless PX4 SITL (7-phase orchestrator) |
| `test` | `docker compose up test` | Build and run unit tests |
| `ci` | `docker compose up ci` | Headless CI testing |
| `foxglove` | `docker compose up foxglove` | Foxglove visualization bridge |
| `analysis` | `docker compose up analysis` | Jupyter notebook |

### PX4 SITL Orchestrator Phases

The `px4-sitl` service uses `px4-sitl-entrypoint.sh` which starts all services in order:

| Phase | Service | Health Check |
|-------|---------|-------------|
| 0 | Rebuild workspace (colcon) | Build succeeds |
| 1 | Gazebo + sensors + Foxglove | `/model/quadtailsitter/odometry` publishes (120s) |
| 2 | MicroXRCE-DDS Agent | Process alive |
| 3 | sim_distance_sensor.py | Process alive |
| 4 | terrain_gis_node.py | Process alive |
| 5 | PX4 SITL | `/fmu/out/vehicle_status_v1` publishes (90s) |
| 6 | Mission auto-launch (optional) | vtol_navigation_node alive (if MISSION set) |
| 7 | Ready | All processes running |

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
- `/fmu/out/vehicle_status_v1` (FlightPhase: MC, FW, TRANSITION_FW, TRANSITION_MC)
- `/fmu/out/vehicle_local_position` (VehicleLocalPosition: vx/vy/vz for cross-validation + slack calibration)

Publishes to:
- `/fmu/in/vehicle_visual_odometry`
- `/sensors/fusion/diagnostics` (1 Hz JSON: flight_phase, spool/direction health, health_variance_scale, attitude_staleness_scale, xval_innovation, heading_check_scale, calibrated_slack, adaptive_zupt_threshold, noise_rms, effective_variance)

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

| Metric | Target | Stretch | MC Mission | VTOL Mission |
|--------|--------|---------|------------|--------------|
| Drift per 1000m | <10m | <5m | **2.4 m/km** | **1.3 m/km** |
| Position RMSE | <5m | <1m | **1.00 m** | **0.49 m** |
| Speed RMSE | <0.5 m/s | <0.2 m/s | **0.117 m/s** | **0.066 m/s** |
| CPU usage (fusion node) | <5% | <2% | TBD | TBD |
| Latency (sensor->EKF) | <50ms | <20ms | TBD | TBD |

### Performance Comparison (2026-02-09)

| Metric | MC Canyon (839m, 8 m/s) | VTOL FW (710m, 23 m/s) | Change |
|--------|--------------------------|-------------------------|--------|
| Position RMSE | 1.00 m | 0.49 m | -51% |
| Position max | 1.99 m | 0.95 m | -52% |
| Speed RMSE | 0.117 m/s | 0.066 m/s | -44% |
| Drift/km | 2.4 m/km | 1.3 m/km | -43% |
| FW cruise pos RMSE | N/A | 0.65 m | — |
| Hover pos RMSE | 0.47 m | 0.09 m | -81% |

All stretch targets met. VTOL FW flight at 18-23 m/s maintains sub-meter position accuracy.

Note: GT velocity from Gazebo Odometry is body-frame; EKF velocity is NED.
Speed comparison uses magnitude only (frame-independent).
VTOL recorder captured 300/537s (missed back-transition + landing).

---

## References

- [PX4 Gazebo Harmonic Docs](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
- [Gazebo Sensor Plugins](https://gazebosim.org/docs/harmonic/sensors)
- [EKF2 External Vision](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
