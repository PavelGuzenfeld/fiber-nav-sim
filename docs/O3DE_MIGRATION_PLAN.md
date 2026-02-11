# O3DE Migration Plan — Gazebo Harmonic → Open 3D Engine

## Context

The terrain mapping feature (now complete) exposed a fundamental limitation of Gazebo Harmonic with DART physics: **no heightmap or mesh collision from SDF**. Only primitive shapes (plane, box, sphere, cylinder, capsule) are supported. The terrain renders visually with satellite texture for optical flow, but collision is a flat plane — the drone cannot interact with terrain elevation.

This makes Gazebo Harmonic unsuitable for scenarios requiring terrain-following, obstacle avoidance, or realistic ground interaction. Additionally, the visual fidelity is limited (no PBR materials, basic lighting) which degrades optical flow quality.

**Motivation (user-confirmed):**
- Better terrain/physics (PhysX supports heightmap + mesh collision natively)
- Higher visual fidelity (Atom renderer, PBR materials)
- Future-proof platform (game engine ecosystem)

**Constraints (user-confirmed):**
- Must stay headless Docker (non-negotiable)
- Drop gz_bridge, use ROS 2 only for PX4 communication

## Critical Risks & GO/NO-GO Gates

### Risk 1: Headless Docker Rendering (HIGH)
O3DE requires GPU for rendering. `-NullRenderer` disables ALL rendering including cameras. Options:
- `VK_KHR_headless_surface` (Vulkan headless) — needs verification on WSL2 + RTX 3060
- Xvfb + GPU — virtual framebuffer with real GPU rendering
- EGL offscreen — similar to headless Vulkan

**GO/NO-GO: Phase 0 must prove headless GPU rendering works before any other work.**

### Risk 2: PX4 Integration (HIGH)
PX4 does NOT accept raw sensor data via ROS 2/DDS topics. Sensor data comes from internal drivers. Non-Gazebo simulators use the **MAVLink Simulator API** (TCP port 4560) with `HIL_SENSOR` + `HIL_GPS` messages.

Despite user preference for "ROS 2 only", a MAVLink bridge is required for sensor injection. ROS 2 handles everything else (setpoints, telemetry, mode registration).

### Risk 3: VTOL Dynamics (MEDIUM)
O3DE has no VTOL dynamics model. Must implement:
- MulticopterMotorModel equivalent (4 tilt motors)
- AdvancedLiftDrag equivalent (wing aerodynamics)
- Transition logic (MC ↔ FW)
- Tailsitter-specific pitch-forward dynamics

### Risk 4: Missing Sensors (LOW)
O3DE ROS 2 Gem provides: IMU, GNSS, Camera, LiDAR
Missing: Barometer, Magnetometer (need custom O3DE components)

## Architecture

```
Current (Gazebo):
  PX4 ←[gz_bridge]→ Gazebo ←[ros_gz_bridge]→ ROS 2

Target (O3DE):
  PX4 ←[MAVLink TCP:4560]→ O3DE MAVLink Bridge Component
  PX4 ←[XRCE-DDS UDP:8888]→ ROS 2 ←[O3DE ROS 2 Gem]→ O3DE
```

Key difference: PX4 built WITHOUT gz_bridge (`make px4_sitl_default` with `SIM_GZ_EN=0`). Sensor data flows via MAVLink. Commands/telemetry flow via XRCE-DDS/ROS 2.

## Phased Implementation

### Phase 0: Headless Docker Feasibility (GO/NO-GO) — ~3 days

**Goal:** Prove O3DE renders headless in Docker with GPU on WSL2.

**Files:**
- `docker/Dockerfile.o3de` — New Dockerfile with O3DE + Vulkan headless
- `docker/test_headless.py` — Minimal O3DE level load + camera capture test

**Steps:**
1. Create Docker image with O3DE Engine SDK + ROS 2 Gem
2. Configure Vulkan headless surface (no display server)
3. Load minimal level with a camera
4. Capture one frame and verify non-black output
5. Measure GPU memory usage and startup time

**Pass criteria:** Camera produces valid image in Docker without X11/Xvfb.
**Fail action:** Evaluate Xvfb+GPU fallback. If that also fails, abort migration.

### Phase 1: PX4 MAVLink Simulator Bridge — ~5 days

**Goal:** O3DE component that speaks PX4's MAVLink Simulator API.

Can run in parallel with Phase 0 (pure C++ component, testable with mock data).

**Files:**
- `src/fiber_nav_o3de/gems/px4_bridge/` — O3DE Gem
  - `Code/Source/PX4MAVLinkBridge.h/.cpp` — TCP client → HIL_SENSOR, HIL_GPS
  - `Code/Source/PX4MotorOutput.h/.cpp` — Receives actuator commands from PX4
- MAVLink header-only library (C, no external deps)

**Protocol:**
```
O3DE → PX4:  HIL_SENSOR (accel, gyro, mag, baro, 200Hz)
             HIL_GPS (lat/lon/alt/vel, 10Hz)
PX4 → O3DE:  HIL_ACTUATOR_CONTROLS (motor commands, 200Hz)
```

**Steps:**
1. Implement MAVLink TCP connection (connect to PX4 port 4560)
2. Pack simulated sensor data into HIL_SENSOR messages
3. Pack GPS data into HIL_GPS messages
4. Receive HIL_ACTUATOR_CONTROLS for motor mixing
5. Unit test with mock TCP server

### Phase 2: VTOL Vehicle Model (GO/NO-GO) — ~8 days

**Goal:** Quadtailsitter with correct dynamics in O3DE PhysX.

**Files:**
- `src/fiber_nav_o3de/gems/vtol_dynamics/` — O3DE Gem
  - `Code/Source/MulticopterMotorComponent.h/.cpp` — Thrust + torque per motor
  - `Code/Source/AerodynamicsComponent.h/.cpp` — Lift/drag from airspeed + AoA
  - `Code/Source/VTOLTransitionComponent.h/.cpp` — MC↔FW state machine
- `src/fiber_nav_o3de/assets/quadtailsitter/` — Vehicle prefab + PhysX colliders

**Port from Gazebo model (`model_px4.sdf`):**
- Inertia tensor (base_link: 2.36 kg, Ixx=0.16, Iyy=0.22, Izz=0.14)
- 4 revolute motor joints with thrust curves
- AdvancedLiftDrag parameters (CL0, CLa, CD0, CDa, area, etc.)
- Sensor mounting (IMU, GPS, baro, mag, camera positions)

**Pass criteria:** Vehicle hovers stably in MC mode, transitions to FW, flies waypoints.
**Fail action:** Simplify to quad-only (no FW/VTOL) as interim.

### Phase 3: Terrain & World — ~3 days

**Goal:** Reuse existing terrain pipeline with O3DE's native heightmap support.

**Files:**
- `scripts/generate_terrain.py` — Add O3DE heightfield export (existing script)
- `src/fiber_nav_o3de/levels/terrain_world/` — O3DE level
- `src/fiber_nav_o3de/assets/terrain/` — Heightfield + satellite texture

**Key advantage:** O3DE PhysX heightfield supports **real collision** with terrain elevation — the main reason for this migration. No flat ground plane workaround needed.

**Steps:**
1. Export heightmap as O3DE PhysX heightfield asset (RAW16 or image)
2. Apply satellite texture as terrain material (Atom PBR)
3. Configure spherical coordinates for GPS origin
4. Set atmosphere/lighting to match Negev conditions

### Phase 4: Integration & Node Adaptation — ~5 days

**Goal:** Port all ROS 2 nodes and launch files.

**Files to modify:**
- `src/fiber_nav_bringup/launch/simulation.launch.py` — Replace gz_sim with O3DE launch
- `src/fiber_nav_sensors/src/stabilized_flight_controller.cpp` — Remove `gz::transport` dependency, use pure ROS 2
- `src/fiber_nav_sensors/CMakeLists.txt` — Remove `find_package(gz-transport13)`, `find_package(gz-msgs10)`
- `scripts/sim_distance_sensor.py` — May use O3DE raycasting instead of heightmap lookup
- `docker/docker-compose.yml` — O3DE container configuration
- `docker/Dockerfile` — Merge O3DE + PX4 build

**Nodes unaffected (pure ROS 2, no Gazebo dependency):**
- `fiber_spool_sim_node` — Pure ROS 2 publisher
- `vision_direction_sim_node` — Pure ROS 2 publisher
- `drag_bow_fusion_node` — Pure ROS 2 subscriber/publisher
- `terrain_gis_node.py` — Pure Python, reads heightmap data
- All offboard scripts (`offboard_*.py`) — Pure PX4 ROS 2 interface

**Nodes requiring changes:**
- `stabilized_flight_controller` — Has direct `gz::transport::Node` for wind force application via `gz::msgs::EntityWrench`. Replace with O3DE equivalent or ROS 2 service.
- `simulation.launch.py` — Replace `gz_sim`, `ros_gz_bridge`, spawn service with O3DE equivalents.

### Phase 5: End-to-End Validation — ~3 days

**Verification checklist:**
1. Docker headless: O3DE starts, renders terrain, GPU utilized
2. PX4 connection: MAVLink bridge connects, sensor data flows
3. Vehicle stability: MC hover at 30m for 60s, < 1m drift
4. FW flight: MC→FW transition, fly 4 waypoints, MC→land
5. Terrain collision: Vehicle lands on elevated terrain, not through it
6. Optical flow: Downward camera sees texture variation over terrain
7. Distance sensor: Reports correct AGL over varying terrain
8. GPS-denied: Disable GPS mid-flight, complete mission on dead reckoning
9. All existing offboard scripts work unmodified

### Phase 6: Cleanup & CI — ~2 days

- Remove Gazebo dependencies from CMakeLists.txt files
- Remove `gz_bridge` build from PX4 compilation
- Update CI/CD pipeline for O3DE Docker image
- Update documentation
- Archive Gazebo world files (keep for reference)

## File Impact Summary

| Category | Files | Action |
|----------|-------|--------|
| New O3DE Gems | ~15 files | Create |
| New O3DE Assets | ~5 files | Create |
| New Docker | 1 Dockerfile | Create |
| Modified Launch | 1 file | Edit |
| Modified C++ | 2 files | Edit (remove gz deps) |
| Modified CMake | 2 files | Edit (remove gz deps) |
| Modified Docker | 2 files | Edit |
| Modified Scripts | 1 file | Edit (terrain gen) |
| Unchanged | ~20 files | No change |

## Estimated Timeline

| Phase | Duration | Depends On | Parallel? |
|-------|----------|------------|-----------|
| 0: Headless Docker | 3 days | — | — |
| 1: MAVLink Bridge | 5 days | — | Yes (with Phase 0) |
| 2: VTOL Model | 8 days | Phase 0 GO | — |
| 3: Terrain | 3 days | Phase 0 GO | Yes (with Phase 2) |
| 4: Integration | 5 days | Phases 1-3 | — |
| 5: Validation | 3 days | Phase 4 | — |
| 6: Cleanup | 2 days | Phase 5 | — |

**Total: ~22 days** (with parallelism) to **~29 days** (sequential)

## Verification

1. **Phase 0 gate:** `docker run --gpus all fiber-nav-o3de python3 test_headless.py` → camera captures valid frame
2. **Phase 1 test:** MAVLink bridge unit tests pass with mock PX4
3. **Phase 2 test:** Vehicle hovers, transitions, flies waypoints in O3DE
4. **Phase 3 test:** Terrain renders with satellite texture, PhysX collision works on elevated ground
5. **Phase 4 test:** `ros2 launch fiber_nav_bringup simulation.launch.py` starts O3DE + PX4
6. **Phase 5 test:** Full VTOL mission (MC takeoff → FW waypoints → MC land) completes successfully
7. **Phase 5 test:** `scripts/offboard_mission.py --vtol` works unmodified
