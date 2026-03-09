# O3DE Phase 0 Retry: Mesa 26.0.0 DZN + Colosseum Fallback

## Context

O3DE headless rendering failed on WSL2 due to a cubemap bug in Mesa DZN 24.3.4 (Vulkan-on-D3D12 driver). Meanwhile, we validated UE4/AirSim (Colosseum) works at 30 FPS FHD using Mesa's **OpenGL** D3D12 Gallium driver on Ubuntu 24.04.

Key insight: Mesa 26.0.0 (released 2026-02-11) is a full year of DZN Vulkan fixes ahead of our 24.3.4 build. The cubemap issue may be fixed. Before committing to a complex Colosseum migration, we test O3DE with the latest Mesa.

**Strategy**: Quick Mesa upgrade test (~2-3 hours). If O3DE renders -> proceed with O3DE. If not -> Colosseum fallback (plan below).

---

## Part A: O3DE with Mesa 26.0.0 (Quick Test)

### Step 1: Minimal Mesa DZN test image

Build a stripped-down image with just Mesa 26.0.0 DZN + vulkaninfo (skip O3DE to save time):

**File**: `docker/Dockerfile.o3de` -- modify Mesa build block (lines 102-118)

Change:
```
git clone --depth 1 --branch mesa-24.3.4 https://gitlab.freedesktop.org/mesa/mesa.git
```
To:
```
git clone --depth 1 --branch mesa-26.0.0 https://gitlab.freedesktop.org/mesa/mesa.git
```

Build just through Mesa layer using `--target` or by creating a minimal test Dockerfile that only includes the Mesa build + vulkaninfo.

### Step 2: Test DZN Vulkan

```bash
docker run --gpus all --rm \
    -v /usr/lib/wsl:/usr/lib/wsl:ro \
    <mesa-test-image> vulkaninfo --summary
```

**PASS**: Shows `NVIDIA GeForce RTX 3060` (not llvmpipe/swrast)
**FAIL**: Crashes or shows only software driver

### Step 3: If Vulkan works -- full O3DE build

Build the complete `Dockerfile.o3de` with Mesa 26.0.0. Run `test_headless.sh`.

Three tests:
1. Vulkan GPU detection
2. GameLauncher starts, ROS 2 nodes appear
3. Camera captures non-empty, non-black image

**GO** -> O3DE Phase 2 (add gems to project, create level, attach components)
**NO-GO** -> Proceed to Part B (Colosseum)

---

## Part B: Colosseum Migration (Fallback)

Only if Part A fails. Full plan for adding Colosseum as third backend on `feature/colosseum-backend`.

### Architecture

```
UE4 Blocks <--msgpack-rpc (TCP:41451)--> C++ Bridge Node (ROS 2)
  (AirSim)                                  publishes all sensor topics
     |                                              |
     | MAVLink (TCP:4560)                           v
     | HIL sensors + motor cmds            sensors.launch.py (unchanged)
     v                                     fusion, spool, vision, etc.
  PX4 SITL
```

Phase 4 adds external VTOL physics override:
```
PX4 motor cmds --> VTOL Physics Node --> simSetVehiclePose() --> AirSim
                   (aero model ported      (200 Hz)
                    from O3DE gems)
```

### Phase 1: Docker + Rendering (3-4 days)

| File | Description |
|------|-------------|
| `docker/Dockerfile.colosseum` | Ubuntu 24.04 + Mesa D3D12 + ROS 2 Jazzy + Blocks binary + msgpack-c |
| `docker/colosseum-entrypoint.sh` | Xvfb + Blocks launch + wait for port 41451 + ROS 2 launch |
| `src/fiber_nav_colosseum/package.xml` | New ROS 2 package |
| `src/fiber_nav_colosseum/CMakeLists.txt` | C++23, FetchContent msgpack-c + doctest |
| `docker/docker-compose.yml` | Add `colosseum-sim` service with `profiles: [colosseum]` |

Docker recipe: `nvidia/cuda:12.6.3-devel-ubuntu24.04`, Mesa D3D12 (`libgl1-mesa-dri`), env: `GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, UE4 flags: `-opengl4 -RenderOffScreen -nosound -nosplash -novsync -ExecCmds="sg.ShadowQuality 0,...,t.MaxFPS 120"`

### Phase 2: ROS 2 Bridge Node (5-7 days)

| File | Description |
|------|-------------|
| `src/fiber_nav_colosseum/src/airsim_rpc_client.hpp/.cpp` | Raw TCP + msgpack-rpc client (port 41451) |
| `src/fiber_nav_colosseum/src/colosseum_bridge_node.hpp/.cpp` | AirSim API -> ROS 2 topics |
| `src/fiber_nav_colosseum/config/colosseum_settings.json` | AirSim settings (ComputerVision mode) |
| `src/fiber_nav_colosseum/tests/airsim_rpc_client_test.cpp` | doctest: msgpack pack/unpack |
| `src/fiber_nav_bringup/launch/colosseum_simulation.launch.py` | Backend launch file |
| `src/fiber_nav_bringup/launch/simulation.launch.py` | Add `'colosseum'` to choices |

Topic mapping: odometry (100Hz), IMU (200Hz), baro/mag (50Hz), GPS (10Hz), camera (30Hz), rangefinder (50Hz), clock (100Hz). All from AirSim API via C++ msgpack-rpc (not Python -- airsim pip broken on 3.12).

### Phase 3: PX4 Integration -- MC Mode (5-7 days)

| File | Description |
|------|-------------|
| `src/fiber_nav_colosseum/config/px4_multirotor_settings.json` | AirSim PX4Multirotor + lockstep |
| `docker/airframes/4253_colosseum_quadtailsitter_vision` | Fork of 4251, no SIM_GZ_* |
| `docker/colosseum-px4-entrypoint.sh` | 9-phase orchestrator |
| `docker/docker-compose.yml` | Add `colosseum-px4-sitl` service |

Uses AirSim's built-in PX4 MAVLink bridge (port 4560, lockstep, HIL sensors).

### Phase 4: VTOL Aerodynamics (7-10 days)

| File | Description |
|------|-------------|
| `src/fiber_nav_colosseum/src/vtol_dynamics/aero_coefficients.hpp` | From O3DE gem, plain C++ |
| `src/fiber_nav_colosseum/src/vtol_dynamics/aerodynamics_model.hpp/.cpp` | CL/CD from `AerodynamicsComponent` |
| `src/fiber_nav_colosseum/src/vtol_dynamics/motor_model.hpp/.cpp` | From `MulticopterMotorComponent` |
| `src/fiber_nav_colosseum/src/vtol_dynamics/vtol_physics_node.hpp/.cpp` | External physics at 200Hz |
| `src/fiber_nav_colosseum/config/vtol_aero_params.yaml` | Aero + motor coefficients |
| `src/fiber_nav_colosseum/tests/aerodynamics_model_test.cpp` | doctest: CL/CD, forces |
| `src/fiber_nav_colosseum/tests/motor_model_test.cpp` | doctest: thrust, torque |

Reuse from O3DE gems (`src/fiber_nav_o3de/gems/vtol_dynamics/Code/Source/`):
- `aerodynamics_component.cpp` -- AdvancedLiftDrag (~230 LOC)
- `multicopter_motor_component.cpp` -- motor forces
- `vtol_transition_component.cpp` -- state machine

### Phase 5: Terrain Parity (3-5 days)

Altitude offset for GPS/baro to match Negev 141m MSL. Real DEM terrain requires UE4 source build (deferred).

### Risks

| Risk | Mitigation |
|------|------------|
| `simSetVehiclePose()` latency at 200Hz | Fall back to source build + custom FastPhysicsEngine |
| PX4 lockstep vs external physics conflict | Use O3DE MAVLink bridge instead of AirSim's built-in |
| Blocks binary API drift from docs | Test all RPC methods in Phase 1 |

---

## Verification

**Part A**: `vulkaninfo --summary` shows NVIDIA GPU via DZN, `test_headless.sh` reports GO
**Part B Phase 1**: Container builds, glxinfo shows D3D12, port 41451 open, FHD camera works
**Part B Phase 2**: All 8 ROS 2 topics at correct rates, Foxglove shows camera
**Part B Phase 3**: PX4 arms, takes off MC, hovers, lands
**Part B Phase 4**: Full VTOL mission: MC->FW->navigate->return->MC->land
