# Fiber Navigation Simulation - Claude Code Instructions

## Naming Conventions

**All directories and files MUST use snake_case.**

- Directories: `fiber_nav_sensors/`, `fiber_nav_gazebo/`, `docker/airframes/`
- Source files: `vtol_mission_executor.hpp`, `spool_velocity_sensor.cpp`
- Launch files: `gazebo_simulation.launch.py`, `px4_sitl.launch.py`
- Scripts: `offboard_mission.py`, `sim_distance_sensor.py`
- Config files: `canyon_mission.yaml`, `sensor_params.yaml`

**Exceptions** (external conventions that override snake_case):
- O3DE Gem directories: `Code/`, `Include/`, `Source/`, `Tests/` (O3DE standard)
- Docker files: `Dockerfile`, `Dockerfile.o3de` (Docker convention)
- ROS 2 files: `CMakeLists.txt`, `package.xml` (ROS/CMake convention)
- Markdown: `README.md`, `CLAUDE.md`, `LICENSE`

## Docker & PX4 Rules

**CRITICAL: Docker and PX4 rules**

1. **ALL `docker exec` commands MUST use timeouts:**
   ```bash
   timeout <N> docker exec container bash -c "..."
   ```

2. **NEVER redirect PX4 output to log files**
   - PX4 generates infinite `pxh>` prompts → multi-GB log files
   - Always use `> /dev/null 2>&1` for detached PX4

3. **PX4 can be started detached** (output to /dev/null):
   ```bash
   docker exec -d container bash -c "... ../bin/px4 > /dev/null 2>&1"
   ```

4. **Airframe is COPY'd into Docker image at build time**
   - Host changes via volume mount don't affect the installed airframe
   - After modifying, copy into container:
   ```bash
   docker exec container cp /root/ws/src/fiber-nav-sim/docker/airframes/4251_gz_quadtailsitter_vision \
     /root/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
   ```

5. **TrajectorySetpoint NaN requirement**
   - ALL unused fields must be set to `float('nan')` explicitly
   - Default `[0,0,0]` causes hard altitude ceilings and control bugs

## C++ Testing

**All C++ tests MUST use doctest, NOT gtest.**

- Use `#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` + `#include <doctest/doctest.h>`
- Use `TEST_CASE("Name")`, `CHECK()`, `REQUIRE()`, `doctest::Approx()`
- CMakeLists.txt: FetchContent doctest v2.4.11, register with `add_test()` (not `ament_add_gtest`)
- See `fiber_nav_sensors/CMakeLists.txt` for reference pattern

## Build & Test

- Build Docker: `docker compose -f docker/docker-compose.yml build simulation`
- Run tests: `docker compose run --rm simulation colcon test`
- Interactive shell: `docker compose run --rm simulation bash`

## Project Structure

```
fiber-nav-sim/
├── src/
│   ├── fiber_nav_fusion/     # Sensor fusion (spool + vision)
│   ├── fiber_nav_sensors/    # Sensor drivers/simulators + flight controller
│   ├── fiber_nav_gazebo/     # Gazebo worlds, models, terrain data
│   │   ├── worlds/           # terrain_world.sdf, canyon_harmonic.sdf
│   │   ├── models/           # quadtailsitter (model.sdf, model_px4.sdf)
│   │   └── terrain/          # heightmap, texture, terrain_data.json
│   ├── fiber_nav_bringup/    # Launch files (dispatcher + backends)
│   ├── fiber_nav_mode/       # PX4 custom flight modes (px4-ros2-interface-lib)
│   ├── fiber_nav_o3de/       # O3DE gems (px4_bridge, vtol_dynamics) — scaffolds
│   └── fiber_nav_analysis/   # Python analysis tools
├── docker/                    # Docker configuration
│   ├── airframes/            # PX4 custom airframes (4251)
│   ├── px4-sitl-entrypoint.sh  # 7-phase PX4 SITL orchestrator (phase 6 = optional mission auto-launch)
│   └── Dockerfile.o3de       # O3DE + Mesa DZN Vulkan
├── foxglove/                  # Foxglove Studio layout
├── scripts/                   # Terrain gen, flight scripts, tests
└── docs/                      # Documentation
```

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/model/quadtailsitter/odometry` | nav_msgs/Odometry | Ground truth from Gazebo |
| `/sensors/fiber_spool/velocity` | std_msgs/Float32 | Scalar spool velocity |
| `/sensors/fiber_spool/status` | fiber_nav_sensors/SpoolStatus | Velocity + total length + is_moving |
| `/sensors/vision_direction` | geometry_msgs/Vector3Stamped | Unit direction vector |
| `/fmu/in/vehicle_visual_odometry` | px4_msgs/VehicleOdometry | Fusion output to PX4 |
| `/fmu/out/vehicle_attitude` | px4_msgs/VehicleAttitude | PX4 attitude for transforms |
| `/fmu/out/vehicle_status_v1` | px4_msgs/VehicleStatus | Flight phase (MC/FW/transition) |
| `/sensors/fusion/diagnostics` | std_msgs/String | 1 Hz JSON: flight_phase, health, variance, health_scale, staleness, xval, heading_check, slack, zupt_threshold |
| `/fmu/out/vehicle_local_position` | px4_msgs/VehicleLocalPosition | EKF velocity for cross-validation + slack calibration |
| `/terrain/query` | geometry_msgs/Point | GIS terrain height query (used by look-ahead) |
| `/terrain/height` | std_msgs/Float64 | GIS terrain height response |
| `/vehicle/nav_sat_fix` | sensor_msgs/NavSatFix | Vehicle position for Foxglove Map panel |
| `/map/terrain_overlay` | foxglove_msgs/GeoJSON | Terrain elevation heatmap overlay |
| `/vehicle/terrain_agl` | std_msgs/Float64 | Terrain height AGL under vehicle |

## PX4 Integration

Using custom airframe `4251_gz_quadtailsitter_vision`:
- GPS for home position + global origin (`SYS_HAS_GPS=1`, `EKF2_GPS_CTRL=7`)
- Vision velocity + position fusion (`EKF2_EV_CTRL=5`)
- Range finder for terrain height (`EKF2_RNG_CTRL=1`, via `sim_distance_sensor.py`)
- Land detector tuning (`LNDMC_ALT_MAX=1.0` to prevent ground_contact deadlock)
- Airspeed disabled for SITL (`CBRK_AIRSPD_CHK=162128`, `FW_ARSP_MODE=1`)
- Gazebo physics lockstep (`SIM_GZ_EN=1`)
- gz_bridge compiled with `GZ_DISTRO=harmonic` (uses ROS Jazzy vendor packages)
- Build target `make px4_sitl_default` (NOT `gz_x500` which hangs)
- Runtime model selected via `PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter`

### Offboard Flight (Verified)
- `scripts/offboard_takeoff.py` — arm, climb to target altitude, hold 30s, land
- `scripts/offboard_mission.py` — MC waypoint mission (back-and-forth pattern)
- `scripts/offboard_mission.py --vtol` — VTOL FW mission (MC takeoff → FW transition → 4 waypoints at 18-23 m/s → MC back-transition → RTL → land)
- `scripts/offboard_transition_test.py` — standalone MC→FW→MC transition test (30m alt, 30s FW cruise east)
- `scripts/sim_distance_sensor.py` — publishes distance sensor from Gazebo ground truth
- `scripts/offboard_terrain_follow.py` — Python reference implementation for terrain-following flight
- `scripts/record_test_flight.py` — records ground truth vs EKF vs fusion to CSV (note: GT velocity is body-frame, EKF velocity is NED; use speed magnitude for comparison)

### Required services (start in order):
1. Gazebo + ros_gz_bridge (via simulation.launch.py)
2. MicroXRCEAgent (DDS bridge, UDP port 8888)
3. sim_distance_sensor.py (terrain-aware AGL)
4. terrain_gis_node.py (terrain height queries)
5. PX4 SITL (output to /dev/null)
6. map_bridge_node.py (NavSatFix + terrain GeoJSON for Foxglove Map)
7. Mission auto-launch (optional, via MISSION env var)

The `px4-sitl` docker-compose service automates all 8 phases via `px4-sitl-entrypoint.sh`.
Set `MISSION=vtol_terrain` or `MISSION=vtol_canyon` to auto-launch the C++ VTOL mission node.

### Default world: terrain_world
- Real terrain from SRTM DEM (Negev desert, 31.16°N 34.53°E, 6km x 6km)
- Satellite texture for optical flow (Bing Maps)
- Elevation: 141-194m MSL (53m range), spawns at map center
- Set via `WORLD=terrain_world` / `WORLD_NAME=terrain_world` in docker-compose

See `docs/PX4_GAZEBO_INTEGRATION_PLAN.md` for full details.

## GPU Support

- NVIDIA GPU (RTX 3060) available via nvidia-container-toolkit
- docker-compose.yml `px4-sitl` and `simulation` services have GPU enabled
- `LIBGL_ALWAYS_SOFTWARE=0` with `NVIDIA_VISIBLE_DEVICES=all`
- Without GPU, Gazebo software rendering pegs CPU at 400-600% and freezes container

## Integration Testing (Manual)

Run in **3 separate terminals** using the `px4-sitl` service (has GPU support):

**Terminal 1 - Gazebo + Sensors + Foxglove:**
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

**Foxglove:** Open https://studio.foxglove.dev → Connect → `ws://localhost:8765`

**Standalone mode:** The stabilized flight controller runs automatically when `auto_fly:=true`.
No separate thrust/controller step needed.

**Success criteria:**
- `cs_ev_vel: true` in PX4 `listener estimator_status_flags`
- `cs_ev_pos: true` (position fusion from drag bow model)
- `home_position_valid: true`
- Vehicle can arm and take off
