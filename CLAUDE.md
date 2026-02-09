# Fiber Navigation Simulation - Claude Code Instructions

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
│   ├── fiber_nav_gazebo/     # Gazebo models (quadtailsitter) and worlds
│   ├── fiber_nav_bringup/    # Launch files
│   ├── fiber_nav_mode/       # PX4 custom flight modes (px4-ros2-interface-lib)
│   └── fiber_nav_analysis/   # Python analysis tools
├── docker/                    # Docker configuration
│   └── airframes/            # PX4 custom airframes (4251)
├── foxglove/                  # Foxglove Studio layout
├── scripts/                   # Analysis and test scripts
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
- `scripts/record_test_flight.py` — records ground truth vs EKF vs fusion to CSV (note: GT velocity is body-frame, EKF velocity is NED; use speed magnitude for comparison)

### Required services (start in order):
1. Gazebo + ros_gz_bridge (via simulation.launch.py)
2. MicroXRCEAgent (DDS bridge, UDP port 8888)
3. sim_distance_sensor.py
4. PX4 SITL (output to /dev/null)
5. offboard_takeoff.py or offboard_mission.py

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
