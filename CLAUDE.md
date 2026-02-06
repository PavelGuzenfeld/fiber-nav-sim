# Fiber Navigation Simulation - Claude Code Instructions

## Docker & PX4 Rules

**CRITICAL: Long-running processes must be started manually**

1. **NEVER start PX4 or any long-running simulator process via `docker exec`**
   - PX4 requires a real TTY for proper operation
   - These must be started manually in a separate terminal

2. **ALL `docker exec` commands MUST use timeouts:**
   ```bash
   timeout <N> docker exec container bash -c "..."
   ```

3. **NEVER redirect PX4 output to log files**
   - PX4 generates infinite `pxh>` prompts that hang processes
   - Always run PX4 interactively in a terminal

4. **For integration testing, provide shell commands for the user to run manually**
   - Give clear step-by-step instructions
   - The user will run PX4/Gazebo in separate terminals

## Build & Test

- Build Docker: `docker compose -f docker/docker-compose.yml build simulation`
- Run tests: `docker compose run --rm simulation colcon test`
- Interactive shell: `docker compose run --rm simulation bash`

## Project Structure

```
fiber-nav-sim/
├── src/
│   ├── fiber_nav_fusion/     # Sensor fusion (spool + vision)
│   ├── fiber_nav_sensors/    # Sensor drivers/simulators + plane_controller
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
| `/sensors/fiber_spool/velocity` | std_msgs/Float64 | Scalar spool velocity |
| `/sensors/vision_direction` | geometry_msgs/Vector3Stamped | Unit direction vector |
| `/fmu/in/vehicle_visual_odometry` | px4_msgs/VehicleOdometry | Fusion output to PX4 |
| `/fmu/out/vehicle_attitude` | px4_msgs/VehicleAttitude | PX4 attitude for transforms |

## PX4 Integration

Using custom airframe `4251_gz_quadtailsitter_vision`:
- GPS-denied (`SYS_HAS_GPS=0`)
- External vision velocity fusion (`EKF2_EV_CTRL=4`)
- Gazebo physics lockstep (`SIM_GZ_EN=1`)

See `docs/PX4_GAZEBO_INTEGRATION_PLAN.md` for full details.

## Integration Testing (Manual)

Run in **separate terminals**:

**Terminal 1 - Gazebo:**
```bash
docker compose run --rm simulation bash
source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
ros2 launch fiber_nav_bringup simulation.launch.py
```

**Terminal 2 - Foxglove:**
```bash
docker compose up foxglove
```

**Terminal 3 - PX4 SITL:**
```bash
docker exec -it <container_id> bash
cd /root/PX4-Autopilot/build/px4_sitl_default/rootfs
rm -f dataman parameters*.bson
PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4
```

**Terminal 4 - DDS Agent + Fusion:**
```bash
docker exec -it <container_id> bash
MicroXRCEAgent udp4 -p 8888 &
sleep 2
source /root/ws/install/setup.bash
ros2 run fiber_nav_fusion fiber_vision_fusion
```

**Terminal 5 - Apply Thrust (standalone mode only):**
```bash
docker exec -it <container_id> bash
source /root/ws/install/setup.bash
ros2 run fiber_nav_sensors plane_controller --ros-args -p thrust:=15.0 -p lift:=10.0 -p model_name:=quadtailsitter
```

**Foxglove:** Open https://studio.foxglove.dev → Connect → `ws://localhost:8765`

**Success:** `cs_ev_vel: true` in `/fmu/out/estimator_status_flags`
