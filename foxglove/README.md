# Foxglove Visualization

Real-time visualization of the fiber optic navigation simulation using Foxglove Studio.

## Quick Start

### 1. Start the Simulation

```bash
cd ~/workspace/fiber-nav-sim/docker

# Option A: Standalone (headless, auto-fly)
docker compose up standalone -d

# Option B: With GUI
docker compose up simulation
```

### 2. Start Foxglove Bridge

```bash
# Using the dedicated service
docker compose up foxglove -d

# Or inside the simulation container
docker exec -it <container> bash
apt-get update && apt-get install -y ros-jazzy-foxglove-bridge
source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### 3. Connect Foxglove Studio

1. Open [Foxglove Studio](https://studio.foxglove.dev/) in your browser
2. Click **"Open connection"**
3. Select **"Foxglove WebSocket"**
4. Enter URL: `ws://localhost:8765`
5. Click **"Open"**

### 4. Load the Layout

1. In Foxglove Studio, click **"Layout"** (top right)
2. Click **"Import from file"**
3. Select `foxglove/fiber_nav_layout.json`

---

## Available Topics

### Sensor Data
| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/fiber_spool/velocity` | Float32 | Spool velocity (m/s) |
| `/sensors/vision_direction` | Vector3Stamped | Direction unit vector |
| `/model/quadtailsitter/odometry` | Odometry | Ground truth position/velocity |
| `/fmu/in/vehicle_visual_odometry` | VehicleOdometry | Fusion output to PX4 |
| `/fmu/out/vehicle_attitude` | VehicleAttitude | Vehicle orientation |

### Camera Feeds
| Topic | Description |
|-------|-------------|
| `/camera` | Forward camera (640x480, 30 Hz) |
| `/camera_down` | Downward camera (640x480, 30 Hz) |
| `/follow_camera` | 3rd person follow camera (1280x720, 30 Hz) |

Camera images render when simulation runs at real-time (~1x speed). In headless mode, cameras use EGL rendering.

---

## Layout Description

The `fiber_nav_layout.json` provides a comprehensive visualization:

```
+---------------------------+---------------------------+
|  Spool vs Ground Truth    |  Vision Direction         |
|  Velocity (Plot)          |  Vector (Plot)            |
+---------------------------+---------------------------+
|  Flight Trajectory        |  3D Velocity              |
|  Position + Altitude      |  Components (Vx,Vy,Vz)   |
+---------------------------+---------------------------+---+
                            |  Follow Camera (3rd person)   |
                            +-------------------------------+
                            |  Forward Cam | Down Cam       |
                            +--------------+----------------+
                            |  Fusion Raw  | Odometry Raw   |
                            +--------------+----------------+
```

### Panels

- **Spool vs GT**: Fiber spool velocity overlaid with ground truth Vx
- **Direction Vector**: Vision direction components (x=forward, y=lateral, z=vertical)
- **Position**: X position and altitude over time
- **Velocity**: Ground truth Vx, Vy, Vz components
- **Follow Camera**: 3rd person chase view (attached to vehicle, 5m behind, 2m above)
- **Forward Camera**: Nose-mounted forward view
- **Down Camera**: Downward-facing ground tracking view
- **Fusion Output**: Raw `/fmu/in/vehicle_visual_odometry` messages
- **Odometry**: Raw `/model/quadtailsitter/odometry` messages

---

## How to Interpret Telemetry

### Spool Velocity vs Ground Truth
- **Spool line**: Fiber spool velocity (measured from cable payout)
- **GT line**: Ground truth Vx from Gazebo
- Lines should track closely; small differences from noise (0.1 m/s) and slack (1.05x)

### Vision Direction Vector
- **X** ~ 1.0 during forward flight
- **Y** ~ 0.0 (deviates during turns)
- **Z** ~ 0.0 (positive = climbing)
- Magnitude always ~ 1.0 (unit vector)

### Success Criteria
1. Spool velocity tracks ground truth Vx (within 10%)
2. Direction vector X stays near 1.0 during forward flight
3. Fusion output velocity matches spool x direction
4. Position increases steadily

---

## Troubleshooting

### No data showing
- Ensure simulation is **unpaused** (click Play in Gazebo)
- Verify bridge is running: `ros2 node list | grep foxglove`
- Check topics: `ros2 topic list | grep sensors`

### Connection refused
- Ensure port 8765 is accessible (network_mode: host in docker-compose)
- Check bridge process: `pgrep -f foxglove_bridge`

### Camera images not showing
- Cameras need EGL rendering in headless mode (`--headless-rendering` flag)
- Verify cameras publish: `ros2 topic hz /camera`
- In GUI mode, simulation must run at ~1x real-time
