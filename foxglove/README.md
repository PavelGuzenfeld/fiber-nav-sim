# Foxglove Visualization Demo

Real-time visualization of the fiber optic navigation simulation using Foxglove Studio.

## Quick Start

### 1. Start the Simulation

**Terminal 1 - Run Gazebo with GUI:**
```bash
cd fiber-nav-sim
docker compose -f docker/docker-compose.yml run --rm -e DISPLAY=$DISPLAY simulation bash

# Inside container:
source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
ros2 launch fiber_nav_bringup simulation.launch.py auto_fly:=true thrust:=15.0 lift:=10.0
```

### 2. Start Foxglove Bridge

**Terminal 2:**
```bash
cd fiber-nav-sim
docker compose -f docker/docker-compose.yml up foxglove
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

### Camera Feeds
| Topic | Description | Resolution |
|-------|-------------|------------|
| `/world/.../camera/image` | Forward camera | 640x480 |
| `/world/.../camera_down/image` | Downward camera | 640x480 |
| `/world/.../follow_cam/image` | 3rd person follow | 1280x720 |

### Sensor Data
| Topic | Type | Description |
|-------|------|-------------|
| `/model/plane/odometry` | Odometry | Ground truth position/velocity |
| `/sensors/fiber_spool/velocity` | Float64 | Spool velocity (m/s) |
| `/sensors/vision_direction` | Vector3Stamped | Direction unit vector |
| `/fmu/in/vehicle_visual_odometry` | VehicleOdometry | Fusion output to PX4 |

### PX4 Status (if running)
| Topic | Description |
|-------|-------------|
| `/fmu/out/estimator_status_flags` | EKF status (check `cs_ev_vel`) |
| `/fmu/out/vehicle_attitude` | Vehicle orientation |

---

## Recommended Panel Setup

```
┌──────────────────────────────────┬─────────────────────────┐
│                                  │     Follow Camera       │
│         3D View                  ├─────────────────────────┤
│     (Plane Trajectory)           │    Forward Camera       │
├──────────────────────────────────┼─────────────────────────┤
│    Velocity Plot                 │    Downward Camera      │
│  (Spool vs Ground Truth)         ├─────────────────────────┤
├──────────────────────────────────│    Fusion Output        │
│    Position Plot                 │    (Raw Messages)       │
│  (X position, Altitude)          │                         │
└──────────────────────────────────┴─────────────────────────┘
```

---

## Panel Configuration Tips

### 3D Panel
- Set "Follow TF" to `base_link` for camera tracking
- Enable `/model/plane/odometry` topic for trajectory
- Use rainbow color mode for time-based coloring

### Plot Panel (Velocity)
Add these paths:
- `/sensors/fiber_spool/velocity.data` - Spool measurement
- `/model/plane/odometry.twist.twist.linear.x` - Ground truth

### Image Panels
Subscribe to the full topic paths:
- `/world/canyon_world/model/plane/link/base_link/sensor/camera/image`
- `/world/canyon_world/model/plane/link/base_link/sensor/camera_down/image`
- `/world/canyon_world/model/follow_camera/link/link/sensor/follow_cam/image`

---

## Troubleshooting

### No images showing
- Ensure Gazebo is running with GUI (`headless:=false`)
- Check that ros_gz_bridge is running
- Verify topics exist: `ros2 topic list | grep image`

### Connection refused
- Ensure Foxglove bridge container is running
- Check port 8765 is not blocked
- Try: `docker compose logs foxglove`

### Slow performance
- Reduce image resolution in model.sdf
- Lower camera update rate
- Use fewer plot history points
