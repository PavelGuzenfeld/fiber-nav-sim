# Foxglove Visualization Demo

Real-time visualization of the fiber optic navigation simulation using Foxglove Studio.

## Quick Start

### 1. Start the Simulation with Foxglove Bridge

```bash
cd ~/workspace/fiber-nav-sim
docker compose -f docker/docker-compose.yml run --rm -p 8765:8765 simulation bash
```

Inside the container:
```bash
# Install Foxglove bridge
apt-get update && apt-get install -y ros-jazzy-foxglove-bridge

# Source environment
source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash

# Start Foxglove bridge in background
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
sleep 3

# Start simulation
ros2 launch fiber_nav_bringup simulation.launch.py auto_fly:=true thrust:=20.0 lift:=30.0 headless:=false spawn_z:=50.0
```

### 2. Connect Foxglove Studio

1. Open [Foxglove Studio](https://studio.foxglove.dev/) in your browser
2. Click **"Open connection"**
3. Select **"Foxglove WebSocket"**
4. Enter URL: `ws://localhost:8765`
5. Click **"Open"**

### 3. Load the Layout

1. In Foxglove Studio, click **"Layout"** (top right)
2. Click **"Import from file"**
3. Select `foxglove/fiber_nav_layout.json`

---

## Available Topics

### Sensor Data (Working)
| Topic | Type | Description |
|-------|------|-------------|
| `/sensors/fiber_spool/velocity` | Float32 | Spool velocity (m/s) |
| `/sensors/vision_direction` | Vector3Stamped | Direction unit vector |
| `/model/plane/odometry` | Odometry | Ground truth position/velocity |
| `/fmu/in/vehicle_visual_odometry` | VehicleOdometry | Fusion output to PX4 |
| `/fmu/out/vehicle_attitude` | VehicleAttitude | Vehicle orientation |

### Camera Feeds (Requires Real-Time Physics)
| Topic | Description | Notes |
|-------|-------------|-------|
| `/world/.../camera/image` | Forward camera | Only works at 1x speed |
| `/world/.../camera_down/image` | Downward camera | Only works at 1x speed |
| `/world/.../follow_cam/image` | 3rd person follow | Only works at 1x speed |

**Note:** Camera images only render when simulation runs at real-time (1x speed).
Use the Gazebo GUI playback controls to slow down the simulation.

---

## Layout Description

The `fiber_nav_layout.json` provides proof-of-concept visualization:

```
┌───────────────────────┬───────────────────────┬───────────────────────┐
│  Spool vs Ground      │  Vision Direction     │  Fusion Output        │
│  Truth Velocity       │  Vector               │  (Raw Messages)       │
│  (proves spool works) │  (x≈1 = forward)      │  /fmu/in/vehicle_     │
│                       │                       │  visual_odometry      │
├───────────────────────┼───────────────────────┼───────────────────────┤
│  Flight Trajectory    │  3D Velocity          │  Ground Truth Odom    │
│  (X position +        │  Components           │  (Raw Messages)       │
│  Altitude)            │  (Vx, Vy, Vz)         │  /model/plane/        │
│                       │                       │  odometry             │
└───────────────────────┴───────────────────────┴───────────────────────┘
```

### Key Proofs

1. **Spool vs Ground Truth**: Shows fiber spool velocity tracks actual forward speed
2. **Direction Vector**: Shows x≈1 (forward), y≈0, z≈small during forward flight
3. **Fusion Output**: Shows `velocity = spool_velocity × direction_vector`

---

## How to Interpret the Telemetry Graphs

### Spool Velocity vs Ground Truth

**What you see:**
- **Red line**: Fiber spool velocity (measured from cable payout)
- **Green line**: Ground truth Vx from Gazebo (actual forward speed)

**What to look for:**
- Lines should track closely together
- Small differences are expected due to:
  - Spool measurement noise (±0.1 m/s configured)
  - Slack factor compensation (1.05x)
- Large divergence indicates sensor failure or model error

**Interpretation:**
- If spool > ground truth: Cable has slack or sensor noise
- If spool < ground truth: Cable tension or missed pulses
- Correlation proves the fiber spool accurately measures forward velocity

### Vision Direction Vector

**What you see:**
- **X component** (red): Forward direction (should be ~1.0)
- **Y component** (green): Lateral direction (should be ~0.0)
- **Z component** (blue): Vertical direction (small positive = climbing)

**What to look for:**
- During straight flight: X ≈ 1.0, Y ≈ 0.0, Z ≈ 0.0
- During turns: Y deviates from 0 (positive = turning right)
- During climb: Z increases (positive = climbing)
- Vector magnitude should always be ~1.0 (unit vector)

**Interpretation:**
- Stable X near 1.0 = Vision correctly tracking forward direction
- Drift in values over time = Vision drift (expected, corrected by landmarks)
- Sudden jumps = Feature tracking loss or scene change

### Flight Trajectory

**What you see:**
- **X Position** (red): Distance traveled forward (meters)
- **Altitude** (blue): Height above ground (meters)

**What to look for:**
- X should increase steadily (plane flying forward)
- Altitude should stabilize (lift balances weight)
- Smooth curves indicate stable flight

**Interpretation:**
- Linear X growth = Constant forward velocity
- Altitude oscillation = Pitch instability or turbulence
- X reaching thousands of meters = Simulation running faster than real-time

### 3D Velocity Components

**What you see:**
- **Vx** (red): Forward velocity (m/s)
- **Vy** (green): Lateral velocity (m/s)
- **Vz** (blue): Vertical velocity (m/s)

**What to look for:**
- Vx should be dominant (forward flight)
- Vy should be near zero (no sideslip)
- Vz should be small (level flight) or positive (climbing)

**Interpretation:**
- High Vx with low Vy/Vz = Clean forward flight
- Growing Vz = Plane climbing (lift > weight)
- Negative Vz = Plane descending
- Non-zero Vy = Crosswind or turning

### Fusion Output (Raw Messages)

**What you see:**
- `velocity[0]`: Fused Vx (body frame)
- `velocity[1]`: Fused Vy (body frame)
- `velocity[2]`: Fused Vz (body frame)
- `velocity_variance`: Uncertainty estimates

**What to look for:**
- Velocity values should match ground truth pattern
- Variance should be reasonable (not zero, not huge)
- Timestamp should be current (not stale)

**Interpretation:**
- `velocity = spool_velocity × direction_vector`
- If velocity[0] ≈ spool_velocity and direction.x ≈ 1.0, fusion is correct
- High variance = Low confidence (poor vision or sensor data)

### Success Criteria

The fiber optic navigation model is **proven working** when:

1. ✅ Spool velocity tracks ground truth Vx (±10%)
2. ✅ Direction vector X stays near 1.0 during forward flight
3. ✅ Direction vector is normalized (magnitude ≈ 1.0)
4. ✅ Fusion output velocity matches spool × direction
5. ✅ Position increases steadily (integration working)

---

## Panel Configuration Tips

### Plot Panel (Velocity Comparison)
Add these paths:
- `/sensors/fiber_spool/velocity.data` - Spool measurement
- `/model/plane/odometry.twist.twist.linear.x` - Ground truth Vx

### Plot Panel (Direction Vector)
Add these paths:
- `/sensors/vision_direction.vector.x` - Should be ~1.0 (forward)
- `/sensors/vision_direction.vector.y` - Should be ~0.0
- `/sensors/vision_direction.vector.z` - Small values (pitch)

### Raw Messages Panel
Subscribe to:
- `/fmu/in/vehicle_visual_odometry` - See fusion velocity output
- `/model/plane/odometry` - See ground truth

---

## Troubleshooting

### No data showing
- Ensure simulation is **unpaused** (click Play ▶️ in Gazebo)
- Verify Foxglove bridge is running: `ros2 node list | grep foxglove`
- Check topics exist: `ros2 topic list | grep sensors`

### Connection refused
- Ensure port 8765 is exposed: `-p 8765:8765`
- Check bridge is running: `ps aux | grep foxglove`

### Stale data
- Simulation might be paused - click Play in Gazebo
- Reconnect Foxglove: disconnect and reconnect to `ws://localhost:8765`

### Camera images not showing
- Cameras only work when simulation runs at ~1x real-time
- Use Gazebo GUI playback controls to slow down
- If Gazebo GUI not visible, cameras won't render

---

## The Fiber Optic Navigation Model

The simulation demonstrates GPS-denied navigation using:

1. **Fiber Spool Sensor**: Measures scalar velocity from cable payout rate
2. **Monocular Vision**: Estimates flight direction as unit vector
3. **Sensor Fusion**: `v_body = (spool_velocity / slack_factor) × direction_vector`

This fused velocity is sent to PX4's EKF2 as external vision velocity, enabling
position estimation without GPS.
