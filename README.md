# Fiber Navigation Simulation

ROS 2 / Gazebo simulation for GPS-denied fixed-wing navigation using "Cable Odometry" and monocular vision.

## Overview

Testing a novel navigation algorithm for a fixed-wing drone in a GPS-denied environment (e.g., a long tunnel or canyon). The drone is tethered by a fiber optic cable. We fuse two non-standard data sources to estimate position/velocity:

- **Cable Spool Sensor**: Measures the scalar payout rate of the fiber (providing absolute velocity magnitude)
- **Monocular Camera**: Provides the direction of motion (relative unit vector) but no scale

## Technical Stack

| Component | Technology |
|-----------|------------|
| Simulator | Gazebo Classic / Gazebo Garden |
| Flight Stack | PX4 Autopilot (SITL) |
| Middleware | ROS 2 (Humble/Jazzy) |
| Language | C++ (performance nodes) |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Gazebo Simulation                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ Fixed-Wing  │  │   Canyon    │  │    Ground Truth         │  │
│  │   Model     │  │   World     │  │  /gazebo/model_states   │  │
│  └─────────────┘  └─────────────┘  └───────────┬─────────────┘  │
└───────────────────────────────────────────────┼─────────────────┘
                                                │
                    ┌───────────────────────────┼───────────────────────────┐
                    │                           │                           │
                    ▼                           ▼                           ▼
        ┌───────────────────┐       ┌───────────────────┐       ┌───────────────────┐
        │  spool_sim_driver │       │ vision_direction  │       │      PX4 SITL     │
        │                   │       │      _sim         │       │                   │
        │  GT → |v| + noise │       │  GT → û + drift   │       │   IMU, Attitude   │
        └─────────┬─────────┘       └─────────┬─────────┘       └─────────┬─────────┘
                  │                           │                           │
                  │ /sensors/fiber_spool      │ /sensors/vision_dir       │ /fmu/out/*
                  │     /velocity             │                           │
                  └───────────┬───────────────┴───────────────────────────┘
                              │
                              ▼
                  ┌───────────────────────────┐
                  │   fiber_vision_fusion     │
                  │                           │
                  │  v_est = (S/slack) × û    │
                  │  Rotate Body → NED        │
                  └─────────────┬─────────────┘
                                │
                                ▼
                  ┌───────────────────────────┐
                  │  /fmu/in/vehicle_visual   │
                  │       _odometry           │
                  └───────────────────────────┘
```

## Packages

| Package | Description |
|---------|-------------|
| `fiber_nav_bringup` | Launch files and configuration |
| `fiber_nav_gazebo` | Gazebo worlds (canyon/tunnel) and vehicle models |
| `fiber_nav_sensors` | Synthetic sensor nodes (spool, vision) |
| `fiber_nav_fusion` | Core fusion algorithm |
| `fiber_nav_analysis` | Python plotting and metrics |

## Phase 1: Environment & Vehicle

### World Generation
- "Infinite Canyon" or "Long Tunnel" world in Gazebo
- Texture-rich walls (rock/concrete) for optical flow features
- Minimum 2km long with gentle curves for heading drift testing

### Vehicle
- Standard fixed-wing airframe (plane or standard_vtol)
- Forward-facing camera link added to URDF/SDF

## Phase 2: Synthetic Spool Sensor

`spool_sim_driver` node - Digital twin of the physical spool sensor:

**Input**: Ground truth velocity from Gazebo

**Processing**:
```
S = ||v_true||                    # Velocity magnitude
S_noisy = S + N(0, 0.1)           # Gaussian noise σ=0.1 m/s
S_measured = S_noisy × 1.05       # Slack bias (over-payout)
```

**Output**: `Float32` on `/sensors/fiber_spool/velocity`

## Phase 3: Synthetic Vision Sensor

`vision_direction_sim` node - Simulates perfect CV pipeline:

**Input**: Ground truth velocity from Gazebo

**Processing**:
```
û = v_true / ||v_true||           # Normalize to unit vector
û_drifted = apply_random_walk(û)  # Slow-varying drift
```

**Output**: `Vector3` on `/sensors/vision_direction` (body frame)

## Phase 4: Fusion Node

`fiber_vision_fusion` node - Replaces GPS:

**Inputs**:
- Spool magnitude from `/sensors/fiber_spool/velocity`
- Vision direction from `/sensors/vision_direction`
- IMU data from PX4 (via MAVROS/micro-XRCE-DDS)

**Processing**:
```
v_estimated = (S_measured / slack_factor) × û_vision   # Body frame
v_ned = R_body_to_ned × v_estimated                    # Rotate to NED
```

**Output**: `/fmu/in/vehicle_visual_odometry` (velocity fields only, position = NaN)

## Phase 5: Success Criteria

### Metrics
- Ground Truth Trajectory (from Gazebo)
- Estimated Trajectory (from PX4 EKF)
- **Drift per 1000m**

### Baseline vs Test
| Scenario | Expected Result |
|----------|----------------|
| IMU only (GPS disabled) | Massive drift |
| Spool + Vision Fusion | Constrained drift |

## Build

```bash
cd ~/workspace/fiber-nav-sim
colcon build --symlink-install
source install/setup.bash
```

## Usage

```bash
# Terminal 1: Launch Gazebo world + PX4 SITL
ros2 launch fiber_nav_bringup simulation.launch.py

# Terminal 2: Launch sensor simulators + fusion
ros2 launch fiber_nav_bringup fusion.launch.py

# Terminal 3: Run analysis
ros2 run fiber_nav_analysis plot_trajectory.py
```

## Why This Design Works

1. **Decoupling**: Separates rope physics (hard) from rope signal (easy) - test navigation logic immediately
2. **Noise Control**: Manual bias injection (1.05 factor) stress-tests EKF slack estimation
3. **Fixed-Wing Specifics**: Canyon forces banking/turning - hardest test for body-to-NED rotation math

## License

MIT
