# Plan: Position EKF + GPS-Denied Navigation (Level 3)

## Context

The current GPS-denied mission uses fixed headings and time-based waypoint acceptance — no position feedback after GPS disable. The drone flies a straight line out and the same heading back, landing wherever time runs out. This prevents complex missions and reliable return-to-home.

With spool encoder (scalar speed) + optical flow (direction) + IMU (attitude), we can dead-reckon a 2D position estimate. Cable deployed length provides a hard upper bound on distance from home. GPS is available at takeoff for baro calibration and home position.

## Sensor Reality (No Cheats)

### Available at takeoff (GPS)
- Home position fix (lat/lon/alt) → NED origin
- Barometer calibrated to GPS altitude
- Magnetic heading calibrated
- EKF initialized with good state

### Available during GPS-denied flight

| Sensor | Provides | Reliable? | Drift |
|--------|----------|-----------|-------|
| Spool encoder | Scalar ground speed (m/s) | Yes — physical | Encoder noise ~0.1 m/s σ |
| Spool integrated length | Total cable deployed (m) | Moderate | Accumulates encoder noise |
| Optical flow (camera) | Body-frame direction (unit vec) | Yes when quality > 0.3 | Random walk ~0.001 rad/s |
| IMU gyro | Attitude, angular rates | Yes for attitude | ~0.5-2° over 10 min |
| IMU accel | Specific force | Short-term only | Integrates to drift |
| Barometer | Relative altitude | Good first 2-5 min | Drifts 30-80m over 10+ min |
| Cable tension | Force magnitude (N) | Yes | N/A |
| Cable deployed length | Upper bound on ||pos|| | Yes | Spool integration drift |

### Eliminated (don't use)
- **sim_distance_sensor.py** — reads Gazebo ground truth XY to index heightmap = GPS cheat
- **Cable tension as home bearing** — cable tangles on terrain, drapes on rocks; tension direction is NOT toward home
- **Height-scaled OF velocity** — no reliable height source after baro drifts; OF gives direction only

## Architecture

```
GPS Fix (takeoff only)
    │
    ▼
┌─────────────────────────────────────────┐
│            Position EKF                  │
│                                          │
│  State: [x, y, vx, vy, wind_x, wind_y] │
│  (6-state, 2D horizontal only)          │
│                                          │
│  Prediction:                             │
│    attitude = IMU (gyro-integrated)      │
│    v_body = spool_speed * OF_dir         │
│    v_NED = R(attitude) * v_body          │
│    v_ground = v_NED - wind               │
│    pos += v_ground * dt                  │
│                                          │
│  Measurement updates:                    │
│    1. OF quality → adaptive R matrix     │
│    2. ||pos|| <= cable_length (ineq.)    │
│    3. Speed consistency (spool vs |v|)   │
│                                          │
│  Altitude: separate baro P-controller   │
│    (GPS-calibrated at start, accept      │
│     gradual drift, time-based descent)   │
└──────────────────┬───────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────┐
│         Mission Navigator                │
│                                          │
│  Outbound: steer toward WPs using pos   │
│  Return: steer toward (0,0) using pos   │
│  Descent: time-based (accept baro drift)│
│  Landing: time-based + baro relative    │
└─────────────────────────────────────────┘
```

## Core Algorithm: Dead Reckoning

```
velocity_ground = spool_speed * body_to_NED(OF_direction)
position += velocity_ground * dt
```

- Spool gives scalar magnitude (physical, no drift)
- OF gives body-frame direction (camera-based, ~0.001 rad/s drift)
- IMU gives body-to-NED rotation matrix (gyro-integrated attitude)
- Wind estimation: body-frame OF velocity vs ground-track divergence in FW flight

### Error Budget

| Source | Effect | Over 10 min flight |
|--------|--------|-------------------|
| OF direction drift | ~0.001 rad/s random walk | ~0.55° heading error |
| Spool speed noise | ~0.1 m/s σ | ~few meters position |
| Attitude (IMU gyro) drift | ~0.5-2° | Affects body-to-NED transform |
| Wind (unmodeled) | Body vs ground divergence | Addressed by wind state |
| **Combined DR drift** | **~2-5% of distance traveled** | **40-100m at 2km range** |

## Cable Constraint

```
||position|| <= cable_deployed_length
```

- Always valid, even when cable is tangled/draped on terrain
- Inequality constraint (not equality — cable has slack when tangled)
- Prevents catastrophic DR divergence
- Loose when lots of cable on ground; tight when cable is mostly airborne
- Implementation: if EKF position exceeds cable length, project back onto circle

## Altitude Strategy

- GPS-calibrated barometer at takeoff → accurate initial altitude
- Baro P-controller for altitude hold during flight: `vz = -clamp((target_alt - baro_alt) * kp, -max_vz, max_vz)`
- Accept gradual baro drift (30-80m over extended flight)
- Descent: time-based (proven reliable in current system)
- Landing: time-based + baro relative altitude check
- **Do NOT use altitude for horizontal navigation decisions**

## Wind Estimation

In FW flight, optical flow measures air-relative velocity direction while spool measures ground-relative speed. The difference is wind:

```
v_air = R(attitude) * (spool_speed * OF_dir)   # what OF "sees"
v_ground = spool_speed * heading_dir             # what spool measures
wind = v_air - v_ground                          # estimated wind vector
```

Wind state in EKF allows correcting FW course for crosswind drift.

In MC flight, wind estimation is less reliable (body-frame OF and ground speed are less distinguishable in hover).

## Capabilities Enabled

| Capability | Current (time-based) | With Position EKF |
|-----------|---------------------|-------------------|
| Outbound navigation | Fixed heading, time-based WP | Steer toward WP, distance-based acceptance |
| Course correction | None | Yes — correct for wind/drift |
| Return to home | Fixed reverse heading, time-based | Steer toward (0,0) continuously |
| Non-straight missions | Impossible | L-shaped, zigzag, area coverage |
| Landing accuracy | Wherever time runs out | Within ~50-100m of home at 2km |
| Wind handling | None (fixed heading drifts) | Estimate and compensate |
| Cable safety | Not monitored | Position vs cable length check |
| Mission range | ~400m straight line | Multi-km complex routes |

## Honest Limitations

1. **Altitude is "good enough" not "accurate"** — baro drifts, we accept it, use time-based descent
2. **2-5% position drift** — won't land on a dime, but will get within ~50-100m of home at 2km
3. **Cable constraint is loose when tangled** — prevents catastrophic errors but doesn't tighten position when lots of cable on ground
4. **Wind estimation degrades in MC mode** — body-frame OF and ground speed less distinguishable in hover
5. **No absolute position fixes** — pure DR, no loop closure or terrain map matching
6. **OF quality depends on terrain texture** — featureless terrain (water, sand) degrades OF → fallback to odometry drift model

## Implementation Components

### New nodes/files:
1. **`position_ekf` node** — 6-state EKF (x, y, vx, vy, wind_x, wind_y)
   - Subscribes: spool velocity, OF direction, IMU attitude, cable status
   - Publishes: estimated position, velocity, uncertainty ellipse
   - Cable length inequality constraint
   - Adaptive noise based on OF quality

2. **`gps_denied_navigator` node** (or upgrade `vtol_navigation_mode`)
   - Uses position estimate for WP steering
   - Course correction for wind drift
   - Return-to-home: continuous steering toward (0,0)
   - Cable length monitoring and abort logic

3. **Updated mission configs**
   - L-shaped, zigzag, and area coverage mission profiles
   - Distance-based WP acceptance (with time-based fallback)
   - Position-based return trigger (not just time)

### Modified files:
4. **`fiber_vision_fusion`** — feed position EKF output to PX4 VehicleVisualOdometry
5. **Foxglove layout** — position estimate vs cable length plots, DR trajectory overlay
6. **Launch files** — integrate new nodes

## Verification Plan

1. **Unit tests**: EKF prediction, cable constraint projection, wind estimation
2. **Straight-line mission**: compare DR position vs ground truth, measure drift %
3. **L-shaped mission**: verify course correction at turn
4. **Return-to-home**: measure landing distance from home position
5. **Wind test**: add Gazebo wind, verify wind estimator and course correction
6. **Cable tangle scenario**: verify constraint doesn't over-constrain when cable drapes
7. **OF degradation**: test fallback when flying over featureless terrain

## Research References

- [Self-Localization of Tethered Drones (MDPI 2021)](https://www.mdpi.com/2504-446X/5/4/135) — EKF with cable, <0.3m error when tension >1N
- [Tethered System Heading Estimation (AIAA 2024)](https://arc.aiaa.org/doi/abs/10.2514/6.2024-4288) — 5-state EKF for position + heading
- [Multi-model Tether-based Localization](https://www.researchgate.net/publication/371346100_A_Multi-model_Framework_for_Tether-based_Drone_Localization)
- [PX4 Optical Flow Integration](https://docs.px4.io/main/en/sensor/optical_flow) — height-scaled flow → velocity
- [OKSI OMNInav](https://oksi.ai/omninav-gps-denied-navigation/) — 0.5% endpoint error with AI sensor fusion
- [PiDR: Physics-Informed Dead Reckoning (2026)](https://arxiv.org/html/2601.03040v1) — physics constraints in DNN for DR
- [Advanced Navigation: Dead Reckoning](https://www.advancednavigation.com/tech-articles/an-introduction-to-dead-reckoning/) — 1-5% drift typical for DR systems
