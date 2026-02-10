# VTOL Navigation Mode

A generalized VTOL waypoint navigation mode using px4-ros2-interface-lib that automatically handles MC/FW transitions. Replaces the Python offboard approach with a robust C++ custom PX4 flight mode.

## Architecture

### VtolNavigationMode (ModeBase)

7-state machine for VTOL waypoint navigation:

```
MC_CLIMB → TRANSITION_FW → FW_NAVIGATE → FW_RETURN → TRANSITION_MC → MC_APPROACH → DONE
```

| State | Control | Description |
|-------|---------|-------------|
| MC_CLIMB | TrajectorySetpoint (velocity) | Climb at configured rate, yaw toward first WP |
| TRANSITION_FW | VTOL::toFixedwing() + accel setpoint | MC→FW pitch-over with altitude hold |
| FW_NAVIGATE | FW course+altitude (NPFG+TECS) | Fly waypoint sequence with flythrough acceptance |
| FW_RETURN | FW course+altitude (NPFG+TECS) | Course toward home (0,0) in FW mode |
| TRANSITION_MC | VTOL::toMulticopter() + accel setpoint | FW→MC back-transition with deceleration |
| MC_APPROACH | MulticopterGoto | Fly to home position, heading toward home |
| DONE | completed() | Signal mission success to executor |

**FW control strategy:** All FW lateral/longitudinal control is delegated to PX4's built-in NPFG (lateral path following) and TECS (altitude/speed management) controllers via `FwLateralLongitudinalSetpointType`. The mode computes only the course angle and target altitude.

**Quad-chute detection:** During FW flight states (FW_NAVIGATE, FW_RETURN), the mode monitors `vtol_vehicle_status`. If PX4 autonomously reverts to MC mode (quad-chute), the mode immediately switches to MC_APPROACH.

### VtolMissionExecutor (ModeExecutorBase)

Thin lifecycle wrapper:

```
Arming (with retry) → TakingOff → Navigating → Landing → WaitDisarmed
```

- Uses `ActivateImmediately` — PX4 activates the executor right after registration
- Arm retry logic: up to 15 retries at 2s intervals (PX4 may reject early arming)
- Mission abort flag prevents auto-restart after mid-flight deactivation
- Skips message compatibility check (`setSkipMessageCompatibilityCheck()`) to avoid blocking during registration

## Files

| File | Description |
|------|-------------|
| `include/fiber_nav_mode/vtol_navigation_mode.hpp` | 7-state VTOL nav with NPFG+TECS FW control |
| `include/fiber_nav_mode/vtol_mission_executor.hpp` | Arm→takeoff→navigate→land→disarm executor |
| `src/vtol_navigation_node.cpp` | ROS 2 node, config from YAML params |
| `config/canyon_mission.yaml` | Canyon 1400m, 7 WPs, 150m cruise alt |
| `config/straight_500m.yaml` | Simple straight-line test |
| `test/test_vtol_navigation.cpp` | 33 unit tests |

## Configuration

Missions are defined in YAML parameter files loaded via `custom_mode.launch.py`:

```yaml
vtol_navigation_node:
  ros__parameters:
    cruise_altitude: 150.0          # AGL [m]
    climb_rate: 4.0                 # [m/s]
    fw_accept_radius: 50.0          # [m] flythrough radius
    mc_transition_dist: 200.0       # [m] switch to MC when this close to home
    mc_approach_speed: 5.0          # [m/s]
    fw_transition_timeout: 30.0     # [s]
    mc_transition_timeout: 60.0     # [s]

    waypoints:
      x: [0.0, 0.0, 0.0, 0.0, 0.0, -20.0, -40.0]     # NED north [m]
      y: [200.0, 400.0, 600.0, 800.0, 1000.0, 1200.0, 1400.0]  # NED east [m]
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cruise_altitude` | 50.0 | AGL altitude for FW cruise [m] |
| `climb_rate` | 2.0 | MC vertical climb rate [m/s] |
| `fw_accept_radius` | 50.0 | Flythrough radius for FW waypoints [m] |
| `mc_transition_dist` | 200.0 | Transition to MC when within this distance from home [m] |
| `mc_approach_speed` | 5.0 | MC approach horizontal speed [m/s] |
| `fw_transition_timeout` | 30.0 | Max time for MC→FW transition [s] |
| `mc_transition_timeout` | 60.0 | Max time for FW→MC transition [s] |

Per-waypoint `heading` and `accept_radius` overrides are supported via optional parallel arrays.

## Launching

```bash
# Inside the px4-sitl container (after building):
ros2 launch fiber_nav_bringup custom_mode.launch.py \
    mode:=vtol \
    params_file:=/root/ws/install/fiber_nav_mode/share/fiber_nav_mode/config/canyon_mission.yaml
```

Prerequisites (in separate terminals):
1. Gazebo + ros_gz_bridge (`simulation.launch.py use_px4:=true headless:=true`)
2. MicroXRCE-DDS agent (`MicroXRCEAgent udp4 -p 8888`)
3. PX4 SITL (`PX4_SYS_AUTOSTART=4251 PX4_GZ_MODEL_NAME=quadtailsitter ../bin/px4`)

The mode registers with PX4, arms, takes off, and flies the mission autonomously.

## Integration Test Results

Canyon mission: 7 waypoints, 1400m east, 150m cruise altitude.

| Phase | Duration | Result |
|-------|----------|--------|
| MC_CLIMB | 219s | 150m AGL, yaw toward WP0 (East) |
| TRANSITION_FW | 20.7s | Clean transition, no errors |
| FW_NAVIGATE | 256.7s | All 7 WPs reached, alt 147-151m |
| FW_RETURN | 273.9s | FW course home 1352m→200m, alt 150m |
| TRANSITION_MC | 17.5s | Clean back-transition |
| MC_APPROACH | 127.9s | MC to within 5m of home |
| Landing | ~416s | PX4 auto-land from 150m |
| **Total** | **~1393s** | **Full success: arm → fly → disarm** |

## PX4 Airframe Parameters

Critical parameters in `4251_gz_quadtailsitter_vision` for VTOL navigation:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `VT_FW_QC_P` | 0 | Disable pitch quad-chute (SITL) |
| `VT_FW_QC_R` | 0 | Disable roll quad-chute (SITL) |
| `VT_FW_MIN_ALT` | 0 | Disable altitude quad-chute (SITL) |
| `COM_OF_LOSS_T` | 10 | Offboard loss timeout [s] (default 1s too tight for Gazebo) |
| `COM_OBC_LOSS_T` | 30 | External mode loss timeout [s] |
| `CBRK_AIRSPD_CHK` | 162128 | Disable airspeed check (no sensor in SITL) |
| `FW_ARSP_MODE` | 1 | Manual airspeed mode |
| `EKF2_GPS_CHECK` | 0 | Disable GPS quality gate (slow sim-time) |
| `FW_THR_TRIM` | 0.65 | FW cruise throttle |
| `FW_AIRSPD_TRIM` | 18 | FW trim airspeed [m/s] |

## VTOL Timeout Patch

The px4-ros2-interface-lib `VTOL` class has a 2-second freshness check on `vtol_vehicle_status`, but PX4 publishes it at ~1 Hz with BEST_EFFORT QoS. Under Gazebo CPU load, messages drop and all transition calls fail.

**Fix:** The Dockerfile patches `vtol.cpp` during build:
```dockerfile
sed -i 's/< 2s)/< 10s)/g' ${WORKSPACE}/src/px4-ros2-interface-lib/px4_ros2_cpp/src/control/vtol.cpp
```

This changes the freshness check from 2s to 10s in both `toFixedwing()` and `toMulticopter()`.

## Lessons Learned

1. **Gazebo CPU load is the #1 enemy** — Causes DDS message drops, offboard timeouts, build starvation. Pause Gazebo during builds (`kill -STOP`).

2. **`docker stop/start` preserves state** — Restarts PID 1 (fresh Gazebo) but keeps filesystem (build artifacts, patches). `docker compose down` destroys everything.

3. **PX4 offboard timeouts are tight** — Default 1s offboard timeout causes PX4 to deactivate external modes under SITL load. Set `COM_OF_LOSS_T=10`.

4. **Tailsitter FW transition loses 30-50m altitude** — The 90-degree pitch-forward is aggressive. Use 150m+ cruise altitude.

5. **VTOL class default timestamp = epoch(0)** — All transition calls fail until first `vtol_vehicle_status` message arrives.

6. **`deferFailsafesSync()` + `ActivateImmediately` = crash** — Synchronous spin conflicts with registration wait set. Don't use them together.

7. **MC_CLIMB must yaw toward first WP** — Otherwise FW transition accelerates in the wrong direction.

8. **`ActivateImmediately` auto-restarts after deactivation** — Need a mission abort flag to prevent infinite restart loops.

9. **`OdometryGlobalPosition` sets GPS requirement** — Use raw `rclcpp::Subscription<VehicleGlobalPosition>` instead to avoid requiring GNSS to arm.

10. **Message compatibility check blocks forever** — `doRegister()` does a 44-message format negotiation. Call `setSkipMessageCompatibilityCheck()` in the constructor.

## Future Work

- **FW cross-track tuning** — Vehicle drifted to x=-144m during FW_RETURN. Tune NPFG period/damping and FW roll gains.
- ~~**Vision fusion during FW flight**~~ — **DONE (Phase 3.1):** Flight mode awareness with adaptive variance scaling (0.01 FW, 0.04 transitions, 0.001 ZUPT). Diagnostics at 1 Hz. Sensor health monitoring via ring buffer. E2E verified across full VTOL mission.
- **GPS-denied FW navigation** — Fly canyon mission using only fiber spool + vision fusion.
- **Terrain following** — Range finder-based altitude adjustment for canyon terrain profile.
- **CI/CD integration** — Automated SITL mission execution with pass/fail criteria.
