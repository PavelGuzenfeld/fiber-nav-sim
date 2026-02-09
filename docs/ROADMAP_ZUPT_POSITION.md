# ZUPT + 1D Position Clamping

## Overview

Enhancement to the fiber-nav-sim GPS-denied navigation pipeline that adds:

1. **Wire ZUPT (Zero-velocity Update)**: Hard-resets velocity to zero when the spool reports no motion, preventing EKF velocity drift when the drone is stationary.

2. **1D Position Clamping (Drag Bow Model)**: Uses the accumulated spool fiber length plus a drag bow correction to provide a position estimate along the tunnel axis, giving the EKF a position constraint it previously lacked.

## Parameters

| Parameter | Node | Default | Description |
|-----------|------|---------|-------------|
| `moving_threshold` | spool_sim_driver | 0.05 m/s | Speed below which spool reports not moving |
| `zupt_threshold` | fiber_vision_fusion | 0.05 m/s | Speed below which ZUPT activates |
| `zupt_velocity_variance` | fiber_vision_fusion | 0.001 | Velocity variance during ZUPT (very tight) |
| `enable_position_clamping` | fiber_vision_fusion | true | Enable drag bow position estimate |
| `k_drag` | fiber_vision_fusion | 0.0005 | Drag bow coefficient |
| `tunnel_heading_deg` | fiber_vision_fusion | 90.0 | NED heading of tunnel axis (degrees) |
| `position_variance_longitudinal` | fiber_vision_fusion | 1.0 | Position variance along tunnel (m^2) |
| `position_variance_lateral` | fiber_vision_fusion | 100.0 | Position variance perpendicular to tunnel (m^2) |

## Drag Bow Model

The fiber cable drags through the tunnel with some slack. The straight-line distance from entry to the drone is less than the total unrolled fiber length due to cable sag ("drag bow"). The correction model is:

```
x_est = L_total * (1 - k_drag * v^2)
```

Where:
- `L_total` = total unrolled fiber length (from spool encoder)
- `v` = current spool payout velocity
- `k_drag` = drag coefficient (higher velocity → more drag → larger correction)
- `x_est` = estimated straight-line distance along tunnel axis

The position estimate is then projected onto the NED frame using the tunnel heading:

```
pos_north = x_est * cos(heading)
pos_east  = x_est * sin(heading)
pos_down  = NaN (altitude unconstrained by spool)
```

## Variance Model

The position variance is anisotropic — tight along the tunnel axis (we know approximately how far we are) and loose perpendicular to it (we don't know lateral offset from the fiber):

```
var_north = var_long * cos²(h) + var_lat * sin²(h)
var_east  = var_long * sin²(h) + var_lat * cos²(h)
var_down  = 1e6 (unconstrained)
```

## Message: SpoolStatus

New message type `fiber_nav_sensors/SpoolStatus`:

```
std_msgs/Header header
float32 velocity          # Payout rate (m/s), non-negative
float32 total_length      # Total unrolled fiber length (m)
bool is_moving            # True if velocity > threshold
```

Published on `/sensors/fiber_spool/status`. The original Float32 velocity topic is preserved for backward compatibility.

## Integration Notes

- PX4 EKF2 should now show `cs_ev_pos: true` in addition to `cs_ev_vel: true`
- Set `EKF2_EV_CTRL` to include position fusion (bit 0) alongside velocity (bit 2)
- The ZUPT variance (0.001) is much tighter than normal velocity variance (0.01), giving the EKF a strong zero-velocity constraint when stopped
- Position altitude (Z) is always NaN — the spool provides no altitude information
