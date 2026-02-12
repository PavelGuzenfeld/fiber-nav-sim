# Plan: Optical Flow Vision Direction + Fish-Eye Down Camera

## Context

The current `vision_direction_sim` cheats — it reads Gazebo ground truth odometry velocity, normalizes it, and adds random walk drift. Now that we have realistic terrain with satellite texture, we can derive motion direction from actual camera images using optical flow on the down-pointing camera. Adding fish-eye distortion makes the simulation more realistic (fiber optic cameras are typically wide-angle).

## Approach

1. Add fish-eye barrel distortion to the down camera in SDF (Gazebo native rendering)
2. Create a new `optical_flow_direction` node that processes `/camera_down` images
3. Sparse Lucas-Kanade optical flow (CPU-friendly under Gazebo load)
4. Fallback to odometry-based direction when flow quality is poor (same drift model as old sim)

---

## Change 1: Fish-Eye Distortion on Down Camera

### Both `model.sdf` and `model_px4.sdf`

Add `<distortion>` block inside `camera_down`'s `<camera>` element. Widen FOV from 60° to 80° to compensate for barrel distortion shrinking effective field at edges:

```xml
<camera>
  <horizontal_fov>1.39626</horizontal_fov>  <!-- widened from 1.047 -->
  ...
  <distortion>
    <k1>-0.21</k1>
    <k2>0.05</k2>
    <k3>0.0004</k3>
    <p1>-0.0003</p1>
    <p2>-0.0003</p2>
    <center>0.5 0.5</center>
  </distortion>
</camera>
```

Coefficients: moderate barrel distortion (GoPro-like). Gazebo Harmonic confirmed to implement `<distortion>` via render passes (gz-sensors #107, closed as complete).

---

## Change 2: New `optical_flow_direction` Node

### Header: `include/fiber_nav_sensors/optical_flow_direction.hpp`

Pure testable functions:

- `FlowResult compute_dominant_flow(prev_pts, curr_pts, status, min_displacement)` — median flow direction + quality from LK tracked points
- `std::array<float,3> flow_to_body_direction(flow_x, flow_y)` — image plane to body frame mapping

`FlowResult` struct: `{dir_x, dir_y, magnitude, quality}` where quality = fraction of successfully tracked points.

### Node: `src/optical_flow_direction.cpp`

**Subscribers:**
- `/camera_down` — `sensor_msgs/Image`, `SensorDataQoS` (BEST_EFFORT)
- `/model/{model}/odometry` — `nav_msgs/Odometry` (fallback source)

**Publishers:**
- `/sensors/vision_direction` — `geometry_msgs/Vector3Stamped` (same topic as old sim, drop-in replacement)
- `/sensors/optical_flow/quality` — `std_msgs/Float64` (for Foxglove diagnostics)
- `/sensors/optical_flow/direction` — `geometry_msgs/Vector3Stamped` — raw OF direction before fallback (body frame)
- `/sensors/optical_flow/source` — `std_msgs/Float64` — 1.0 = optical flow, 0.0 = fallback

**Parameters:**
| Param | Default | Description |
|-------|---------|-------------|
| `model_name` | `"quadtailsitter"` | For odometry fallback topic |
| `max_features` | `200` | Shi-Tomasi corners to detect |
| `min_quality` | `0.3` | Below this → odometry fallback |
| `process_scale` | `0.5` | Downsample factor (320x240) |
| `min_displacement` | `0.5` | Min avg pixel flow to count |
| `fallback_drift_rate` | `0.001` | rad/s drift for odometry fallback |
| `publish_rate` | `15.0` | Output rate cap (Hz) |
| `redetect_interval` | `10` | Re-detect corners every N frames |

**Algorithm per frame:**
1. Convert to `mono8` via cv_bridge, downsample by `process_scale`
2. First frame: detect Shi-Tomasi corners → store as `prev_pts_`, return
3. Track with `cv::calcOpticalFlowPyrLK(prev_gray_, curr_gray, prev_pts_, curr_pts, status, err)`
4. Call `compute_dominant_flow()` → `FlowResult`
5. If `quality >= min_quality`: convert flow to body direction via `flow_to_body_direction()`, publish
6. If `quality < min_quality`: use odometry velocity + drift (same algorithm as `vision_direction_sim`)
7. Re-detect corners every `redetect_interval` frames or when tracked count < `max_features / 3`
8. Publish quality metric

**Body frame mapping (model_px4.sdf tailsitter):**
Down camera has pose `0.1 0 0 0 0 0` — looks along body +X (nadir in hover).
- Image +X (right) → body +Y
- Image +Y (down) → body +Z (which is forward in tailsitter)
- `body_direction = normalize(0, flow_x, flow_y)`

---

## Change 3: CMakeLists.txt

Add after `vision_direction_sim` block:

```cmake
add_executable(optical_flow_direction src/optical_flow_direction.cpp)
target_include_directories(optical_flow_direction PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(optical_flow_direction
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge)
target_link_libraries(optical_flow_direction ${OpenCV_LIBS})
```

Add to `install(TARGETS ...)`. Add test `test_optical_flow` with doctest.

---

## Change 4: sensor_params.yaml

```yaml
optical_flow_direction:
  ros__parameters:
    model_name: "quadtailsitter"
    max_features: 200
    min_quality: 0.3
    process_scale: 0.5
    min_displacement: 0.5
    fallback_drift_rate: 0.001
    publish_rate: 15.0
    redetect_interval: 10
```

---

## Change 5: Launch File

In `sensors.launch.py`: replace `vision_direction_sim` executable with `optical_flow_direction`. Keep old sim available but not launched by default.

---

## Change 6: Foxglove — Optical Flow Visualization in Sensors Tab

### New publishers from `optical_flow_direction` node

In addition to the existing `/sensors/vision_direction` (Vector3Stamped) and `/sensors/optical_flow/quality` (Float64), also publish:

- `/sensors/optical_flow/direction` — `geometry_msgs/Vector3Stamped` — raw OF direction before fallback (body frame)
- `/sensors/optical_flow/source` — `std_msgs/Float64` — 1.0 = optical flow, 0.0 = fallback

### `foxglove/fiber_nav_layout.json`

Add to **Sensors tab** layout: a new `Plot!optical_flow` panel alongside the existing direction/spool plots.

**Plot!optical_flow** config — 4 series showing flow vs fused direction:
- `/sensors/optical_flow/direction.vector.x` (label: "OF X", color: red, dashed)
- `/sensors/optical_flow/direction.vector.y` (label: "OF Y", color: green, dashed)
- `/sensors/vision_direction.vector.x` (label: "Fused X", color: red, solid)
- `/sensors/vision_direction.vector.y` (label: "Fused Y", color: green, solid)

**Plot!flow_quality** config — quality + source indicator:
- `/sensors/optical_flow/quality.data` (label: "OF Quality", color: blue)
- `/sensors/optical_flow/source.data` (label: "Source (1=OF)", color: orange)

Place these in the Sensors tab: split `direction_proof` row to include `Plot!optical_flow`, and add `Plot!flow_quality` below.

---

## Files Modified

| File | Change |
|------|--------|
| `src/fiber_nav_gazebo/models/quadtailsitter/model_px4.sdf` | Add `<distortion>` + wider FOV to camera_down |
| `src/fiber_nav_gazebo/models/quadtailsitter/model.sdf` | Same distortion changes |
| `src/fiber_nav_sensors/include/fiber_nav_sensors/optical_flow_direction.hpp` | **NEW** — pure functions |
| `src/fiber_nav_sensors/src/optical_flow_direction.cpp` | **NEW** — OF node |
| `src/fiber_nav_sensors/test/test_optical_flow.cpp` | **NEW** — unit tests |
| `src/fiber_nav_sensors/CMakeLists.txt` | Add executable + test |
| `src/fiber_nav_bringup/config/sensor_params.yaml` | Add OF params |
| `src/fiber_nav_bringup/launch/sensors.launch.py` | Switch to OF node |
| `foxglove/fiber_nav_layout.json` | Add OF direction + quality plots to Sensors tab |

## Implementation Order

1. SDF distortion on both model files
2. Header with pure functions
3. Unit tests (TDD)
4. Node implementation (including OF direction + quality + source publishers)
5. CMakeLists.txt
6. sensor_params.yaml + launch file
7. Foxglove layout — OF plots in Sensors tab
8. Build + test in container
9. E2E: GPS-denied VTOL mission with OF-based direction

## Verification

1. **Build**: `colcon build --packages-select fiber_nav_sensors` in container
2. **Unit tests**: `ctest` — test_optical_flow passes
3. **Fish-eye visual**: Foxglove camera_down shows barrel distortion
4. **OF quality**: `/sensors/optical_flow/quality` > 0.3 during flight over textured terrain
5. **Direction plots**: Sensors tab shows OF direction vs fused direction — should track closely when quality is high, diverge when fallback kicks in
6. **Source indicator**: shows 1.0 during good OF, 0.0 during fallback
7. **Fallback**: direction still published when hovering (low texture, no features)
8. **E2E**: GPS-denied VTOL mission completes with OF-based direction
