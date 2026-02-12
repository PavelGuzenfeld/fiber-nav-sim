#pragma once

#include <array>
#include <cmath>
#include <vector>

#include <opencv2/core.hpp>

namespace fiber_nav_sensors {

struct FlowResult {
    float dir_x = 0.0f;      // Median flow X (pixels)
    float dir_y = 0.0f;      // Median flow Y (pixels)
    float magnitude = 0.0f;  // Median flow magnitude (pixels)
    float quality = 0.0f;    // Fraction of successfully tracked points [0,1]
};

/// Compute dominant flow direction from Lucas-Kanade tracked points.
/// Uses median filtering for outlier robustness.
///
/// @param prev_pts  Feature positions in previous frame
/// @param curr_pts  Feature positions in current frame
/// @param status    LK status vector (1 = tracked, 0 = lost)
/// @param min_displacement  Minimum avg pixel displacement to consider valid
/// @return FlowResult with median direction, magnitude, and quality metric
inline FlowResult compute_dominant_flow(
    const std::vector<cv::Point2f>& prev_pts,
    const std::vector<cv::Point2f>& curr_pts,
    const std::vector<uchar>& status,
    float min_displacement)
{
    if (prev_pts.empty() || prev_pts.size() != curr_pts.size()
        || prev_pts.size() != status.size()) {
        return {};
    }

    // Collect valid flow vectors
    std::vector<float> flow_x;
    std::vector<float> flow_y;
    std::vector<float> flow_mag;
    flow_x.reserve(prev_pts.size());
    flow_y.reserve(prev_pts.size());
    flow_mag.reserve(prev_pts.size());

    for (size_t i = 0; i < prev_pts.size(); ++i) {
        if (status[i] == 0) continue;
        float dx = curr_pts[i].x - prev_pts[i].x;
        float dy = curr_pts[i].y - prev_pts[i].y;
        flow_x.push_back(dx);
        flow_y.push_back(dy);
        flow_mag.push_back(std::sqrt(dx * dx + dy * dy));
    }

    if (flow_x.empty()) {
        return {};
    }

    float quality = static_cast<float>(flow_x.size())
                  / static_cast<float>(prev_pts.size());

    // Median via nth_element
    auto median = [](std::vector<float>& v) -> float {
        size_t n = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + static_cast<ptrdiff_t>(n), v.end());
        return v[n];
    };

    float med_x = median(flow_x);
    float med_y = median(flow_y);
    float med_mag = median(flow_mag);

    if (med_mag < min_displacement) {
        return {med_x, med_y, med_mag, quality};
    }

    return {med_x, med_y, med_mag, quality};
}

/// Convert image-plane optical flow to body-frame direction vector.
///
/// For model_px4.sdf tailsitter, down camera at pose `0.1 0 0 0 0 0`
/// looks along body +X (nadir in hover):
///   Image +X (right)  → body +Y
///   Image +Y (down)   → body +Z (forward in tailsitter)
///
/// Returns normalized [bx, by, bz] direction. Returns [0,0,0] if flow is zero.
inline std::array<float, 3> flow_to_body_direction(float flow_x, float flow_y)
{
    // Map image flow to body frame
    float by = flow_x;   // image right → body +Y
    float bz = flow_y;   // image down  → body +Z

    float norm = std::sqrt(by * by + bz * bz);
    if (norm < 1e-9f) {
        return {0.0f, 0.0f, 0.0f};
    }

    return {0.0f, by / norm, bz / norm};
}

}  // namespace fiber_nav_sensors
