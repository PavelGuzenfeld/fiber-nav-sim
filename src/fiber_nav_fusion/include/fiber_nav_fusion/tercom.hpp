#pragma once

#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace fiber_nav_fusion {

/// Pre-loaded DEM grid for TERCOM matching
struct TerrainMap {
    std::vector<float> elevation;  // Row-major, Gazebo Z (above min_elev)
    int width = 0;                 // pixels
    int height = 0;                // pixels
    float meters_per_pixel = 0.f;  // spatial resolution
    float origin_x = 0.f;         // center X in NED [m]
    float origin_y = 0.f;         // center Y in NED [m]

    /// Bilinear lookup at NED (x, y). Returns Gazebo Z.
    float height_at(float x, float y) const;
};

/// A single terrain altitude sample
struct TerrainSample {
    float agl;       // measured AGL from laser [m]
    float baro_alt;  // barometric altitude [m] (from PX4 VehicleLocalPosition.z, negated)
    float dx;        // displacement since last sample [m] (from dead-reckoning)
    float dy;        // displacement since last sample [m]
};

/// TERCOM match result
struct TercomResult {
    float x = 0.f;        // Best match position NED X [m]
    float y = 0.f;        // Best match position NED Y [m]
    float ncc = 0.f;       // Normalized cross-correlation peak [-1, 1]
    float par = 0.f;       // Peak ambiguity ratio (peak / 2nd best)
    float sigma = 0.f;     // Position uncertainty estimate [m] (isotropic, legacy)
    bool valid = false;    // par > threshold && ncc > min_ncc

    // Anisotropic covariance (NED frame, from NCC Hessian at peak)
    float var_xx = 0.f;   // Position variance in X (North) [m²]
    float var_yy = 0.f;   // Position variance in Y (East) [m²]
    float var_xy = 0.f;   // Cross-covariance [m²]
};

struct TercomConfig {
    int min_samples = 10;         // Minimum profile length for matching
    float sample_spacing = 12.f;  // Desired spacing ~= DEM resolution [m]
    float search_radius = 300.f;  // Grid search radius around EKF estimate [m]
    float search_step = 12.f;     // Grid search step [m] (~= DEM resolution)
    float min_ncc = 0.5f;         // Minimum NCC for valid match
    float par_threshold = 1.5f;   // Minimum PAR for valid match
    float par_sigma_scale = 50.f; // sigma = par_sigma_scale / par [m]

    // Coarse-to-fine search
    float coarse_factor = 2.f;    // Coarse step = search_step * coarse_factor
    int refine_top_n = 3;         // Number of top coarse candidates to refine

    // Anisotropic uncertainty from NCC Hessian
    float hessian_variance_scale = 100.f;  // Maps NCC curvature to position variance
};

/// Build terrain profile from AGL + baro: terrain_z = baro_alt - agl
std::vector<float> build_profile(std::span<const TerrainSample> samples);

/// Extract DEM profile along a path starting at (x0,y0) with given displacements.
std::vector<float> extract_dem_profile(
    const TerrainMap& map,
    float x0, float y0,
    std::span<const TerrainSample> samples);

/// Normalized cross-correlation between two equal-length profiles.
/// Mean-invariant (handles baro offset drift).
float normalized_cross_correlation(
    std::span<const float> measured,
    std::span<const float> reference);

/// Grid-search TERCOM: slide the measured profile across the DEM
/// around (est_x, est_y) within search_radius.
TercomResult tercom_match(
    const TerrainMap& map,
    std::span<const TerrainSample> samples,
    float est_x, float est_y,
    const TercomConfig& config);

/// Load terrain DEM from terrain_data.json directory.
/// Returns populated TerrainMap (width=0 on failure).
TerrainMap load_terrain_map(const std::string& terrain_data_path,
                          std::function<void(const std::string&)> log_fn = {});

// --- Path prior: terrain discriminability for cross-track EKF constraint ---

/// Pre-computed mission leg with terrain discriminability profile.
struct MissionLeg {
    float start_x, start_y, end_x, end_y;  // NED [m]
    float heading, length;
    float along_x, along_y;   // unit along-track: (cos(h), sin(h))
    float cross_x, cross_y;   // unit cross-track: (-sin(h), cos(h))
    std::vector<float> disc_scores;  // [0,1] per sample point
    float disc_step;                 // along-track spacing [m]
};

/// Result of projecting a position onto the nearest mission leg.
struct LegProjection {
    int leg_index = -1;
    float along_track = 0.f;
    float cross_track = 0.f;       // signed distance from centerline [m]
    float discriminability = 0.f;  // interpolated score [0,1]
};

/// Pre-compute terrain discriminability along a mission path.
/// For each leg, samples terrain profiles at cross-track offsets and computes
/// NCC-based discriminability: 0 = featureless (all offsets look same),
/// 1 = distinctive (unique cross-track position).
std::vector<MissionLeg> compute_mission_discriminability(
    const TerrainMap& map,
    std::span<const std::pair<float, float>> waypoints,
    int disc_profile_length, float disc_spacing,
    float disc_corridor_width, float disc_step);

/// Project a position onto the nearest mission leg.
/// Returns leg index, along/cross-track distances, and interpolated discriminability.
LegProjection project_onto_legs(
    float x, float y,
    std::span<const MissionLeg> legs,
    float corridor_width);

}  // namespace fiber_nav_fusion
