#pragma once

#include <functional>
#include <span>
#include <string>
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
    float sigma = 0.f;     // Position uncertainty estimate [m]
    bool valid = false;    // par > threshold && ncc > min_ncc
};

struct TercomConfig {
    int min_samples = 10;         // Minimum profile length for matching
    float sample_spacing = 12.f;  // Desired spacing ~= DEM resolution [m]
    float search_radius = 300.f;  // Grid search radius around EKF estimate [m]
    float search_step = 12.f;     // Grid search step [m] (~= DEM resolution)
    float min_ncc = 0.5f;         // Minimum NCC for valid match
    float par_threshold = 1.5f;   // Minimum PAR for valid match
    float par_sigma_scale = 50.f; // sigma = par_sigma_scale / par [m]
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

}  // namespace fiber_nav_fusion
