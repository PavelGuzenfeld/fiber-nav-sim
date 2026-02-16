#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <fiber_nav_fusion/tercom.hpp>

#include <cmath>
#include <utility>
#include <vector>

using namespace fiber_nav_fusion;

// Helper: create a simple ramp DEM (elevation increases linearly with X)
static TerrainMap makeRampDem(int size, float mpp, float slope) {
    TerrainMap map;
    map.width = size;
    map.height = size;
    map.meters_per_pixel = mpp;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.resize(size * size);

    float half = static_cast<float>(size) / 2.f;
    for (int r = 0; r < size; ++r) {
        for (int c = 0; c < size; ++c) {
            // NED X = -(r - half) * mpp (row 0 = north = +X)
            float x = -(static_cast<float>(r) - half) * mpp;
            map.elevation[r * size + c] = slope * x;
        }
    }
    return map;
}

// Helper: create a sinusoidal DEM (unique terrain signature)
// Frequencies chosen so a 180m profile (~15 samples at 12m) captures >1 full cycle,
// giving enough spatial variation for TERCOM to disambiguate positions.
static TerrainMap makeSineDem(int size, float mpp) {
    TerrainMap map;
    map.width = size;
    map.height = size;
    map.meters_per_pixel = mpp;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.resize(size * size);

    float half = static_cast<float>(size) / 2.f;
    for (int r = 0; r < size; ++r) {
        for (int c = 0; c < size; ++c) {
            float x = -(static_cast<float>(r) - half) * mpp;
            float y = (static_cast<float>(c) - half) * mpp;
            // Multiple frequencies with cross-terms for unique spatial fingerprint.
            map.elevation[r * size + c] =
                10.f * std::sin(x * 0.05f) +
                5.f * std::sin(y * 0.1f + x * 0.025f) +
                3.f * std::cos(x * 0.08f - y * 0.04f);
        }
    }
    return map;
}

TEST_CASE("build_profile: terrain_z = baro_alt - agl") {
    std::vector<TerrainSample> samples = {
        {.agl = 100.f, .baro_alt = 150.f, .dx = 0.f, .dy = 0.f},
        {.agl = 95.f,  .baro_alt = 148.f, .dx = 10.f, .dy = 0.f},
        {.agl = 90.f,  .baro_alt = 145.f, .dx = 10.f, .dy = 0.f},
    };

    auto profile = build_profile(samples);

    REQUIRE(profile.size() == 3);
    CHECK(profile[0] == doctest::Approx(50.f));   // 150 - 100
    CHECK(profile[1] == doctest::Approx(53.f));   // 148 - 95
    CHECK(profile[2] == doctest::Approx(55.f));   // 145 - 90
}

TEST_CASE("NCC: identical profiles produce 1.0") {
    std::vector<float> a = {1.f, 3.f, 5.f, 7.f, 9.f};
    std::vector<float> b = a;

    float ncc = normalized_cross_correlation(a, b);
    CHECK(ncc == doctest::Approx(1.0f).epsilon(1e-5));
}

TEST_CASE("NCC: inverted profiles produce -1.0") {
    std::vector<float> a = {1.f, 3.f, 5.f, 7.f, 9.f};
    std::vector<float> b = {9.f, 7.f, 5.f, 3.f, 1.f};

    float ncc = normalized_cross_correlation(a, b);
    CHECK(ncc == doctest::Approx(-1.0f).epsilon(1e-5));
}

TEST_CASE("NCC: offset-invariant (adding constant doesn't change result)") {
    std::vector<float> a = {1.f, 3.f, 5.f, 7.f, 9.f};
    std::vector<float> b = a;
    std::vector<float> b_offset(b.size());
    for (std::size_t i = 0; i < b.size(); ++i) {
        b_offset[i] = b[i] + 1000.f;
    }

    float ncc_orig = normalized_cross_correlation(a, b);
    float ncc_offset = normalized_cross_correlation(a, b_offset);
    CHECK(ncc_orig == doctest::Approx(ncc_offset).epsilon(1e-4));
}

TEST_CASE("NCC: flat profiles return 0") {
    std::vector<float> flat = {5.f, 5.f, 5.f, 5.f, 5.f};
    std::vector<float> ramp = {1.f, 2.f, 3.f, 4.f, 5.f};

    float ncc = normalized_cross_correlation(flat, ramp);
    CHECK(ncc == doctest::Approx(0.f).epsilon(1e-5));
}

TEST_CASE("NCC: empty profiles return 0") {
    std::vector<float> empty;
    float ncc = normalized_cross_correlation(empty, empty);
    CHECK(ncc == doctest::Approx(0.f));
}

TEST_CASE("extract_dem_profile: flat DEM gives constant profile") {
    TerrainMap map;
    map.width = 50;
    map.height = 50;
    map.meters_per_pixel = 10.f;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.assign(50 * 50, 42.f);

    std::vector<TerrainSample> samples = {
        {.agl = 0.f, .baro_alt = 0.f, .dx = 0.f, .dy = 0.f},
        {.agl = 0.f, .baro_alt = 0.f, .dx = 10.f, .dy = 0.f},
        {.agl = 0.f, .baro_alt = 0.f, .dx = 10.f, .dy = 0.f},
    };

    auto profile = extract_dem_profile(map, 0.f, 0.f, samples);
    REQUIRE(profile.size() == 3);
    CHECK(profile[0] == doctest::Approx(42.f));
    CHECK(profile[1] == doctest::Approx(42.f));
    CHECK(profile[2] == doctest::Approx(42.f));
}

TEST_CASE("extract_dem_profile: ramp DEM gives linear profile") {
    auto map = makeRampDem(101, 10.f, 0.1f);  // slope = 0.1 m/m along X

    // Walk along X (North) from x=0
    std::vector<TerrainSample> samples;
    for (int i = 0; i < 10; ++i) {
        samples.push_back({
            .agl = 0.f, .baro_alt = 0.f,
            .dx = (i == 0) ? 0.f : 10.f,
            .dy = 0.f
        });
    }

    auto profile = extract_dem_profile(map, 0.f, 0.f, samples);
    REQUIRE(profile.size() == 10);

    // Profile should increase linearly: 0, 1, 2, 3, ...
    for (int i = 1; i < 10; ++i) {
        CHECK(profile[i] > profile[i - 1]);
    }
}

TEST_CASE("TerrainMap::height_at: center of map returns center elevation") {
    auto map = makeRampDem(101, 10.f, 0.1f);

    // Center is at (0, 0) — ramp = 0.1 * 0 = 0
    float h = map.height_at(0.f, 0.f);
    CHECK(h == doctest::Approx(0.f).epsilon(0.5f));
}

TEST_CASE("TerrainMap::height_at: positive X gives positive elevation on ramp") {
    auto map = makeRampDem(101, 10.f, 0.1f);

    float h = map.height_at(100.f, 0.f);
    CHECK(h == doctest::Approx(10.f).epsilon(1.0f));
}

TEST_CASE("tercom_match: finds correct position on sine DEM") {
    auto map = makeSineDem(201, 12.f);

    // True position: (120, 60)
    float true_x = 120.f;
    float true_y = 60.f;

    // Build samples at the true position (25 samples for strong discrimination)
    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;  // constant baro altitude
    for (int i = 0; i < 25; ++i) {
        float sx = true_x + static_cast<float>(i) * 12.f;
        float sy = true_y;
        float terrain_z = map.height_at(sx, sy);
        float agl = baro_alt - terrain_z;
        samples.push_back({
            .agl = agl, .baro_alt = baro_alt,
            .dx = (i == 0) ? 0.f : 12.f,
            .dy = 0.f
        });
    }

    TercomConfig config;
    config.min_samples = 10;
    config.search_radius = 200.f;
    config.search_step = 12.f;
    config.min_ncc = 0.5f;
    config.par_threshold = 1.01f;  // Low threshold: synthetic sine terrain has low PAR

    // Search centered on true position (should find exact match)
    auto result = tercom_match(map, samples, true_x, true_y, config);

    // Algorithm must find correct position with near-perfect NCC
    CHECK(result.ncc == doctest::Approx(1.0f).epsilon(0.01));
    CHECK(result.x == doctest::Approx(true_x).epsilon(13.f));
    CHECK(result.y == doctest::Approx(true_y).epsilon(13.f));
    CHECK(result.par > 1.0f);  // Best must be better than second-best
}

TEST_CASE("tercom_match: finds correct position with offset search center") {
    auto map = makeSineDem(201, 12.f);

    float true_x = 120.f;
    float true_y = 60.f;

    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;
    for (int i = 0; i < 25; ++i) {
        float sx = true_x + static_cast<float>(i) * 12.f;
        float sy = true_y;
        float terrain_z = map.height_at(sx, sy);
        float agl = baro_alt - terrain_z;
        samples.push_back({
            .agl = agl, .baro_alt = baro_alt,
            .dx = (i == 0) ? 0.f : 12.f,
            .dy = 0.f
        });
    }

    TercomConfig config;
    config.min_samples = 10;
    config.search_radius = 200.f;
    config.search_step = 12.f;
    config.min_ncc = 0.5f;
    config.par_threshold = 1.01f;

    // Search centered 96m away (multiple of search_step so true pos is on grid)
    auto result = tercom_match(map, samples, true_x + 96.f, true_y, config);

    CHECK(result.ncc > 0.8f);
    // Should find position within one search step of truth
    float err = std::sqrt((result.x - true_x) * (result.x - true_x) +
                          (result.y - true_y) * (result.y - true_y));
    CHECK(err < config.search_step * 2.f);
}

TEST_CASE("tercom_match: flat terrain gives invalid result (low PAR)") {
    TerrainMap map;
    map.width = 101;
    map.height = 101;
    map.meters_per_pixel = 12.f;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.assign(101 * 101, 100.f);  // Completely flat

    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;
    for (int i = 0; i < 15; ++i) {
        samples.push_back({
            .agl = 100.f, .baro_alt = baro_alt,
            .dx = (i == 0) ? 0.f : 12.f,
            .dy = 0.f
        });
    }

    TercomConfig config;
    config.min_samples = 10;
    config.par_threshold = 1.5f;

    auto result = tercom_match(map, samples, 0.f, 0.f, config);

    // Flat terrain: NCC is 0 for all candidates (no variance)
    CHECK_FALSE(result.valid);
}

TEST_CASE("tercom_match: too few samples gives invalid result") {
    auto map = makeSineDem(101, 12.f);

    std::vector<TerrainSample> samples = {
        {.agl = 100.f, .baro_alt = 200.f, .dx = 0.f, .dy = 0.f},
    };

    TercomConfig config;
    config.min_samples = 10;

    auto result = tercom_match(map, samples, 0.f, 0.f, config);
    CHECK_FALSE(result.valid);
}

// --- Anisotropic uncertainty tests ---

TEST_CASE("AnisotropicSigma: straight east path has larger var_yy than var_xx") {
    // DEM with terrain variation in both X and Y
    auto map = makeSineDem(201, 12.f);

    // Straight east path: samples vary only in Y
    float true_x = 0.f;
    float true_y = 0.f;

    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;
    for (int i = 0; i < 25; ++i) {
        float sx = true_x;
        float sy = true_y + static_cast<float>(i) * 12.f;
        float terrain_z = map.height_at(sx, sy);
        float agl = baro_alt - terrain_z;
        samples.push_back({
            .agl = agl, .baro_alt = baro_alt,
            .dx = 0.f,
            .dy = (i == 0) ? 0.f : 12.f
        });
    }

    TercomConfig config;
    config.min_samples = 10;
    config.search_radius = 200.f;
    config.search_step = 12.f;
    config.min_ncc = 0.5f;
    config.par_threshold = 1.01f;

    auto result = tercom_match(map, samples, true_x, true_y, config);

    CHECK(result.valid);
    // Straight east path: good along-track (Y) correlation, poor cross-track (X)
    // var_xx (cross-track, North) should be larger than var_yy (along-track, East)
    CHECK(result.var_xx > result.var_yy);
    // Both should be positive and finite
    CHECK(result.var_xx > 0.f);
    CHECK(result.var_yy > 0.f);
}

TEST_CASE("AnisotropicSigma: L-shaped path has comparable var_xx and var_yy") {
    auto map = makeSineDem(201, 12.f);

    float true_x = 0.f;
    float true_y = 0.f;

    // L-shaped path: 12 samples east, then 13 samples north
    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;
    float cx = true_x, cy = true_y;

    for (int i = 0; i < 25; ++i) {
        float dx_step, dy_step;
        if (i == 0) {
            dx_step = 0.f;
            dy_step = 0.f;
        } else if (i < 12) {
            dx_step = 0.f;
            dy_step = 12.f;
        } else {
            dx_step = 12.f;
            dy_step = 0.f;
        }

        // Apply displacement BEFORE reading terrain (matches extract_dem_profile)
        if (i > 0) {
            cx += dx_step;
            cy += dy_step;
        }
        float terrain_z = map.height_at(cx, cy);
        float agl = baro_alt - terrain_z;

        samples.push_back({
            .agl = agl, .baro_alt = baro_alt,
            .dx = dx_step, .dy = dy_step
        });
    }

    TercomConfig config;
    config.min_samples = 10;
    config.search_radius = 200.f;
    config.search_step = 12.f;
    config.min_ncc = 0.5f;
    config.par_threshold = 1.01f;

    auto result = tercom_match(map, samples, true_x, true_y, config);

    CHECK(result.valid);
    // L-path provides information in both directions
    // var_xx and var_yy should be within 10x of each other (both reasonable)
    float ratio = result.var_xx / result.var_yy;
    CHECK(ratio > 0.1f);
    CHECK(ratio < 10.f);
}

// --- Coarse-to-fine search tests ---

TEST_CASE("CoarseToFine: matches exhaustive search within tolerance") {
    auto map = makeSineDem(201, 12.f);

    float true_x = 120.f;
    float true_y = 60.f;

    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;
    for (int i = 0; i < 25; ++i) {
        float sx = true_x + static_cast<float>(i) * 12.f;
        float sy = true_y;
        float terrain_z = map.height_at(sx, sy);
        float agl = baro_alt - terrain_z;
        samples.push_back({
            .agl = agl, .baro_alt = baro_alt,
            .dx = (i == 0) ? 0.f : 12.f,
            .dy = 0.f
        });
    }

    // Coarse-to-fine (default: coarse_factor=2, refine_top_n=3)
    TercomConfig config;
    config.min_samples = 10;
    config.search_radius = 200.f;
    config.search_step = 12.f;
    config.min_ncc = 0.5f;
    config.par_threshold = 1.01f;

    auto result = tercom_match(map, samples, true_x + 96.f, true_y, config);

    CHECK(result.valid);
    float err = std::sqrt((result.x - true_x) * (result.x - true_x) +
                          (result.y - true_y) * (result.y - true_y));
    // Should find correct position within 2 search steps
    CHECK(err < config.search_step * 2.f);
}

TEST_CASE("CoarseToFine: larger radius still finds correct position") {
    // Create DEM with incommensurate frequencies to avoid periodic ambiguity
    // at the 600m search radius. Standard makeSineDem repeats every ~126m.
    TerrainMap map;
    constexpr int sz = 301;
    constexpr float mpp = 12.f;
    map.width = sz;
    map.height = sz;
    map.meters_per_pixel = mpp;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.resize(sz * sz);
    float half = static_cast<float>(sz) / 2.f;
    for (int r = 0; r < sz; ++r) {
        for (int c = 0; c < sz; ++c) {
            float x = -(static_cast<float>(r) - half) * mpp;
            float y = (static_cast<float>(c) - half) * mpp;
            // Incommensurate frequencies: periods ≈ 170m, 88m, 250m, 130m
            // Cross-terms make each (x,y) position unique within ±1000m.
            map.elevation[r * sz + c] =
                10.f * std::sin(x * 0.037f) +
                7.f * std::sin(y * 0.071f) +
                5.f * std::cos(x * 0.025f + y * 0.048f) +
                3.f * std::sin(x * 0.011f - y * 0.019f);
        }
    }

    float true_x = 120.f;
    float true_y = 60.f;

    // L-shaped path: 15 samples east, then 15 samples north
    std::vector<TerrainSample> samples;
    float baro_alt = 200.f;
    float cx = true_x, cy = true_y;
    for (int i = 0; i < 30; ++i) {
        float dx_step, dy_step;
        if (i == 0) { dx_step = 0.f; dy_step = 0.f; }
        else if (i < 15) { dx_step = 0.f; dy_step = 12.f; }
        else { dx_step = 12.f; dy_step = 0.f; }

        // Apply displacement BEFORE reading terrain (matches extract_dem_profile)
        if (i > 0) {
            cx += dx_step;
            cy += dy_step;
        }
        float terrain_z = map.height_at(cx, cy);
        float agl = baro_alt - terrain_z;
        samples.push_back({
            .agl = agl, .baro_alt = baro_alt,
            .dx = dx_step, .dy = dy_step
        });
    }

    TercomConfig config;
    config.min_samples = 10;
    config.search_radius = 600.f;  // Larger search radius
    config.search_step = 12.f;
    config.min_ncc = 0.5f;
    config.par_threshold = 1.01f;

    auto result = tercom_match(map, samples, true_x, true_y, config);

    CHECK(result.ncc > 0.9f);
    float err = std::sqrt((result.x - true_x) * (result.x - true_x) +
                          (result.y - true_y) * (result.y - true_y));
    CHECK(err < config.search_step * 2.f);
}

// --- Terrain discriminability tests ---

/// Helper: create a DEM where profile SHAPE varies with cross-track (X/North).
/// For East-heading flight: cross-track = North (X), so different cross-track
/// offsets see different profile shapes → high discriminability.
/// Uses X*Y interaction so NCC (which removes mean) still detects differences.
static TerrainMap makeCrossTrackRidgeDem(int size, float mpp) {
    TerrainMap map;
    map.width = size;
    map.height = size;
    map.meters_per_pixel = mpp;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.resize(size * size);

    float half = static_cast<float>(size) / 2.f;
    for (int r = 0; r < size; ++r) {
        for (int c = 0; c < size; ++c) {
            float x = -(static_cast<float>(r) - half) * mpp;
            float y = (static_cast<float>(c) - half) * mpp;
            // Multi-frequency terrain with incommensurate plane waves.
            // NCC is invariant to mean shifts and amplitude scaling, so we need
            // genuinely different profile SHAPES at different cross-track offsets.
            // Multiple plane waves at different angles ensure no periodic aliases.
            map.elevation[r * size + c] =
                10.f * std::sin(0.5f  * x + 0.13f * y) +
                 7.f * std::sin(0.71f * x - 0.09f * y + 1.f) +
                 5.f * std::sin(0.37f * x + 0.19f * y + 2.3f) +
                 3.f * std::sin(0.89f * x - 0.17f * y + 0.7f);
        }
    }
    return map;
}

/// Helper: create a DEM with elevation varying in Y (East) only.
/// For East-heading flight: along-track = East (Y), so all cross-track offsets
/// see the same elevation profile → low discriminability.
static TerrainMap makeAlongTrackRidgeDem(int size, float mpp) {
    TerrainMap map;
    map.width = size;
    map.height = size;
    map.meters_per_pixel = mpp;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.resize(size * size);

    float half = static_cast<float>(size) / 2.f;
    for (int r = 0; r < size; ++r) {
        for (int c = 0; c < size; ++c) {
            float y = (static_cast<float>(c) - half) * mpp;
            // Elevation varies with Y (East) only — same profile at any cross-track offset
            map.elevation[r * size + c] = 20.f * std::sin(y * 0.05f);
        }
    }
    return map;
}

TEST_CASE("Discriminability: flat terrain gives low discriminability") {
    TerrainMap map;
    map.width = 101;
    map.height = 101;
    map.meters_per_pixel = 12.f;
    map.origin_x = 0.f;
    map.origin_y = 0.f;
    map.elevation.assign(101 * 101, 100.f);  // Completely flat

    // Single leg heading East: (0,0) → (0,400)
    std::vector<std::pair<float, float>> waypoints = {{0.f, 400.f}};

    auto legs = compute_mission_discriminability(map, waypoints, 10, 12.f, 100.f, 50.f);

    REQUIRE(legs.size() == 1);
    // Flat terrain: all cross-track profiles identical → disc ≈ 0
    for (float score : legs[0].disc_scores) {
        CHECK(score < 0.1f);
    }
}

TEST_CASE("Discriminability: cross-track ridge gives high discriminability") {
    auto map = makeCrossTrackRidgeDem(201, 12.f);

    // Leg heading East along ridge axis: (0,0) → (0,400)
    std::vector<std::pair<float, float>> waypoints = {{0.f, 400.f}};

    auto legs = compute_mission_discriminability(map, waypoints, 10, 12.f, 100.f, 50.f);

    REQUIRE(legs.size() == 1);
    // Cross-track ridge: profiles at different cross-track offsets differ → high disc
    float avg_disc = 0.f;
    for (float score : legs[0].disc_scores) {
        avg_disc += score;
    }
    avg_disc /= static_cast<float>(legs[0].disc_scores.size());
    // NCC is conservative: at 12m resolution, nearest offset profiles are always
    // moderately similar. 0.22 is typical for rich synthetic terrain. Real DEMs
    // with sharp features (canyons, ridges) score higher.
    CHECK(avg_disc > 0.15f);
}

TEST_CASE("Discriminability: along-track ridge gives low discriminability for east heading") {
    auto map = makeAlongTrackRidgeDem(201, 12.f);

    // Leg heading East: (0,0) → (0,400)
    // Ridge is along North (X) — along-track profiles at cross-track offsets are similar
    std::vector<std::pair<float, float>> waypoints = {{0.f, 400.f}};

    auto legs = compute_mission_discriminability(map, waypoints, 10, 12.f, 100.f, 50.f);

    REQUIRE(legs.size() == 1);
    // Along-track ridge is uniform in cross-track → low disc
    float avg_disc = 0.f;
    for (float score : legs[0].disc_scores) {
        avg_disc += score;
    }
    avg_disc /= static_cast<float>(legs[0].disc_scores.size());
    CHECK(avg_disc < 0.3f);
}

TEST_CASE("ProjectOntoLegs: correct projection geometry") {
    // Create two legs: East then North (L-shaped)
    MissionLeg leg1;
    leg1.start_x = 0.f; leg1.start_y = 0.f;
    leg1.end_x = 0.f; leg1.end_y = 400.f;
    leg1.length = 400.f;
    leg1.heading = static_cast<float>(M_PI / 2.0);  // East
    leg1.along_x = 0.f; leg1.along_y = 1.f;
    leg1.cross_x = -1.f; leg1.cross_y = 0.f;   // -sin(pi/2), cos(pi/2)
    leg1.disc_scores = {0.1f, 0.5f, 0.8f, 0.5f, 0.2f};
    leg1.disc_step = 100.f;

    MissionLeg leg2;
    leg2.start_x = 0.f; leg2.start_y = 400.f;
    leg2.end_x = 400.f; leg2.end_y = 400.f;
    leg2.length = 400.f;
    leg2.heading = 0.f;  // North
    leg2.along_x = 1.f; leg2.along_y = 0.f;
    leg2.cross_x = 0.f; leg2.cross_y = 1.f;
    leg2.disc_scores = {0.3f, 0.6f, 0.9f, 0.6f, 0.3f};
    leg2.disc_step = 100.f;

    std::vector<MissionLeg> legs = {leg1, leg2};

    // Point on leg 1 at y=200, with 50m cross-track offset (North)
    auto proj1 = project_onto_legs(50.f, 200.f, legs, 200.f);
    CHECK(proj1.leg_index == 0);
    CHECK(proj1.along_track == doctest::Approx(200.f).epsilon(1.f));
    CHECK(proj1.cross_track == doctest::Approx(-50.f).epsilon(1.f));  // North = -cross for East heading
    CHECK(proj1.discriminability > 0.f);  // interpolated from disc_scores

    // Point near leg 2 at x=200, y=400
    auto proj2 = project_onto_legs(200.f, 400.f, legs, 200.f);
    CHECK(proj2.leg_index == 1);
    CHECK(proj2.along_track == doctest::Approx(200.f).epsilon(1.f));
    CHECK(std::abs(proj2.cross_track) < 1.f);  // on the leg

    // Point far from any leg
    auto proj_far = project_onto_legs(1000.f, 1000.f, legs, 200.f);
    CHECK(proj_far.leg_index == -1);  // rejected — too far
}
