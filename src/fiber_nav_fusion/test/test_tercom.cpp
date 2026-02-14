#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <fiber_nav_fusion/tercom.hpp>

#include <cmath>
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
