#include <fiber_nav_fusion/tercom.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>

// stb_image for loading heightmap PNG
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include "stb_image.h"

namespace fiber_nav_fusion {

float TerrainMap::height_at(float x, float y) const {
    if (elevation.empty() || width <= 0 || height <= 0 || meters_per_pixel <= 0.f) {
        return 0.f;
    }

    // Convert NED (x=North, y=East) to pixel coordinates.
    // DEM row 0 = North edge, col 0 = West edge.
    // center pixel = (height/2, width/2)
    float col_f = (y - origin_y) / meters_per_pixel + static_cast<float>(width) / 2.f;
    float row_f = -(x - origin_x) / meters_per_pixel + static_cast<float>(height) / 2.f;

    // Clamp to valid range
    col_f = std::clamp(col_f, 0.f, static_cast<float>(width - 1));
    row_f = std::clamp(row_f, 0.f, static_cast<float>(height - 1));

    int c0 = static_cast<int>(col_f);
    int r0 = static_cast<int>(row_f);
    int c1 = std::min(c0 + 1, width - 1);
    int r1 = std::min(r0 + 1, height - 1);

    float fc = col_f - static_cast<float>(c0);
    float fr = row_f - static_cast<float>(r0);

    // Bilinear interpolation
    float v00 = elevation[r0 * width + c0];
    float v01 = elevation[r0 * width + c1];
    float v10 = elevation[r1 * width + c0];
    float v11 = elevation[r1 * width + c1];

    return v00 * (1.f - fc) * (1.f - fr) +
           v01 * fc * (1.f - fr) +
           v10 * (1.f - fc) * fr +
           v11 * fc * fr;
}

std::vector<float> build_profile(std::span<const TerrainSample> samples) {
    std::vector<float> profile(samples.size());
    std::transform(samples.begin(), samples.end(), profile.begin(),
                   [](const auto& s) { return s.baro_alt - s.agl; });
    return profile;
}

std::vector<float> extract_dem_profile(
    const TerrainMap& map,
    float x0, float y0,
    std::span<const TerrainSample> samples)
{
    std::vector<float> profile;
    profile.reserve(samples.size());

    float cx = x0;
    float cy = y0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (i > 0) {
            cx += samples[i].dx;
            cy += samples[i].dy;
        }
        profile.push_back(map.height_at(cx, cy));
    }
    return profile;
}

float normalized_cross_correlation(
    std::span<const float> measured,
    std::span<const float> reference)
{
    if (measured.size() != reference.size() || measured.empty()) {
        return 0.f;
    }

    auto n = static_cast<float>(measured.size());

    // Compute means
    float mean_m = std::accumulate(measured.begin(), measured.end(), 0.f) / n;
    float mean_r = std::accumulate(reference.begin(), reference.end(), 0.f) / n;

    // Compute NCC: sum((m-mean_m)*(r-mean_r)) / sqrt(sum((m-mean_m)^2) * sum((r-mean_r)^2))
    float num = 0.f, den_m = 0.f, den_r = 0.f;
    for (std::size_t i = 0; i < measured.size(); ++i) {
        float dm = measured[i] - mean_m;
        float dr = reference[i] - mean_r;
        num += dm * dr;
        den_m += dm * dm;
        den_r += dr * dr;
    }

    float den = std::sqrt(den_m * den_r);
    if (den < 1e-10f) {
        return 0.f;  // flat profiles — no meaningful correlation
    }

    return num / den;
}

TercomResult tercom_match(
    const TerrainMap& map,
    std::span<const TerrainSample> samples,
    float est_x, float est_y,
    const TercomConfig& config)
{
    TercomResult result;

    if (static_cast<int>(samples.size()) < config.min_samples) {
        return result;
    }

    // Build measured terrain profile
    auto measured = build_profile(samples);

    // --- Phase 1: Coarse search ---
    float coarse_step = config.search_step * config.coarse_factor;
    int coarse_steps = static_cast<int>(config.search_radius / coarse_step);

    // Store top-N candidates from coarse pass
    struct Candidate {
        float x, y, ncc;
    };
    std::vector<Candidate> top_candidates(config.refine_top_n, {0.f, 0.f, -2.f});

    float best_ncc = -2.f;
    float second_ncc = -2.f;
    float best_x = est_x;
    float best_y = est_y;

    for (int ix = -coarse_steps; ix <= coarse_steps; ++ix) {
        for (int iy = -coarse_steps; iy <= coarse_steps; ++iy) {
            float cx = est_x + static_cast<float>(ix) * coarse_step;
            float cy = est_y + static_cast<float>(iy) * coarse_step;

            auto ref = extract_dem_profile(map, cx, cy, samples);
            float ncc = normalized_cross_correlation(measured, ref);

            // Maintain top-N candidates for refinement
            for (auto& c : top_candidates) {
                if (ncc > c.ncc) {
                    // Shift: demote this candidate, insert new one
                    // (simple insertion into sorted-ish list)
                    Candidate tmp = {cx, cy, ncc};
                    std::swap(c, tmp);
                    // Push displaced candidate down
                    for (std::size_t j = (&c - top_candidates.data()) + 1;
                         j < top_candidates.size(); ++j) {
                        if (tmp.ncc > top_candidates[j].ncc) {
                            std::swap(tmp, top_candidates[j]);
                        }
                    }
                    break;
                }
            }
        }
    }

    // --- Phase 2: Fine refinement around top-N candidates ---
    int refine_radius = 2;  // ±2 fine steps around each candidate

    for (const auto& cand : top_candidates) {
        if (cand.ncc <= -2.f) continue;  // unused slot

        for (int ix = -refine_radius; ix <= refine_radius; ++ix) {
            for (int iy = -refine_radius; iy <= refine_radius; ++iy) {
                float cx = cand.x + static_cast<float>(ix) * config.search_step;
                float cy = cand.y + static_cast<float>(iy) * config.search_step;

                auto ref = extract_dem_profile(map, cx, cy, samples);
                float ncc = normalized_cross_correlation(measured, ref);

                if (ncc > best_ncc) {
                    second_ncc = best_ncc;
                    best_ncc = ncc;
                    best_x = cx;
                    best_y = cy;
                } else if (ncc > second_ncc) {
                    second_ncc = ncc;
                }
            }
        }
    }

    result.x = best_x;
    result.y = best_y;
    result.ncc = best_ncc;

    // Peak ambiguity ratio
    if (second_ncc > 0.f) {
        result.par = best_ncc / second_ncc;
    } else if (best_ncc > 0.f) {
        result.par = 10.f;  // Only one positive correlation — very unambiguous
    } else {
        result.par = 0.f;
    }

    result.sigma = (result.par > 0.f)
        ? config.par_sigma_scale / result.par
        : config.search_radius;

    result.valid = (result.ncc >= config.min_ncc) && (result.par >= config.par_threshold);

    // --- Anisotropic uncertainty from NCC Hessian at peak ---
    if (result.valid) {
        float step = config.search_step;
        float peak = best_ncc;

        // NCC at 4 neighbors
        auto ref_xp = extract_dem_profile(map, best_x + step, best_y, samples);
        auto ref_xm = extract_dem_profile(map, best_x - step, best_y, samples);
        auto ref_yp = extract_dem_profile(map, best_x, best_y + step, samples);
        auto ref_ym = extract_dem_profile(map, best_x, best_y - step, samples);

        float ncc_xp = normalized_cross_correlation(measured, ref_xp);
        float ncc_xm = normalized_cross_correlation(measured, ref_xm);
        float ncc_yp = normalized_cross_correlation(measured, ref_yp);
        float ncc_ym = normalized_cross_correlation(measured, ref_ym);

        // Second derivatives (curvature of NCC surface)
        float step_sq = step * step;
        float d2_xx = (ncc_xp - 2.f * peak + ncc_xm) / step_sq;
        float d2_yy = (ncc_yp - 2.f * peak + ncc_ym) / step_sq;

        // Convert curvature to variance: sharper curvature = lower variance
        constexpr float epsilon = 1e-6f;
        float abs_d2_xx = std::max(std::abs(d2_xx), epsilon);
        float abs_d2_yy = std::max(std::abs(d2_yy), epsilon);

        result.var_xx = config.hessian_variance_scale / abs_d2_xx;
        result.var_yy = config.hessian_variance_scale / abs_d2_yy;
        result.var_xy = 0.f;  // Grid-aligned, no rotation needed

        // Clamp to [step², search_radius²]
        float min_var = step_sq;
        float max_var = config.search_radius * config.search_radius;
        result.var_xx = std::clamp(result.var_xx, min_var, max_var);
        result.var_yy = std::clamp(result.var_yy, min_var, max_var);
    } else {
        // Invalid match: set large isotropic uncertainty
        float fallback = config.search_radius * config.search_radius;
        result.var_xx = fallback;
        result.var_yy = fallback;
        result.var_xy = 0.f;
    }

    return result;
}

TerrainMap load_terrain_map(const std::string& terrain_data_path,
                          std::function<void(const std::string&)> log_fn)
{
    auto log = [&](const std::string& msg) {
        if (log_fn) log_fn(msg);
    };

    TerrainMap map;
    std::string path = terrain_data_path;

    // Auto-discover terrain_data.json
    if (path.empty()) {
        std::vector<std::string> candidates = {
            "/root/ws/src/fiber-nav-sim/src/fiber_nav_gazebo/terrain/terrain_data.json",
        };
        try {
            auto exe = std::filesystem::read_symlink("/proc/self/exe");
            auto ws = exe.parent_path().parent_path().parent_path().parent_path();
            candidates.push_back(
                (ws / "src" / "fiber_nav_gazebo" / "terrain" / "terrain_data.json").string());
        } catch (...) {}

        for (const auto& c : candidates) {
            if (std::filesystem::exists(c)) {
                path = c;
                break;
            }
        }
    }

    if (path.empty() || !std::filesystem::exists(path)) {
        log("Cannot find terrain_data.json");
        return map;
    }

    auto terrain_dir = std::filesystem::path(path).parent_path();

    // Parse JSON (minimal parser — extract fields we need)
    std::ifstream file(path);
    std::string json_str((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());

    auto get_float = [&](const std::string& key) -> float {
        auto pos = json_str.find("\"" + key + "\"");
        if (pos == std::string::npos) return 0.f;
        pos = json_str.find(':', pos);
        if (pos == std::string::npos) return 0.f;
        return std::stof(json_str.substr(pos + 1));
    };
    auto get_int = [&](const std::string& key) -> int {
        auto pos = json_str.find("\"" + key + "\"");
        if (pos == std::string::npos) return 0;
        pos = json_str.find(':', pos);
        if (pos == std::string::npos) return 0;
        return std::stoi(json_str.substr(pos + 1));
    };
    auto get_string = [&](const std::string& key) -> std::string {
        auto pos = json_str.find("\"" + key + "\"");
        if (pos == std::string::npos) return "";
        pos = json_str.find('"', pos + key.size() + 2);
        if (pos == std::string::npos) return "";
        auto end = json_str.find('"', pos + 1);
        return json_str.substr(pos + 1, end - pos - 1);
    };

    float elev_range = get_float("heightmap_range_m");
    float mpp = get_float("meters_per_pixel");
    int res = get_int("resolution_px");
    std::string hm_file = get_string("heightmap_file");

    if (res <= 0 || mpp <= 0.f || hm_file.empty()) {
        log("Invalid terrain_data.json");
        return map;
    }

    // Load heightmap PNG
    auto hm_path = (terrain_dir / hm_file).string();
    std::ifstream hm_stream(hm_path, std::ios::binary);
    if (!hm_stream) {
        log("Cannot open heightmap: " + hm_path);
        return map;
    }
    std::vector<uint8_t> hm_data((std::istreambuf_iterator<char>(hm_stream)),
                                  std::istreambuf_iterator<char>());

    int w, h, channels;

    // Try 16-bit first
    auto* pixels16 = stbi_load_16_from_memory(
        hm_data.data(), static_cast<int>(hm_data.size()),
        &w, &h, &channels, 1);

    if (pixels16) {
        map.width = w;
        map.height = h;
        map.meters_per_pixel = mpp;
        map.origin_x = 0.f;
        map.origin_y = 0.f;
        map.elevation.resize(w * h);
        for (int i = 0; i < w * h; ++i) {
            map.elevation[i] =
                (static_cast<float>(pixels16[i]) / 65535.f) * elev_range;
        }
        stbi_image_free(pixels16);
    } else {
        // Fallback: 8-bit
        auto* pixels = stbi_load_from_memory(
            hm_data.data(), static_cast<int>(hm_data.size()),
            &w, &h, &channels, 1);
        if (!pixels) {
            log("Failed to load heightmap PNG: " + hm_path);
            return map;
        }
        map.width = w;
        map.height = h;
        map.meters_per_pixel = mpp;
        map.origin_x = 0.f;
        map.origin_y = 0.f;
        map.elevation.resize(w * h);
        for (int i = 0; i < w * h; ++i) {
            map.elevation[i] =
                (static_cast<float>(pixels[i]) / 255.f) * elev_range;
        }
        stbi_image_free(pixels);
    }

    log("Loaded DEM: " + std::to_string(w) + "x" + std::to_string(h) +
        ", " + std::to_string(mpp) + " m/px");
    return map;
}

}  // namespace fiber_nav_fusion
