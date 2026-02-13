#include <fiber_nav_fusion/tercom.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace fiber_nav_fusion {

float TerrainMap::heightAt(float x, float y) const {
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

std::vector<float> buildProfile(std::span<const TerrainSample> samples) {
    std::vector<float> profile;
    profile.reserve(samples.size());
    for (const auto& s : samples) {
        profile.push_back(s.baro_alt - s.agl);
    }
    return profile;
}

std::vector<float> extractDemProfile(
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
        profile.push_back(map.heightAt(cx, cy));
    }
    return profile;
}

float normalizedCrossCorrelation(
    std::span<const float> measured,
    std::span<const float> reference)
{
    if (measured.size() != reference.size() || measured.empty()) {
        return 0.f;
    }

    auto n = static_cast<float>(measured.size());

    // Compute means
    float mean_m = 0.f, mean_r = 0.f;
    for (std::size_t i = 0; i < measured.size(); ++i) {
        mean_m += measured[i];
        mean_r += reference[i];
    }
    mean_m /= n;
    mean_r /= n;

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

TercomResult tercomMatch(
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
    auto measured = buildProfile(samples);

    float best_ncc = -2.f;
    float second_ncc = -2.f;
    float best_x = est_x;
    float best_y = est_y;

    int steps = static_cast<int>(config.search_radius / config.search_step);

    for (int ix = -steps; ix <= steps; ++ix) {
        for (int iy = -steps; iy <= steps; ++iy) {
            float cx = est_x + static_cast<float>(ix) * config.search_step;
            float cy = est_y + static_cast<float>(iy) * config.search_step;

            auto ref = extractDemProfile(map, cx, cy, samples);
            float ncc = normalizedCrossCorrelation(measured, ref);

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

    return result;
}

}  // namespace fiber_nav_fusion
