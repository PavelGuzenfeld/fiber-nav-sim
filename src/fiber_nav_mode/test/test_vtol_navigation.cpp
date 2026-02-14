#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Core>

// Test pure geometry and config logic without px4_ros2 dependencies.
// Re-define the structs and free functions locally (same pattern as test_canyon_waypoints.cpp).

namespace fiber_nav_mode {

struct CableMonitorConfig
{
    bool enabled = false;
    float tension_warn_percent = 70.f;
    float tension_abort_percent = 85.f;
    float breaking_strength = 50.f;
    float spool_capacity = -1.f;
    float spool_warn_percent = 80.f;
    float spool_abort_percent = 95.f;
};

struct GpsDeniedConfig
{
    bool enabled = false;
    float wp_time_s = 30.f;
    float return_time_s = 120.f;
    float descent_time_s = 60.f;
    float altitude_kp = 0.5f;
    float altitude_max_vz = 3.f;
    float fw_speed = 18.f;
    float return_heading = NAN;
    bool use_position_ekf = false;
    float ekf_wp_accept_radius = 80.f;
    float ekf_home_accept_radius = 100.f;
    float ekf_max_uncertainty = 200.f;
};

struct VtolNavConfig
{
    float cruise_alt_m = 50.f;
    float climb_rate = 2.f;
    float fw_accept_radius = 50.f;
    float mc_transition_dist = 200.f;
    float mc_approach_speed = 5.f;
    float fw_transition_timeout = 30.f;
    float mc_transition_timeout = 60.f;
    CableMonitorConfig cable_monitor;
    GpsDeniedConfig gps_denied;
};

enum class State
{
    McClimb,
    TransitionFw,
    FwNavigate,
    FwReturn,
    TransitionMc,
    McApproach,
    Done,
};

/// Replicate the cable tension check logic from VtolNavigationMode.
/// Returns the new state if a transition should happen, or std::nullopt if no change.
struct CableTensionResult
{
    bool should_transition = false;
    State new_state{State::Done};
    bool is_warning = false;
};

CableTensionResult checkCableTensionLogic(
    const CableMonitorConfig& cfg, State current_state,
    float tension, bool is_broken, bool already_aborted)
{
    CableTensionResult result;

    if (!cfg.enabled) return result;

    const float breaking = cfg.breaking_strength;

    if (is_broken && !already_aborted) {
        result.should_transition = true;
        result.new_state = State::McApproach;
        return result;
    }

    const float abort_threshold = breaking * cfg.tension_abort_percent / 100.f;
    const float warn_threshold = breaking * cfg.tension_warn_percent / 100.f;

    if (tension > abort_threshold && !already_aborted) {
        result.should_transition = true;
        switch (current_state) {
        case State::FwNavigate:
            result.new_state = State::FwReturn;
            break;
        case State::McClimb:
        case State::TransitionFw:
            result.new_state = State::McApproach;
            break;
        default:
            result.should_transition = false;  // already returning
            break;
        }
        return result;
    }

    if (tension > warn_threshold && tension <= abort_threshold) {
        result.is_warning = true;
    }

    return result;
};

/// Replicate the spool exhaustion check logic from VtolNavigationMode.
CableTensionResult checkSpoolExhaustionLogic(
    const CableMonitorConfig& cfg, State current_state,
    float deployed_length, bool already_aborted)
{
    CableTensionResult result;

    if (!cfg.enabled || cfg.spool_capacity <= 0.f) return result;

    const float deployed_percent = (deployed_length / cfg.spool_capacity) * 100.f;

    if (deployed_percent >= cfg.spool_abort_percent && !already_aborted) {
        result.should_transition = true;
        switch (current_state) {
        case State::FwNavigate:
            result.new_state = State::FwReturn;
            break;
        case State::McClimb:
        case State::TransitionFw:
            result.new_state = State::McApproach;
            break;
        default:
            result.should_transition = false;
            break;
        }
        return result;
    }

    if (deployed_percent >= cfg.spool_warn_percent && deployed_percent < cfg.spool_abort_percent) {
        result.is_warning = true;
    }

    return result;
};

struct VtolWaypoint
{
    float x;
    float y;
    float heading;
    float acceptance_radius;
};

// Replicate the static helper functions from VtolNavigationMode
float courseToTarget(const Eigen::Vector3f& pos, float target_x, float target_y)
{
    return std::atan2(target_y - pos.y(), target_x - pos.x());
}

float horizontalDistTo(const Eigen::Vector3f& pos, float target_x, float target_y)
{
    const float dx = target_x - pos.x();
    const float dy = target_y - pos.y();
    return std::sqrt(dx * dx + dy * dy);
}

/// Compute fixed heading for each waypoint leg from waypoint geometry.
std::vector<float> computeLegHeadingsFromWaypoints(
    const std::vector<VtolWaypoint>& waypoints)
{
    std::vector<float> headings;
    headings.reserve(waypoints.size());
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        const float from_x = (i == 0) ? 0.f : waypoints[i - 1].x;
        const float from_y = (i == 0) ? 0.f : waypoints[i - 1].y;
        headings.push_back(
            std::atan2(waypoints[i].y - from_y, waypoints[i].x - from_x));
    }
    return headings;
}

/// Compute return heading from last waypoint back to home (0,0).
float computeReturnHeading(
    const std::vector<VtolWaypoint>& waypoints, float config_heading)
{
    if (!std::isnan(config_heading)) {
        return config_heading;
    }
    if (waypoints.empty()) {
        return 0.f;
    }
    const auto& last = waypoints.back();
    return std::atan2(-last.y, -last.x);
}

/// Altitude P-controller for GPS-denied flight.
float altitudeHoldVz(float cruise_alt_m, float current_agl,
                     float kp, float max_vz)
{
    const float error = cruise_alt_m - current_agl;
    return std::clamp(error * kp, -max_vz, max_vz);
}

/// Replicate drPosition() logic from VtolNavigationMode.
/// Returns position estimate if EKF is enabled, valid, and uncertainty below threshold.
std::optional<Eigen::Vector2f> drPosition(
    const GpsDeniedConfig& cfg,
    bool ekf_valid, float ekf_x, float ekf_y,
    float sigma_x, float sigma_y)
{
    if (!cfg.use_position_ekf || !ekf_valid) {
        return std::nullopt;
    }
    float max_sigma = std::max(sigma_x, sigma_y);
    if (max_sigma > cfg.ekf_max_uncertainty) {
        return std::nullopt;
    }
    return Eigen::Vector2f{ekf_x, ekf_y};
}

struct GimbalAccommodationConfig
{
    bool enabled = false;
    float saturation_threshold = 0.7f;
    float base_turn_rate = 5.f;
    float min_turn_rate = 1.f;
};

float angleDiff(float a, float b)
{
    float d = a - b;
    while (d > static_cast<float>(M_PI)) d -= 2.f * static_cast<float>(M_PI);
    while (d < -static_cast<float>(M_PI)) d += 2.f * static_cast<float>(M_PI);
    return d;
}

float rateLimitedCourse(float current_course, float target_course,
                        float max_rate_deg_s, float dt_s)
{
    const float max_delta = max_rate_deg_s * static_cast<float>(M_PI) / 180.f * dt_s;
    const float diff = angleDiff(target_course, current_course);
    const float delta = std::clamp(diff, -max_delta, max_delta);
    float result = current_course + delta;
    while (result > static_cast<float>(M_PI)) result -= 2.f * static_cast<float>(M_PI);
    while (result < -static_cast<float>(M_PI)) result += 2.f * static_cast<float>(M_PI);
    return result;
}

float effectiveTurnRate(float saturation, float threshold,
                        float base_rate, float min_rate)
{
    if (saturation <= threshold) {
        return base_rate;
    }
    const float t = std::clamp(
        (saturation - threshold) / (1.f - threshold), 0.f, 1.f);
    return base_rate + t * (min_rate - base_rate);
}

float effectiveAltRateScale(float pitch_saturation, float threshold,
                             float min_scale)
{
    if (pitch_saturation <= threshold) {
        return 1.f;
    }
    const float t = std::clamp(
        (pitch_saturation - threshold) / (1.f - threshold), 0.f, 1.f);
    return std::lerp(1.f, min_scale, t);
}

}  // namespace fiber_nav_mode

using fiber_nav_mode::VtolNavConfig;
using fiber_nav_mode::GpsDeniedConfig;
using fiber_nav_mode::VtolWaypoint;
using fiber_nav_mode::CableMonitorConfig;
using fiber_nav_mode::GimbalAccommodationConfig;
using fiber_nav_mode::State;
using fiber_nav_mode::CableTensionResult;
using fiber_nav_mode::checkCableTensionLogic;
using fiber_nav_mode::checkSpoolExhaustionLogic;
using fiber_nav_mode::courseToTarget;
using fiber_nav_mode::horizontalDistTo;
using fiber_nav_mode::computeLegHeadingsFromWaypoints;
using fiber_nav_mode::computeReturnHeading;
using fiber_nav_mode::altitudeHoldVz;
using fiber_nav_mode::drPosition;
using fiber_nav_mode::angleDiff;
using fiber_nav_mode::rateLimitedCourse;
using fiber_nav_mode::effectiveTurnRate;
using fiber_nav_mode::effectiveAltRateScale;

// --- Course angle tests ---

TEST_CASE("VtolCourse.NorthIsZero")
{
    // Target directly north (positive x in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 100.f, 0.f);
    CHECK(course == doctest::Approx(0.f).epsilon(0.01));
}

TEST_CASE("VtolCourse.EastIsHalfPi")
{
    // Target directly east (positive y in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 0.f, 100.f);
    CHECK(course == doctest::Approx(static_cast<float>(M_PI / 2.0)).epsilon(0.01));
}

TEST_CASE("VtolCourse.SouthIsPi")
{
    // Target directly south (negative x in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, -100.f, 0.f);
    // atan2(0, -100) = pi
    CHECK(std::abs(course) == doctest::Approx(static_cast<float>(M_PI)).epsilon(0.01));
}

TEST_CASE("VtolCourse.WestIsNegativeHalfPi")
{
    // Target directly west (negative y in NED)
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 0.f, -100.f);
    CHECK(course == doctest::Approx(static_cast<float>(-M_PI / 2.0)).epsilon(0.01));
}

TEST_CASE("VtolCourse.DiagonalNorthEast")
{
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float course = courseToTarget(pos, 100.f, 100.f);
    CHECK(course == doctest::Approx(static_cast<float>(M_PI / 4.0)).epsilon(0.01));
}

TEST_CASE("VtolCourse.FromNonOriginPosition")
{
    // Vehicle at (500, 300), target at (500, 400) = due east
    Eigen::Vector3f pos{500.f, 300.f, -50.f};
    float course = courseToTarget(pos, 500.f, 400.f);
    CHECK(course == doctest::Approx(static_cast<float>(M_PI / 2.0)).epsilon(0.01));
}

TEST_CASE("VtolCourse.CourseToHomeFromFar")
{
    // Vehicle far away, course back to (0,0)
    Eigen::Vector3f pos{-40.f, 1400.f, -50.f};
    float course = courseToTarget(pos, 0.f, 0.f);
    // Should be roughly south-west (negative y, positive x)
    CHECK(course < 0.f);
    CHECK(course > -static_cast<float>(M_PI));
}

// --- Horizontal distance tests ---

TEST_CASE("VtolDistance.ZeroDistance")
{
    Eigen::Vector3f pos{100.f, 200.f, -50.f};
    CHECK(horizontalDistTo(pos, 100.f, 200.f) == doctest::Approx(0.f).epsilon(0.001));
}

TEST_CASE("VtolDistance.SimpleDistance")
{
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    CHECK(horizontalDistTo(pos, 300.f, 400.f) == doctest::Approx(500.f).epsilon(0.1));
}

TEST_CASE("VtolDistance.IgnoresAltitude")
{
    // Same XY, different altitude -> distance should be 0
    Eigen::Vector3f pos{100.f, 200.f, -10.f};
    CHECK(horizontalDistTo(pos, 100.f, 200.f) == doctest::Approx(0.f).epsilon(0.001));
}

TEST_CASE("VtolDistance.CanyonEndToHome")
{
    // From the last canyon waypoint back to home
    Eigen::Vector3f pos{-40.f, 1400.f, -50.f};
    float dist = horizontalDistTo(pos, 0.f, 0.f);
    CHECK(dist == doctest::Approx(1400.6f).epsilon(1.f));
}

// --- Waypoint acceptance tests ---

TEST_CASE("VtolAcceptance.WithinRadius")
{
    VtolWaypoint wp{100.f, 200.f, 0.f, 50.f};
    Eigen::Vector3f pos{80.f, 190.f, -50.f};
    float dist = horizontalDistTo(pos, wp.x, wp.y);
    CHECK(dist < wp.acceptance_radius);
}

TEST_CASE("VtolAcceptance.OutsideRadius")
{
    VtolWaypoint wp{100.f, 200.f, 0.f, 50.f};
    Eigen::Vector3f pos{0.f, 0.f, -50.f};
    float dist = horizontalDistTo(pos, wp.x, wp.y);
    CHECK(dist > wp.acceptance_radius);
}

TEST_CASE("VtolAcceptance.DefaultRadiusFromConfig")
{
    VtolNavConfig config;
    VtolWaypoint wp{100.f, 0.f, NAN, 0.f};  // accept_radius = 0 -> use config default
    float accept = wp.acceptance_radius > 0.f ? wp.acceptance_radius : config.fw_accept_radius;
    CHECK(accept == doctest::Approx(50.f));
}

TEST_CASE("VtolAcceptance.OverrideRadiusPerWaypoint")
{
    VtolNavConfig config;
    VtolWaypoint wp{100.f, 0.f, NAN, 30.f};  // explicit per-WP radius
    float accept = wp.acceptance_radius > 0.f ? wp.acceptance_radius : config.fw_accept_radius;
    CHECK(accept == doctest::Approx(30.f));
}

// --- Altitude AMSL conversion tests ---

TEST_CASE("VtolAltitude.AmslConversion")
{
    // Ground AMSL ref = 100m, cruise alt = 50m -> target AMSL = 150m
    float alt_amsl_ref = 100.f;
    float cruise_alt_m = 50.f;
    float target = alt_amsl_ref + cruise_alt_m;
    CHECK(target == doctest::Approx(150.f));
}

TEST_CASE("VtolAltitude.AmslRefComputation")
{
    // Current AMSL = 145m, current AGL = 45m -> ground AMSL = 100m
    float current_amsl = 145.f;
    float current_agl = 45.f;
    float alt_amsl_ref = current_amsl - current_agl;
    CHECK(alt_amsl_ref == doctest::Approx(100.f));
}

// --- MC transition distance tests ---

TEST_CASE("VtolTransition.WithinMcTransitionDist")
{
    VtolNavConfig config;
    config.mc_transition_dist = 200.f;
    Eigen::Vector3f pos{150.f, 0.f, -50.f};
    float dist = horizontalDistTo(pos, 0.f, 0.f);
    CHECK(dist < config.mc_transition_dist);
}

TEST_CASE("VtolTransition.OutsideMcTransitionDist")
{
    VtolNavConfig config;
    config.mc_transition_dist = 200.f;
    Eigen::Vector3f pos{-40.f, 1400.f, -50.f};
    float dist = horizontalDistTo(pos, 0.f, 0.f);
    CHECK(dist > config.mc_transition_dist);
}

// --- State transition logic tests ---

TEST_CASE("VtolStateMachine.ClimbAltitudeCheck")
{
    VtolNavConfig config;
    config.cruise_alt_m = 50.f;
    float tolerance = 2.f;

    // At 47m AGL -> not yet at cruise
    float agl_47 = 47.f;
    CHECK_FALSE(agl_47 >= config.cruise_alt_m - tolerance);

    // At 48m AGL -> within tolerance
    float agl_48 = 48.f;
    CHECK(agl_48 >= config.cruise_alt_m - tolerance);

    // At 50m AGL -> exactly at cruise
    float agl_50 = 50.f;
    CHECK(agl_50 >= config.cruise_alt_m - tolerance);
}

TEST_CASE("VtolStateMachine.HomeApproachCheck")
{
    float home_approach_dist = 5.f;

    // At 4m from home -> done
    Eigen::Vector3f pos_close{3.f, 2.f, -50.f};
    CHECK(horizontalDistTo(pos_close, 0.f, 0.f) < home_approach_dist);

    // At 10m from home -> not done
    Eigen::Vector3f pos_far{8.f, 6.f, -50.f};
    CHECK(horizontalDistTo(pos_far, 0.f, 0.f) > home_approach_dist);
}

// --- Config default tests ---

TEST_CASE("VtolConfig.DefaultValues")
{
    VtolNavConfig config;
    CHECK(config.cruise_alt_m == doctest::Approx(50.f));
    CHECK(config.climb_rate == doctest::Approx(2.f));
    CHECK(config.fw_accept_radius == doctest::Approx(50.f));
    CHECK(config.mc_transition_dist == doctest::Approx(200.f));
    CHECK(config.mc_approach_speed == doctest::Approx(5.f));
    CHECK(config.fw_transition_timeout == doctest::Approx(30.f));
    CHECK(config.mc_transition_timeout == doctest::Approx(60.f));
}

// --- Waypoint sequence tests ---

TEST_CASE("VtolWaypoints.CanyonSequenceProgression")
{
    // Canyon waypoints should progress monotonically in Y (east)
    std::vector<VtolWaypoint> wps = {
        {0.f, 200.f, NAN, 0.f},
        {0.f, 400.f, NAN, 0.f},
        {0.f, 600.f, NAN, 0.f},
        {0.f, 800.f, NAN, 0.f},
        {0.f, 1000.f, NAN, 0.f},
        {-20.f, 1200.f, NAN, 0.f},
        {-40.f, 1400.f, NAN, 0.f},
    };

    for (std::size_t i = 1; i < wps.size(); ++i) {
        CHECK_MESSAGE(wps[i].y > wps[i - 1].y,
            "Waypoint ", i, " Y should be greater than waypoint ", i - 1);
    }
}

TEST_CASE("VtolWaypoints.ConsecutiveDistanceReasonable")
{
    std::vector<VtolWaypoint> wps = {
        {0.f, 200.f, NAN, 0.f},
        {0.f, 400.f, NAN, 0.f},
        {0.f, 600.f, NAN, 0.f},
    };

    for (std::size_t i = 1; i < wps.size(); ++i) {
        float dx = wps[i].x - wps[i - 1].x;
        float dy = wps[i].y - wps[i - 1].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        CHECK_MESSAGE(dist > 10.f, "Waypoints too close together");
        CHECK_MESSAGE(dist < 500.f, "Waypoints too far apart");
    }
}

// --- Cable tension monitor tests ---

TEST_CASE("CableTension.BelowWarnNoChange")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_warn_percent = 70.f;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    // 30N = 60% of 50N, below 70% warn threshold
    auto result = checkCableTensionLogic(cfg, State::FwNavigate, 30.f, false, false);
    CHECK_FALSE(result.should_transition);
    CHECK_FALSE(result.is_warning);
}

TEST_CASE("CableTension.AboveWarnBelowAbort")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_warn_percent = 70.f;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    // 40N = 80% of 50N, above 70% warn but below 85% abort
    auto result = checkCableTensionLogic(cfg, State::FwNavigate, 40.f, false, false);
    CHECK_FALSE(result.should_transition);
    CHECK(result.is_warning);
}

TEST_CASE("CableTension.AbortInFwNavigateTransitionsToFwReturn")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    // 44N = 88% of 50N, above 85% abort threshold
    auto result = checkCableTensionLogic(cfg, State::FwNavigate, 44.f, false, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::FwReturn);
}

TEST_CASE("CableTension.AbortInMcClimbTransitionsToMcApproach")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    // 44N above abort threshold during climb
    auto result = checkCableTensionLogic(cfg, State::McClimb, 44.f, false, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::McApproach);
}

TEST_CASE("CableTension.AbortInTransitionFwToMcApproach")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    auto result = checkCableTensionLogic(cfg, State::TransitionFw, 44.f, false, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::McApproach);
}

TEST_CASE("CableTension.AbortInFwReturnNoTransition")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    // Already returning — no state change needed
    auto result = checkCableTensionLogic(cfg, State::FwReturn, 44.f, false, false);
    CHECK_FALSE(result.should_transition);
}

TEST_CASE("CableTension.BrokenCableTransitionsToMcApproach")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.breaking_strength = 50.f;

    auto result = checkCableTensionLogic(cfg, State::FwNavigate, 55.f, true, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::McApproach);
}

TEST_CASE("CableTension.DisabledMonitorNoChange")
{
    CableMonitorConfig cfg;
    cfg.enabled = false;

    // Even with extreme tension, disabled monitor should not trigger
    auto result = checkCableTensionLogic(cfg, State::FwNavigate, 100.f, false, false);
    CHECK_FALSE(result.should_transition);
    CHECK_FALSE(result.is_warning);
}

TEST_CASE("CableTension.AlreadyAbortedNoRetrigger")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.tension_abort_percent = 85.f;
    cfg.breaking_strength = 50.f;

    // Abort was already triggered — should not trigger again
    auto result = checkCableTensionLogic(cfg, State::FwReturn, 44.f, false, true);
    CHECK_FALSE(result.should_transition);
}

TEST_CASE("CableTension.ThresholdComputation")
{
    CableMonitorConfig cfg;
    cfg.breaking_strength = 50.f;
    cfg.tension_warn_percent = 70.f;
    cfg.tension_abort_percent = 85.f;

    float warn = cfg.breaking_strength * cfg.tension_warn_percent / 100.f;
    float abort = cfg.breaking_strength * cfg.tension_abort_percent / 100.f;
    CHECK(warn == doctest::Approx(35.f));
    CHECK(abort == doctest::Approx(42.5f));
}

TEST_CASE("CableMonitorConfig.DefaultValues")
{
    CableMonitorConfig cfg;
    CHECK_FALSE(cfg.enabled);
    CHECK(cfg.tension_warn_percent == doctest::Approx(70.f));
    CHECK(cfg.tension_abort_percent == doctest::Approx(85.f));
    CHECK(cfg.breaking_strength == doctest::Approx(50.f));
    CHECK(cfg.spool_capacity == doctest::Approx(-1.f));
    CHECK(cfg.spool_warn_percent == doctest::Approx(80.f));
    CHECK(cfg.spool_abort_percent == doctest::Approx(95.f));
}

// --- Spool exhaustion tests ---

TEST_CASE("SpoolExhaustion.BelowWarnNoChange")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_warn_percent = 80.f;
    cfg.spool_abort_percent = 95.f;

    // 5000m deployed = 66.7%, below 80% warn
    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 5000.f, false);
    CHECK_FALSE(result.should_transition);
    CHECK_FALSE(result.is_warning);
}

TEST_CASE("SpoolExhaustion.AboveWarnBelowAbort")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_warn_percent = 80.f;
    cfg.spool_abort_percent = 95.f;

    // 6500m deployed = 86.7%, above 80% warn but below 95% abort
    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 6500.f, false);
    CHECK_FALSE(result.should_transition);
    CHECK(result.is_warning);
}

TEST_CASE("SpoolExhaustion.AbortInFwNavigateTransitionsToFwReturn")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_abort_percent = 95.f;

    // 7200m deployed = 96%, above 95% abort
    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 7200.f, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::FwReturn);
}

TEST_CASE("SpoolExhaustion.AbortInMcClimbTransitionsToMcApproach")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_abort_percent = 95.f;

    auto result = checkSpoolExhaustionLogic(cfg, State::McClimb, 7200.f, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::McApproach);
}

TEST_CASE("SpoolExhaustion.AbortInFwReturnNoTransition")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_abort_percent = 95.f;

    // Already returning
    auto result = checkSpoolExhaustionLogic(cfg, State::FwReturn, 7200.f, false);
    CHECK_FALSE(result.should_transition);
}

TEST_CASE("SpoolExhaustion.AlreadyAbortedNoRetrigger")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_abort_percent = 95.f;

    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 7200.f, true);
    CHECK_FALSE(result.should_transition);
}

TEST_CASE("SpoolExhaustion.DisabledWhenCapacityNegative")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = -1.f;  // unlimited

    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 99999.f, false);
    CHECK_FALSE(result.should_transition);
    CHECK_FALSE(result.is_warning);
}

TEST_CASE("SpoolExhaustion.DisabledWhenMonitorOff")
{
    CableMonitorConfig cfg;
    cfg.enabled = false;
    cfg.spool_capacity = 7500.f;

    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 7200.f, false);
    CHECK_FALSE(result.should_transition);
}

TEST_CASE("SpoolExhaustion.ExactBoundary")
{
    CableMonitorConfig cfg;
    cfg.enabled = true;
    cfg.spool_capacity = 7500.f;
    cfg.spool_warn_percent = 80.f;
    cfg.spool_abort_percent = 95.f;

    // Exactly at 95% = 7125m
    auto result = checkSpoolExhaustionLogic(cfg, State::FwNavigate, 7125.f, false);
    CHECK(result.should_transition);
    CHECK(result.new_state == State::FwReturn);
}

// --- GPS-denied: Leg heading computation tests ---

TEST_CASE("GpsDenied.LegHeadings.StraightEast")
{
    // 4 waypoints heading due east (positive Y)
    std::vector<VtolWaypoint> wps = {
        {0.f, 100.f, NAN, 0.f},
        {0.f, 200.f, NAN, 0.f},
        {0.f, 300.f, NAN, 0.f},
        {0.f, 400.f, NAN, 0.f},
    };
    auto hdgs = computeLegHeadingsFromWaypoints(wps);
    REQUIRE(hdgs.size() == 4);
    for (const auto& h : hdgs) {
        CHECK(h == doctest::Approx(static_cast<float>(M_PI / 2.0)).epsilon(0.01));
    }
}

TEST_CASE("GpsDenied.LegHeadings.StraightNorth")
{
    std::vector<VtolWaypoint> wps = {
        {100.f, 0.f, NAN, 0.f},
        {200.f, 0.f, NAN, 0.f},
    };
    auto hdgs = computeLegHeadingsFromWaypoints(wps);
    REQUIRE(hdgs.size() == 2);
    for (const auto& h : hdgs) {
        CHECK(h == doctest::Approx(0.f).epsilon(0.01));
    }
}

TEST_CASE("GpsDenied.LegHeadings.Diagonal")
{
    // Single WP northeast from origin
    std::vector<VtolWaypoint> wps = {
        {100.f, 100.f, NAN, 0.f},
    };
    auto hdgs = computeLegHeadingsFromWaypoints(wps);
    REQUIRE(hdgs.size() == 1);
    CHECK(hdgs[0] == doctest::Approx(static_cast<float>(M_PI / 4.0)).epsilon(0.01));
}

TEST_CASE("GpsDenied.LegHeadings.MultiSegmentTurn")
{
    // First leg east, second leg north-east
    std::vector<VtolWaypoint> wps = {
        {0.f, 200.f, NAN, 0.f},      // From (0,0) → east
        {100.f, 400.f, NAN, 0.f},    // From (0,200) → NE
    };
    auto hdgs = computeLegHeadingsFromWaypoints(wps);
    REQUIRE(hdgs.size() == 2);
    CHECK(hdgs[0] == doctest::Approx(static_cast<float>(M_PI / 2.0)).epsilon(0.01));
    // Second leg: atan2(400-200, 100-0) = atan2(200, 100) ≈ 1.107 rad
    CHECK(hdgs[1] == doctest::Approx(std::atan2(200.f, 100.f)).epsilon(0.01));
}

TEST_CASE("GpsDenied.LegHeadings.EmptyWaypoints")
{
    std::vector<VtolWaypoint> wps;
    auto hdgs = computeLegHeadingsFromWaypoints(wps);
    CHECK(hdgs.empty());
}

// --- GPS-denied: Return heading tests ---

TEST_CASE("GpsDenied.ReturnHeading.AutoFromLastWp")
{
    // Last WP at (0, 400) → return heading = atan2(-400, 0) = -pi/2 (west)
    std::vector<VtolWaypoint> wps = {
        {0.f, 100.f, NAN, 0.f},
        {0.f, 400.f, NAN, 0.f},
    };
    float h = computeReturnHeading(wps, NAN);
    CHECK(h == doctest::Approx(static_cast<float>(-M_PI / 2.0)).epsilon(0.01));
}

TEST_CASE("GpsDenied.ReturnHeading.ExplicitOverride")
{
    std::vector<VtolWaypoint> wps = {
        {0.f, 400.f, NAN, 0.f},
    };
    float h = computeReturnHeading(wps, 1.5f);
    CHECK(h == doctest::Approx(1.5f));
}

TEST_CASE("GpsDenied.ReturnHeading.EmptyWpDefaultsNorth")
{
    std::vector<VtolWaypoint> wps;
    float h = computeReturnHeading(wps, NAN);
    CHECK(h == doctest::Approx(0.f));
}

TEST_CASE("GpsDenied.ReturnHeading.DiagonalReturn")
{
    // Last WP at (200, 300) → return = atan2(-300, -200)
    std::vector<VtolWaypoint> wps = {
        {200.f, 300.f, NAN, 0.f},
    };
    float h = computeReturnHeading(wps, NAN);
    CHECK(h == doctest::Approx(std::atan2(-300.f, -200.f)).epsilon(0.01));
}

// --- GPS-denied: Altitude P-controller tests ---

TEST_CASE("GpsDenied.AltitudeHold.BelowTarget")
{
    // At 20m AGL, target 30m → error = +10m → vz = +5 (climb)
    float vz = altitudeHoldVz(30.f, 20.f, 0.5f, 3.f);
    CHECK(vz == doctest::Approx(3.f));  // clamped to max_vz
}

TEST_CASE("GpsDenied.AltitudeHold.AboveTarget")
{
    // At 40m AGL, target 30m → error = -10m → vz = -5 → clamped to -3
    float vz = altitudeHoldVz(30.f, 40.f, 0.5f, 3.f);
    CHECK(vz == doctest::Approx(-3.f));  // clamped to -max_vz
}

TEST_CASE("GpsDenied.AltitudeHold.AtTarget")
{
    float vz = altitudeHoldVz(30.f, 30.f, 0.5f, 3.f);
    CHECK(vz == doctest::Approx(0.f));
}

TEST_CASE("GpsDenied.AltitudeHold.SmallError")
{
    // At 28m AGL, target 30m → error = +2m → vz = +1.0 (no clamping)
    float vz = altitudeHoldVz(30.f, 28.f, 0.5f, 3.f);
    CHECK(vz == doctest::Approx(1.f));
}

TEST_CASE("GpsDenied.AltitudeHold.DescentToGround")
{
    // GPS-denied McApproach: target = 0m (ground), at 30m AGL
    // error = -30m → vz = -15 → clamped to -3
    float vz = altitudeHoldVz(0.f, 30.f, 0.5f, 3.f);
    CHECK(vz == doctest::Approx(-3.f));
}

// --- GPS-denied: Config defaults ---

TEST_CASE("GpsDeniedConfig.DefaultValues")
{
    GpsDeniedConfig cfg;
    CHECK_FALSE(cfg.enabled);
    CHECK(cfg.wp_time_s == doctest::Approx(30.f));
    CHECK(cfg.return_time_s == doctest::Approx(120.f));
    CHECK(cfg.descent_time_s == doctest::Approx(60.f));
    CHECK(cfg.altitude_kp == doctest::Approx(0.5f));
    CHECK(cfg.altitude_max_vz == doctest::Approx(3.f));
    CHECK(cfg.fw_speed == doctest::Approx(18.f));
    CHECK(std::isnan(cfg.return_heading));
}

TEST_CASE("VtolConfig.GpsDeniedInConfig")
{
    VtolNavConfig config;
    CHECK_FALSE(config.gps_denied.enabled);
    config.gps_denied.enabled = true;
    config.gps_denied.wp_time_s = 25.f;
    CHECK(config.gps_denied.wp_time_s == doctest::Approx(25.f));
}

// --- Position EKF: drPosition() logic tests ---

TEST_CASE("GpsDenied.DrPosition.ReturnsNulloptWhenDisabled")
{
    GpsDeniedConfig cfg;
    cfg.use_position_ekf = false;
    auto pos = drPosition(cfg, true, 100.f, 200.f, 5.f, 5.f);
    CHECK_FALSE(pos.has_value());
}

TEST_CASE("GpsDenied.DrPosition.ReturnsNulloptWhenInvalid")
{
    GpsDeniedConfig cfg;
    cfg.use_position_ekf = true;
    auto pos = drPosition(cfg, false, 100.f, 200.f, 5.f, 5.f);
    CHECK_FALSE(pos.has_value());
}

TEST_CASE("GpsDenied.DrPosition.ReturnsNulloptWhenUncertaintyTooHigh")
{
    GpsDeniedConfig cfg;
    cfg.use_position_ekf = true;
    cfg.ekf_max_uncertainty = 200.f;
    // sigma_y = 250 > 200 threshold
    auto pos = drPosition(cfg, true, 100.f, 200.f, 50.f, 250.f);
    CHECK_FALSE(pos.has_value());
}

TEST_CASE("GpsDenied.DrPosition.ReturnsPositionWhenValid")
{
    GpsDeniedConfig cfg;
    cfg.use_position_ekf = true;
    cfg.ekf_max_uncertainty = 200.f;
    auto pos = drPosition(cfg, true, 100.f, 200.f, 10.f, 15.f);
    REQUIRE(pos.has_value());
    CHECK(pos->x() == doctest::Approx(100.f));
    CHECK(pos->y() == doctest::Approx(200.f));
}

TEST_CASE("GpsDenied.DrPosition.BoundaryUncertaintyExact")
{
    GpsDeniedConfig cfg;
    cfg.use_position_ekf = true;
    cfg.ekf_max_uncertainty = 200.f;
    // Exactly at threshold — NOT rejected (> is strict, 200 is not > 200)
    auto pos_at = drPosition(cfg, true, 0.f, 0.f, 200.f, 200.f);
    CHECK(pos_at.has_value());
    // Just above threshold — rejected
    auto pos_above = drPosition(cfg, true, 0.f, 0.f, 201.f, 201.f);
    CHECK_FALSE(pos_above.has_value());
}

// --- Position EKF: Course steering with EKF position ---

TEST_CASE("GpsDenied.EkfSteering.CourseToWaypointFromEkfPos")
{
    // EKF reports position at (0, 200), next WP at (0, 400)
    // Course should be due east (pi/2)
    Eigen::Vector3f ekf_pos{0.f, 200.f, -30.f};
    float course = courseToTarget(ekf_pos, 0.f, 400.f);
    CHECK(course == doctest::Approx(static_cast<float>(M_PI / 2.0)).epsilon(0.01));
}

TEST_CASE("GpsDenied.EkfSteering.CourseToHomeFromEkfPos")
{
    // EKF reports position at (0, 300), home is (0, 0)
    // Course = atan2(0 - 300, 0 - 0) = atan2(-300, 0) = -pi/2 (west)
    Eigen::Vector3f ekf_pos{0.f, 300.f, -30.f};
    float course = courseToTarget(ekf_pos, 0.f, 0.f);
    CHECK(course == doctest::Approx(static_cast<float>(-M_PI / 2.0)).epsilon(0.01));
}

TEST_CASE("GpsDenied.EkfSteering.CourseCorrection")
{
    // EKF detects lateral drift: at (50, 200) instead of (0, 200)
    // Next WP at (0, 400) → course should point slightly south-east to correct
    Eigen::Vector3f ekf_pos{50.f, 200.f, -30.f};
    float course = courseToTarget(ekf_pos, 0.f, 400.f);
    // Expected: atan2(200, -50) ≈ 1.816 rad (between pi/2 and pi)
    CHECK(course == doctest::Approx(std::atan2(200.f, -50.f)).epsilon(0.01));
    CHECK(course > static_cast<float>(M_PI / 2.0));  // correcting south
}

// --- Position EKF: Distance-based WP/home acceptance ---

TEST_CASE("GpsDenied.EkfAcceptance.WpAcceptedWithinRadius")
{
    GpsDeniedConfig cfg;
    cfg.ekf_wp_accept_radius = 80.f;
    // EKF at (0, 370), WP at (0, 400) → dist = 30m < 80m
    Eigen::Vector3f ekf_pos{0.f, 370.f, -30.f};
    float dist = horizontalDistTo(ekf_pos, 0.f, 400.f);
    CHECK(dist < cfg.ekf_wp_accept_radius);
}

TEST_CASE("GpsDenied.EkfAcceptance.WpNotAcceptedOutsideRadius")
{
    GpsDeniedConfig cfg;
    cfg.ekf_wp_accept_radius = 80.f;
    // EKF at (0, 200), WP at (0, 400) → dist = 200m > 80m
    Eigen::Vector3f ekf_pos{0.f, 200.f, -30.f};
    float dist = horizontalDistTo(ekf_pos, 0.f, 400.f);
    CHECK(dist > cfg.ekf_wp_accept_radius);
}

TEST_CASE("GpsDenied.EkfAcceptance.HomeAcceptedWithinRadius")
{
    GpsDeniedConfig cfg;
    cfg.ekf_home_accept_radius = 100.f;
    // EKF at (30, 40, -30) → dist to home = 50m < 100m
    Eigen::Vector3f ekf_pos{30.f, 40.f, -30.f};
    float dist = horizontalDistTo(ekf_pos, 0.f, 0.f);
    CHECK(dist < cfg.ekf_home_accept_radius);
}

TEST_CASE("GpsDenied.EkfAcceptance.HomeNotAcceptedOutsideRadius")
{
    GpsDeniedConfig cfg;
    cfg.ekf_home_accept_radius = 100.f;
    // EKF at (0, 300) → dist = 300m > 100m
    Eigen::Vector3f ekf_pos{0.f, 300.f, -30.f};
    float dist = horizontalDistTo(ekf_pos, 0.f, 0.f);
    CHECK(dist > cfg.ekf_home_accept_radius);
}

// --- Position EKF: Config defaults ---

TEST_CASE("GpsDeniedConfig.EkfDefaultValues")
{
    GpsDeniedConfig cfg;
    CHECK_FALSE(cfg.use_position_ekf);
    CHECK(cfg.ekf_wp_accept_radius == doctest::Approx(80.f));
    CHECK(cfg.ekf_home_accept_radius == doctest::Approx(100.f));
    CHECK(cfg.ekf_max_uncertainty == doctest::Approx(200.f));
}

// --- Gimbal accommodation tests ---

TEST_CASE("GimbalAccom.DefaultConfig")
{
    GimbalAccommodationConfig cfg;
    CHECK_FALSE(cfg.enabled);
    CHECK(cfg.saturation_threshold == doctest::Approx(0.7f));
    CHECK(cfg.base_turn_rate == doctest::Approx(5.f));
    CHECK(cfg.min_turn_rate == doctest::Approx(1.f));
}

TEST_CASE("GimbalAccom.EffectiveTurnRate.BelowThreshold")
{
    // Saturation below threshold → full base rate
    float rate = effectiveTurnRate(0.3f, 0.7f, 5.f, 1.f);
    CHECK(rate == doctest::Approx(5.f));
}

TEST_CASE("GimbalAccom.EffectiveTurnRate.AtThreshold")
{
    // Exactly at threshold → still base rate
    float rate = effectiveTurnRate(0.7f, 0.7f, 5.f, 1.f);
    CHECK(rate == doctest::Approx(5.f));
}

TEST_CASE("GimbalAccom.EffectiveTurnRate.FullSaturation")
{
    // Saturation at 1.0 → min rate
    float rate = effectiveTurnRate(1.0f, 0.7f, 5.f, 1.f);
    CHECK(rate == doctest::Approx(1.f));
}

TEST_CASE("GimbalAccom.EffectiveTurnRate.MidSaturation")
{
    // Saturation at 0.85 → halfway between threshold and 1.0
    // t = (0.85 - 0.7) / (1.0 - 0.7) = 0.5
    // rate = 5.0 + 0.5 * (1.0 - 5.0) = 3.0
    float rate = effectiveTurnRate(0.85f, 0.7f, 5.f, 1.f);
    CHECK(rate == doctest::Approx(3.f));
}

TEST_CASE("GimbalAccom.EffectiveTurnRate.ClampAboveOne")
{
    // Saturation > 1.0 (shouldn't happen, but should clamp)
    float rate = effectiveTurnRate(1.5f, 0.7f, 5.f, 1.f);
    CHECK(rate == doctest::Approx(1.f));
}

TEST_CASE("GimbalAccom.EffectiveTurnRate.ZeroSaturation")
{
    // Zero saturation → base rate
    float rate = effectiveTurnRate(0.f, 0.7f, 5.f, 1.f);
    CHECK(rate == doctest::Approx(5.f));
}

TEST_CASE("GimbalAccom.RateLimitedCourse.SmallStepAtBaseRate")
{
    // 5 deg/s for 0.1s = 0.5 deg max step
    float result = rateLimitedCourse(0.f, 0.1f, 5.f, 0.1f);
    float max_step = 5.f * static_cast<float>(M_PI) / 180.f * 0.1f;
    CHECK(std::abs(result) <= max_step + 0.001f);
    CHECK(result > 0.f);  // Moving toward target
}

TEST_CASE("GimbalAccom.RateLimitedCourse.LargeStepClamped")
{
    // Target is 90° away, but rate limit clamps step to ~5°*dt
    float target = static_cast<float>(M_PI / 2.0);  // 90° east
    float result = rateLimitedCourse(0.f, target, 5.f, 1.f);
    // At 5 deg/s for 1s = 5 deg = ~0.087 rad
    float expected_step = 5.f * static_cast<float>(M_PI) / 180.f;
    CHECK(result == doctest::Approx(expected_step).epsilon(0.01));
}

TEST_CASE("GimbalAccom.CourseConvergence.GimbalModulated")
{
    // Simulate convergence with varying gimbal saturation
    // Start heading north (0), target heading east (π/2)
    float course = 0.f;
    float target = static_cast<float>(M_PI / 2.0);
    float dt = 0.02f;  // 50 Hz

    // First 50 steps: low saturation → base rate (5 deg/s)
    for (int i = 0; i < 50; ++i) {
        float rate = effectiveTurnRate(0.2f, 0.7f, 5.f, 1.f);
        course = rateLimitedCourse(course, target, rate, dt);
    }
    // 50 * 0.02 = 1.0s at 5 deg/s = 5 degrees
    float expected_1 = 5.f * static_cast<float>(M_PI) / 180.f;
    CHECK(course == doctest::Approx(expected_1).epsilon(0.02));

    // Next 50 steps: high saturation → min rate (1 deg/s)
    float course_before = course;
    for (int i = 0; i < 50; ++i) {
        float rate = effectiveTurnRate(1.0f, 0.7f, 5.f, 1.f);
        course = rateLimitedCourse(course, target, rate, dt);
    }
    // 50 * 0.02 = 1.0s at 1 deg/s = 1 degree additional
    float expected_delta = 1.f * static_cast<float>(M_PI) / 180.f;
    CHECK((course - course_before) == doctest::Approx(expected_delta).epsilon(0.02));
}

// --- Altitude rate scale tests ---

TEST_CASE("GimbalAccom.AltRateScale.BelowThreshold")
{
    float scale = effectiveAltRateScale(0.3f, 0.7f, 0.3f);
    CHECK(scale == doctest::Approx(1.f));
}

TEST_CASE("GimbalAccom.AltRateScale.AtThreshold")
{
    float scale = effectiveAltRateScale(0.7f, 0.7f, 0.3f);
    CHECK(scale == doctest::Approx(1.f));
}

TEST_CASE("GimbalAccom.AltRateScale.FullSaturation")
{
    float scale = effectiveAltRateScale(1.0f, 0.7f, 0.3f);
    CHECK(scale == doctest::Approx(0.3f));
}

TEST_CASE("GimbalAccom.AltRateScale.MidSaturation")
{
    // t = (0.85 - 0.7) / (1.0 - 0.7) = 0.5
    // scale = lerp(1.0, 0.3, 0.5) = 0.65
    float scale = effectiveAltRateScale(0.85f, 0.7f, 0.3f);
    CHECK(scale == doctest::Approx(0.65f));
}
