#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <cmath>
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

}  // namespace fiber_nav_mode

using fiber_nav_mode::VtolNavConfig;
using fiber_nav_mode::VtolWaypoint;
using fiber_nav_mode::CableMonitorConfig;
using fiber_nav_mode::State;
using fiber_nav_mode::CableTensionResult;
using fiber_nav_mode::checkCableTensionLogic;
using fiber_nav_mode::courseToTarget;
using fiber_nav_mode::horizontalDistTo;

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
}
