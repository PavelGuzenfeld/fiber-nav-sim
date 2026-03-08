#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <fiber_nav_fusion/position_ekf.hpp>

#include <cmath>

using namespace fiber_nav_fusion;

TEST_CASE("initialize: state is zero with correct covariance") {
    PositionEkfConfig config;
    config.p0_pos = 2.0f;
    config.p0_vel = 3.0f;
    config.p0_wind = 5.0f;

    auto s = initialize(config);

    CHECK(s.initialized);
    CHECK(s.x(0) == 0.0f);
    CHECK(s.x(1) == 0.0f);
    CHECK(s.x(2) == 0.0f);
    CHECK(s.x(3) == 0.0f);
    CHECK(s.x(4) == 0.0f);
    CHECK(s.x(5) == 0.0f);

    CHECK(s.P(0, 0) == doctest::Approx(2.0f));
    CHECK(s.P(1, 1) == doctest::Approx(2.0f));
    CHECK(s.P(2, 2) == doctest::Approx(3.0f));
    CHECK(s.P(3, 3) == doctest::Approx(3.0f));
    CHECK(s.P(4, 4) == doctest::Approx(5.0f));
    CHECK(s.P(5, 5) == doctest::Approx(5.0f));
    CHECK(s.P(0, 1) == doctest::Approx(0.0f));
}

TEST_CASE("initializeAt: sets position") {
    PositionEkfConfig config;
    auto s = initializeAt(config, 10.0f, 20.0f);

    CHECK(s.x(0) == doctest::Approx(10.0f));
    CHECK(s.x(1) == doctest::Approx(20.0f));
    CHECK(s.initialized);
}

TEST_CASE("predict with zero velocity: position unchanged, P grows") {
    PositionEkfConfig config;
    auto s = initialize(config);

    float p00_before = s.P(0, 0);
    auto s2 = predict(s, config, 0.1f, 0.0f, 0.0f);

    CHECK(s2.x(0) == doctest::Approx(0.0f));
    CHECK(s2.x(1) == doctest::Approx(0.0f));
    // Covariance should grow due to process noise
    CHECK(s2.P(0, 0) > p00_before);
}

TEST_CASE("predict with constant velocity: position advances") {
    PositionEkfConfig config;
    auto s = initialize(config);

    float dt = 1.0f;
    auto s2 = predict(s, config, dt, 10.0f, 5.0f);

    // pos += (v - wind) * dt, wind=0
    CHECK(s2.x(0) == doctest::Approx(10.0f));
    CHECK(s2.x(1) == doctest::Approx(5.0f));
    CHECK(s2.x(2) == doctest::Approx(10.0f));
    CHECK(s2.x(3) == doctest::Approx(5.0f));
}

TEST_CASE("predict with wind: position offset by wind") {
    PositionEkfConfig config;
    auto s = initialize(config);
    // Set wind state manually
    s.x(4) = 2.0f;  // 2 m/s north wind
    s.x(5) = 1.0f;  // 1 m/s east wind

    float dt = 1.0f;
    auto s2 = predict(s, config, dt, 10.0f, 5.0f);

    // pos += (v - wind) * dt = (10-2, 5-1) = (8, 4)
    CHECK(s2.x(0) == doctest::Approx(8.0f));
    CHECK(s2.x(1) == doctest::Approx(4.0f));
}

TEST_CASE("predict with zero dt: no change") {
    PositionEkfConfig config;
    auto s = initialize(config);
    s.x(0) = 5.0f;

    auto s2 = predict(s, config, 0.0f, 10.0f, 10.0f);
    CHECK(s2.x(0) == doctest::Approx(5.0f));
}

TEST_CASE("updateVelocity high quality: state converges to measurement") {
    PositionEkfConfig config;
    config.r_velocity = 0.01f;  // low noise
    auto s = initialize(config);

    // Velocity state is zero, measurement says (5, 3)
    auto s2 = updateVelocity(s, config, 5.0f, 3.0f, 1.0f);

    // With low R and moderate P, should converge close to measurement
    CHECK(s2.x(2) == doctest::Approx(5.0f).epsilon(0.5));
    CHECK(s2.x(3) == doctest::Approx(3.0f).epsilon(0.5));
}

TEST_CASE("updateVelocity low quality: slow convergence") {
    PositionEkfConfig config;
    config.r_velocity = 0.5f;
    config.of_quality_scale = 10.0f;
    auto s = initialize(config);

    // Low quality → high R → less correction
    auto s_low = updateVelocity(s, config, 5.0f, 3.0f, 0.3f);
    auto s_high = updateVelocity(s, config, 5.0f, 3.0f, 1.0f);

    // High quality should converge more
    float correction_low = std::abs(s_low.x(2));
    float correction_high = std::abs(s_high.x(2));
    CHECK(correction_high > correction_low);
}

TEST_CASE("updateSpeedConsistency: corrects speed magnitude") {
    PositionEkfConfig config;
    config.r_speed = 0.01f;
    auto s = initialize(config);
    // State thinks velocity is (10, 0) → speed = 10
    s.x(2) = 10.0f;
    s.x(3) = 0.0f;

    // Spool says speed is 12
    auto s2 = updateSpeedConsistency(s, config, 12.0f);

    float speed_after = std::sqrt(s2.x(2) * s2.x(2) + s2.x(3) * s2.x(3));
    // Should move toward 12
    CHECK(speed_after > 10.0f);
    CHECK(speed_after <= 12.1f);
}

TEST_CASE("updateSpeedConsistency: skips when speed near zero") {
    PositionEkfConfig config;
    auto s = initialize(config);
    s.x(2) = 0.01f;
    s.x(3) = 0.01f;

    auto s2 = updateSpeedConsistency(s, config, 5.0f);

    // Should be unchanged (speed < 0.1 threshold)
    CHECK(s2.x(2) == doctest::Approx(s.x(2)));
    CHECK(s2.x(3) == doctest::Approx(s.x(3)));
}

TEST_CASE("applyCableConstraint: no change when inside margin") {
    PositionEkfConfig config;
    config.cable_margin = 0.95f;
    auto s = initialize(config);
    s.x(0) = 50.0f;
    s.x(1) = 0.0f;

    // Cable is 100m, position is 50m → well within margin (95m), no correction
    auto s2 = applyCableConstraint(s, config, 100.0f);

    CHECK(s2.x(0) == doctest::Approx(50.0f));
    CHECK(s2.x(1) == doctest::Approx(0.0f));
}

TEST_CASE("applyCableConstraint: pulls position back when outside margin") {
    PositionEkfConfig config;
    config.cable_margin = 0.95f;
    auto s = initialize(config);
    s.x(0) = 120.0f;  // 120m north
    s.x(1) = 0.0f;
    s.P(0, 0) = 100.0f;

    // Cable is 100m, position is 120m → exceeds margin (95m), pulls back
    auto s2 = applyCableConstraint(s, config, 100.0f);

    CHECK(s2.x(0) < 120.0f);
    CHECK(s2.x(0) > 50.0f);  // shouldn't overshoot
}

TEST_CASE("applyCableConstraint: zero cable length returns unchanged") {
    PositionEkfConfig config;
    auto s = initialize(config);
    s.x(0) = 100.0f;

    auto s2 = applyCableConstraint(s, config, 0.0f);
    CHECK(s2.x(0) == doctest::Approx(100.0f));
}

TEST_CASE("updatePosition: corrects position toward measurement") {
    PositionEkfConfig config;
    config.p0_pos = 100.0f;  // large initial uncertainty
    auto s = initialize(config);
    s.x(0) = 50.0f;
    s.x(1) = 30.0f;

    // TERCOM says we're at (60, 40) with 25 m² variance
    auto s2 = updatePosition(s, 60.0f, 40.0f, 25.0f);

    // Position should move toward measurement
    CHECK(s2.x(0) > 50.0f);
    CHECK(s2.x(0) < 60.0f);
    CHECK(s2.x(1) > 30.0f);
    CHECK(s2.x(1) < 40.0f);

    // Covariance should decrease
    CHECK(s2.P(0, 0) < s.P(0, 0));
    CHECK(s2.P(1, 1) < s.P(1, 1));
}

TEST_CASE("updatePosition: low variance measurement dominates") {
    PositionEkfConfig config;
    config.p0_pos = 100.0f;
    auto s = initialize(config);
    s.x(0) = 50.0f;

    // Very precise measurement (low variance)
    auto s2 = updatePosition(s, 60.0f, 0.0f, 0.01f);

    // Should snap very close to measurement
    CHECK(s2.x(0) == doctest::Approx(60.0f).epsilon(1.0));
}

TEST_CASE("updatePosition: high variance measurement has little effect") {
    PositionEkfConfig config;
    config.p0_pos = 1.0f;  // small state uncertainty
    auto s = initialize(config);
    s.x(0) = 50.0f;

    // Very noisy measurement (high variance)
    auto s2 = updatePosition(s, 100.0f, 0.0f, 10000.0f);

    // Should barely move
    CHECK(s2.x(0) == doctest::Approx(50.0f).epsilon(1.0));
}

TEST_CASE("resetPosition: sets position and variance") {
    PositionEkfConfig config;
    auto s = initialize(config);
    s.x(0) = 100.0f;
    s.x(1) = 200.0f;
    s.x(2) = 5.0f;

    auto s2 = resetPosition(s, 10.0f, 20.0f, 0.5f);

    CHECK(s2.x(0) == doctest::Approx(10.0f));
    CHECK(s2.x(1) == doctest::Approx(20.0f));
    CHECK(s2.x(2) == doctest::Approx(5.0f));  // velocity unchanged
    CHECK(s2.P(0, 0) == doctest::Approx(0.5f));
    CHECK(s2.P(1, 1) == doctest::Approx(0.5f));
}

TEST_CASE("full cycle: predict + update converges") {
    PositionEkfConfig config;
    config.r_velocity = 0.1f;
    auto s = initialize(config);

    // Simulate 10 steps of constant 5 m/s north velocity
    float dt = 0.1f;
    for (int i = 0; i < 10; ++i) {
        s = predict(s, config, dt, 5.0f, 0.0f);
        s = updateVelocity(s, config, 5.0f, 0.0f, 1.0f);
    }

    // After 1 second at 5 m/s north, position should be ~5m north
    CHECK(s.x(0) == doctest::Approx(5.0f).epsilon(1.0));
    CHECK(s.x(2) == doctest::Approx(5.0f).epsilon(1.0));
}

TEST_CASE("positionSigma decreases with velocity updates") {
    PositionEkfConfig config;
    config.p0_vel = 10.0f;
    config.r_velocity = 0.01f;
    auto s = initialize(config);

    float p_vel_before = s.P(2, 2);

    // Multiple predict+update cycles reduce velocity uncertainty,
    // which through correlation reduces position uncertainty
    for (int i = 0; i < 50; ++i) {
        s = predict(s, config, 0.02f, 5.0f, 0.0f);
        s = updateVelocity(s, config, 5.0f, 0.0f, 1.0f);
    }

    // Velocity sigma should decrease (checked via vel diagonal)
    CHECK(s.P(2, 2) < p_vel_before);
}

TEST_CASE("distanceFromHome: basic geometry") {
    PositionEkfConfig config;
    auto s = initialize(config);
    s.x(0) = 3.0f;
    s.x(1) = 4.0f;

    CHECK(distanceFromHome(s) == doctest::Approx(5.0f));
}

TEST_CASE("accessors: position, velocity, wind") {
    PositionEkfConfig config;
    auto s = initialize(config);
    s.x(0) = 1.0f; s.x(1) = 2.0f;
    s.x(2) = 3.0f; s.x(3) = 4.0f;
    s.x(4) = 5.0f; s.x(5) = 6.0f;

    auto pos = position(s);
    CHECK(pos[0] == doctest::Approx(1.0f));
    CHECK(pos[1] == doctest::Approx(2.0f));

    auto vel = velocity(s);
    CHECK(vel[0] == doctest::Approx(3.0f));
    CHECK(vel[1] == doctest::Approx(4.0f));

    auto w = wind(s);
    CHECK(w[0] == doctest::Approx(5.0f));
    CHECK(w[1] == doctest::Approx(6.0f));
}

// --- Anisotropic updatePosition tests ---

TEST_CASE("AnisotropicUpdate: large cross-track variance gives small Y correction") {
    PositionEkfConfig config;
    config.p0_pos = 100.0f;  // large initial uncertainty
    auto s = initialize(config);
    s.x(0) = 50.0f;   // North
    s.x(1) = 30.0f;   // East

    // Measurement at (60, 40) with tight X (North) but loose Y (East)
    float var_xx = 25.0f;    // 5m sigma in X — tight
    float var_yy = 10000.0f; // 100m sigma in Y — very loose
    float var_xy = 0.0f;

    auto s2 = updatePosition(s, 60.0f, 40.0f, var_xx, var_yy, var_xy);

    // X correction should be significant (low var_xx vs high P)
    float x_correction = std::abs(s2.x(0) - s.x(0));
    // Y correction should be small (high var_yy, measurement is ignored)
    float y_correction = std::abs(s2.x(1) - s.x(1));

    CHECK(x_correction > 3.0f);   // Significant X correction
    CHECK(y_correction < 2.0f);   // Small Y correction (high var_yy dominates)
    CHECK(x_correction > y_correction);  // X correction dominates
}

// --- Cross-track path prior tests ---

TEST_CASE("CrossTrackPrior: pulls position toward path (featureless terrain)") {
    PositionEkfConfig config;
    config.p0_pos = 100.0f;  // large uncertainty
    auto s = initialize(config);
    // Position 50m east of a North-heading path at x=0
    s.x(0) = 0.0f;   // North: on path
    s.x(1) = 50.0f;  // East: 50m off path

    // Cross-track vector for North-heading leg: cross = (-sin(0), cos(0)) = (0, 1)
    float cross_x = 0.f;
    float cross_y = 1.f;
    float cross_track_dist = 50.f;  // 50m east of path

    // Low discriminability → strong constraint (R = r_min)
    float r_min = 100.f;    // 10m sigma
    float r_max = 90000.f;  // 300m sigma

    auto s2 = updateCrossTrackPrior(s, cross_x, cross_y, cross_track_dist,
                                     0.0f, r_min, r_max);

    // East position should be pulled toward 0 (path centerline)
    CHECK(s2.x(1) < 50.0f);   // moved toward path
    CHECK(s2.x(1) > 0.0f);    // didn't overshoot
    // North position should be unaffected (H only observes cross-track)
    CHECK(s2.x(0) == doctest::Approx(0.0f).epsilon(0.1f));
    // Covariance in cross-track direction should decrease
    CHECK(s2.P(1, 1) < s.P(1, 1));
}

TEST_CASE("CrossTrackPrior: high discriminability has minimal effect") {
    PositionEkfConfig config;
    config.p0_pos = 100.0f;
    auto s = initialize(config);
    s.x(0) = 0.0f;
    s.x(1) = 50.0f;  // 50m off path

    float cross_x = 0.f;
    float cross_y = 1.f;
    float cross_track_dist = 50.f;

    float r_min = 100.f;
    float r_max = 90000.f;

    // High discriminability → weak constraint (R = r_max)
    auto s2 = updateCrossTrackPrior(s, cross_x, cross_y, cross_track_dist,
                                     1.0f, r_min, r_max);

    // With R = 90000 and P = 100, correction should be tiny
    float correction = std::abs(s2.x(1) - 50.0f);
    CHECK(correction < 1.0f);  // barely moved
}

TEST_CASE("AnisotropicUpdate: isotropic matches scalar updatePosition") {
    PositionEkfConfig config;
    config.p0_pos = 100.0f;
    auto s = initialize(config);
    s.x(0) = 50.0f;
    s.x(1) = 30.0f;

    float variance = 25.0f;

    // Scalar update
    auto s_scalar = updatePosition(s, 60.0f, 40.0f, variance);

    // Anisotropic update with equal variances (should match scalar)
    auto s_aniso = updatePosition(s, 60.0f, 40.0f, variance, variance, 0.0f);

    CHECK(s_aniso.x(0) == doctest::Approx(s_scalar.x(0)).epsilon(1e-4));
    CHECK(s_aniso.x(1) == doctest::Approx(s_scalar.x(1)).epsilon(1e-4));
    CHECK(s_aniso.P(0, 0) == doctest::Approx(s_scalar.P(0, 0)).epsilon(1e-4));
    CHECK(s_aniso.P(1, 1) == doctest::Approx(s_scalar.P(1, 1)).epsilon(1e-4));
}
