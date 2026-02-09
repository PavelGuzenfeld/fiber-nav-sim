#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <random>
#include <cmath>
#include <vector>
#include <numeric>

namespace {

// Replicate spool sensor math for testing
struct SpoolModel {
    double noise_stddev = 0.1;
    double slack_factor = 1.05;

    std::mt19937 rng{42};  // Fixed seed for reproducibility
    std::normal_distribution<double> noise_dist{0.0, noise_stddev};

    double compute(double true_velocity) {
        double noisy = true_velocity + noise_dist(rng);
        double biased = noisy * slack_factor;
        return std::max(0.0, biased);
    }
};

}  // namespace

TEST_CASE("SpoolSensor.BiasApplication") {
    SpoolModel model;
    model.noise_stddev = 1e-9;  // Negligible noise for this test
    model.noise_dist = std::normal_distribution<double>(0.0, 1e-9);

    double true_vel = 10.0;
    double measured = model.compute(true_vel);

    // Expected: 10.0 * 1.05 = 10.5 (within tiny noise margin)
    CHECK(measured == doctest::Approx(10.5).epsilon(1e-6));
}

TEST_CASE("SpoolSensor.NoiseDistribution") {
    SpoolModel model;
    model.slack_factor = 1.0;  // No bias for this test

    std::vector<double> errors;
    double true_vel = 10.0;

    for (int i = 0; i < 10000; ++i) {
        double measured = model.compute(true_vel);
        errors.push_back(measured - true_vel);
    }

    // Calculate mean and stddev
    double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
    double sq_sum = 0.0;
    for (double e : errors) {
        sq_sum += (e - mean) * (e - mean);
    }
    double stddev = std::sqrt(sq_sum / static_cast<double>(errors.size()));

    // Mean should be ~0 (within 3σ/√N)
    CHECK(mean == doctest::Approx(0.0).epsilon(0.01));

    // Stddev should match configured noise
    CHECK(stddev == doctest::Approx(0.1).epsilon(0.01));
}

TEST_CASE("SpoolSensor.ZeroVelocity") {
    SpoolModel model;

    // Even with noise, output should be clamped to non-negative
    model.rng.seed(12345);
    for (int i = 0; i < 1000; ++i) {
        double measured = model.compute(0.0);
        CHECK(measured >= 0.0);
    }
}

TEST_CASE("SpoolSensor.HighVelocity") {
    SpoolModel model;
    model.noise_stddev = 1e-9;
    model.noise_dist = std::normal_distribution<double>(0.0, 1e-9);

    // Test at cruise speed
    double measured = model.compute(18.0);
    CHECK(measured == doctest::Approx(18.0 * 1.05).epsilon(1e-6));

    // Test at max speed
    measured = model.compute(30.0);
    CHECK(measured == doctest::Approx(30.0 * 1.05).epsilon(1e-6));
}

TEST_CASE("SpoolSensor.NegativeVelocityClamp") {
    SpoolModel model;
    model.noise_stddev = 1e-9;
    model.noise_dist = std::normal_distribution<double>(0.0, 1e-9);

    // Negative velocity (shouldn't happen, but handle it)
    double measured = model.compute(-5.0);

    // After clamping, should be 0
    CHECK(measured == 0.0);
}

// --- SpoolStatus model tests ---

struct SpoolStatusModel {
    double moving_threshold = 0.05;
    double total_length = 0.0;

    void update(double speed, double dt) {
        total_length += speed * dt;
    }

    bool is_moving(double speed) const {
        return speed > moving_threshold;
    }
};

TEST_CASE("SpoolStatus.TotalLengthIntegration") {
    SpoolStatusModel model;

    // 10 m/s for 1 second
    model.update(10.0, 1.0);
    CHECK(model.total_length == doctest::Approx(10.0));

    // 5 m/s for 2 seconds
    model.update(5.0, 2.0);
    CHECK(model.total_length == doctest::Approx(20.0));
}

TEST_CASE("SpoolStatus.TotalLengthAtRest") {
    SpoolStatusModel model;

    // 0 m/s for 10 seconds
    model.update(0.0, 10.0);
    CHECK(model.total_length == doctest::Approx(0.0));
}

TEST_CASE("SpoolStatus.IsMovingAboveThreshold") {
    SpoolStatusModel model;
    CHECK(model.is_moving(1.0) == true);
}

TEST_CASE("SpoolStatus.IsMovingBelowThreshold") {
    SpoolStatusModel model;
    CHECK(model.is_moving(0.01) == false);
}

TEST_CASE("SpoolStatus.IsMovingAtThreshold") {
    SpoolStatusModel model;
    // At exactly the threshold, not strictly greater → false
    CHECK(model.is_moving(0.05) == false);
}
