#include <gtest/gtest.h>
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

TEST(SpoolSensor, BiasApplication) {
    SpoolModel model;
    model.noise_stddev = 0.0;  // No noise for this test
    model.noise_dist = std::normal_distribution<double>(0.0, 0.0);

    double true_vel = 10.0;
    double measured = model.compute(true_vel);

    // Expected: 10.0 * 1.05 = 10.5
    EXPECT_NEAR(measured, 10.5, 1e-6);
}

TEST(SpoolSensor, NoiseDistribution) {
    SpoolModel model;
    model.slack_factor = 1.0;  // No bias for this test

    std::vector<double> errors;
    double true_vel = 10.0;

    for (int i = 0; i < 10000; ++i) {
        double measured = model.compute(true_vel);
        errors.push_back(measured - true_vel);
    }

    // Calculate mean and stddev
    double mean = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
    double sq_sum = 0.0;
    for (double e : errors) {
        sq_sum += (e - mean) * (e - mean);
    }
    double stddev = std::sqrt(sq_sum / errors.size());

    // Mean should be ~0 (within 3σ/√N)
    EXPECT_NEAR(mean, 0.0, 0.01);

    // Stddev should match configured noise
    EXPECT_NEAR(stddev, 0.1, 0.01);
}

TEST(SpoolSensor, ZeroVelocity) {
    SpoolModel model;

    // Even with noise, output should be clamped to non-negative
    model.rng.seed(12345);
    for (int i = 0; i < 1000; ++i) {
        double measured = model.compute(0.0);
        EXPECT_GE(measured, 0.0);
    }
}

TEST(SpoolSensor, HighVelocity) {
    SpoolModel model;
    model.noise_stddev = 0.0;
    model.noise_dist = std::normal_distribution<double>(0.0, 0.0);

    // Test at cruise speed
    double measured = model.compute(18.0);
    EXPECT_NEAR(measured, 18.0 * 1.05, 1e-6);

    // Test at max speed
    measured = model.compute(30.0);
    EXPECT_NEAR(measured, 30.0 * 1.05, 1e-6);
}

TEST(SpoolSensor, NegativeVelocityClamp) {
    SpoolModel model;
    model.noise_stddev = 0.0;
    model.noise_dist = std::normal_distribution<double>(0.0, 0.0);

    // Negative velocity (shouldn't happen, but handle it)
    double measured = model.compute(-5.0);

    // After clamping, should be 0
    EXPECT_EQ(measured, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
