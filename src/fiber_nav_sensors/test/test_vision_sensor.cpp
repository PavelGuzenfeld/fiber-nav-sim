#include <gtest/gtest.h>
#include <random>
#include <cmath>
#include <array>

namespace {

// Replicate vision sensor math for testing
struct VisionModel {
    double drift_rate = 0.001;  // rad/s
    double min_velocity = 0.5;

    std::mt19937 rng{42};
    std::normal_distribution<double> drift_dist{0.0, drift_rate};

    double drift_yaw = 0.0;
    double drift_pitch = 0.0;

    struct Vector3 {
        double x, y, z;
        double norm() const { return std::sqrt(x*x + y*y + z*z); }
        Vector3 normalized() const {
            double n = norm();
            return {x/n, y/n, z/n};
        }
    };

    std::pair<bool, Vector3> compute(Vector3 true_velocity, double dt) {
        double speed = true_velocity.norm();

        // Reject low velocity
        if (speed < min_velocity) {
            return {false, {}};
        }

        // Normalize
        Vector3 direction = true_velocity.normalized();

        // Apply drift
        drift_yaw += drift_dist(rng) * std::sqrt(dt);
        drift_pitch += drift_dist(rng) * std::sqrt(dt);

        double cos_yaw = std::cos(drift_yaw);
        double sin_yaw = std::sin(drift_yaw);
        double cos_pitch = std::cos(drift_pitch);
        double sin_pitch = std::sin(drift_pitch);

        // Rotate
        double ux_yaw = cos_yaw * direction.x - sin_yaw * direction.y;
        double uy_yaw = sin_yaw * direction.x + cos_yaw * direction.y;
        double uz_yaw = direction.z;

        double ux_final = cos_pitch * ux_yaw + sin_pitch * uz_yaw;
        double uy_final = uy_yaw;
        double uz_final = -sin_pitch * ux_yaw + cos_pitch * uz_yaw;

        Vector3 result = {ux_final, uy_final, uz_final};
        result = result.normalized();

        return {true, result};
    }
};

}  // namespace

TEST(VisionSensor, UnitVector) {
    VisionModel model;

    VisionModel::Vector3 true_vel = {10.0, 5.0, -2.0};
    auto [valid, direction] = model.compute(true_vel, 0.02);

    ASSERT_TRUE(valid);

    // Must be unit vector (magnitude = 1)
    EXPECT_NEAR(direction.norm(), 1.0, 1e-6);
}

TEST(VisionSensor, DirectionPreserved) {
    VisionModel model;
    model.drift_rate = 0.0;  // No drift
    model.drift_dist = std::normal_distribution<double>(0.0, 0.0);

    VisionModel::Vector3 true_vel = {10.0, 0.0, 0.0};
    auto [valid, direction] = model.compute(true_vel, 0.02);

    ASSERT_TRUE(valid);

    // Without drift, direction should match normalized velocity
    EXPECT_NEAR(direction.x, 1.0, 1e-6);
    EXPECT_NEAR(direction.y, 0.0, 1e-6);
    EXPECT_NEAR(direction.z, 0.0, 1e-6);
}

TEST(VisionSensor, LowVelocityReject) {
    VisionModel model;

    // Below threshold
    VisionModel::Vector3 low_vel = {0.1, 0.1, 0.0};
    auto [valid, direction] = model.compute(low_vel, 0.02);

    EXPECT_FALSE(valid);
}

TEST(VisionSensor, DriftAccumulation) {
    VisionModel model;
    model.rng.seed(12345);

    VisionModel::Vector3 true_vel = {10.0, 0.0, 0.0};
    VisionModel::Vector3 initial_dir;
    VisionModel::Vector3 final_dir;

    // First sample
    {
        auto [valid, dir] = model.compute(true_vel, 0.02);
        ASSERT_TRUE(valid);
        initial_dir = dir;
    }

    // Simulate 1000 seconds of drift
    for (int i = 0; i < 50000; ++i) {  // 50000 * 0.02s = 1000s
        auto [valid, dir] = model.compute(true_vel, 0.02);
        ASSERT_TRUE(valid);
        final_dir = dir;
    }

    // Calculate angular difference
    double dot = initial_dir.x * final_dir.x +
                 initial_dir.y * final_dir.y +
                 initial_dir.z * final_dir.z;
    double angle = std::acos(std::clamp(dot, -1.0, 1.0));

    // After 1000s with drift_rate=0.001 rad/s, expect ~√1000 * 0.001 ≈ 0.032 rad
    // But this is random walk, so allow wide tolerance
    EXPECT_LT(angle, 0.5);  // Should be less than ~30 degrees
    EXPECT_GT(angle, 0.001); // Should have some drift
}

TEST(VisionSensor, DiagonalVelocity) {
    VisionModel model;
    model.drift_rate = 0.0;
    model.drift_dist = std::normal_distribution<double>(0.0, 0.0);

    VisionModel::Vector3 true_vel = {10.0, 10.0, 0.0};
    auto [valid, direction] = model.compute(true_vel, 0.02);

    ASSERT_TRUE(valid);

    double expected = 1.0 / std::sqrt(2.0);
    EXPECT_NEAR(direction.x, expected, 1e-6);
    EXPECT_NEAR(direction.y, expected, 1e-6);
    EXPECT_NEAR(direction.z, 0.0, 1e-6);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
