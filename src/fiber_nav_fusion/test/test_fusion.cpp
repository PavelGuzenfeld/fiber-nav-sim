#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "../../../third_party/doctest.h"

#include <cmath>

namespace {

// Simplified quaternion operations for testing
struct Quaternion {
    double w, x, y, z;

    static Quaternion from_euler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll / 2);
        double sr = std::sin(roll / 2);
        double cp = std::cos(pitch / 2);
        double sp = std::sin(pitch / 2);
        double cy = std::cos(yaw / 2);
        double sy = std::sin(yaw / 2);

        return {
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        };
    }

    Quaternion conjugate() const {
        return {w, -x, -y, -z};
    }
};

struct Vector3 {
    double x, y, z;

    double norm() const { return std::sqrt(x*x + y*y + z*z); }

    Vector3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vector3 operator+(const Vector3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vector3 operator-(const Vector3& v) const { return {x-v.x, y-v.y, z-v.z}; }
};

// Rotate vector by quaternion: q * v * q^(-1)
Vector3 rotate(const Quaternion& q, const Vector3& v) {
    // Convert vector to quaternion
    Quaternion p = {0, v.x, v.y, v.z};

    // q * p
    Quaternion qp = {
        q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
        q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
        q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
        q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
    };

    // (q * p) * q^(-1)
    Quaternion qc = q.conjugate();
    Quaternion result = {
        qp.w*qc.w - qp.x*qc.x - qp.y*qc.y - qp.z*qc.z,
        qp.w*qc.x + qp.x*qc.w + qp.y*qc.z - qp.z*qc.y,
        qp.w*qc.y - qp.x*qc.z + qp.y*qc.w + qp.z*qc.x,
        qp.w*qc.z + qp.x*qc.y - qp.y*qc.x + qp.z*qc.w
    };

    return {result.x, result.y, result.z};
}

// Replicate fusion math
struct FusionModel {
    double slack_factor = 1.05;

    Vector3 fuse(double spool_velocity, Vector3 direction, Quaternion attitude) {
        // Reconstruct body velocity
        double corrected_speed = spool_velocity / slack_factor;
        Vector3 v_body = direction * corrected_speed;

        // Rotate to NED
        Vector3 v_ned = rotate(attitude, v_body);

        return v_ned;
    }
};

}  // namespace

TEST_CASE("Fusion.VelocityReconstruction") {
    FusionModel model;
    model.slack_factor = 1.0;  // No slack correction

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};  // Forward
    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};  // Identity (level, north)

    Vector3 result = model.fuse(spool, direction, attitude);

    CHECK(result.x == doctest::Approx(10.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.SlackCorrection") {
    FusionModel model;
    model.slack_factor = 1.05;

    double spool = 10.5;  // Measured with slack
    Vector3 direction = {1.0, 0.0, 0.0};
    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.fuse(spool, direction, attitude);

    // 10.5 / 1.05 = 10.0
    CHECK(result.x == doctest::Approx(10.0).epsilon(1e-6));
}

TEST_CASE("Fusion.FrameRotation90Yaw") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};  // Forward in body

    // 90° yaw (heading East)
    Quaternion attitude = Quaternion::from_euler(0, 0, M_PI / 2);

    Vector3 result = model.fuse(spool, direction, attitude);

    // Body X (forward) should become NED Y (East)
    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(10.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.FrameRotation180Yaw") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};

    // 180° yaw (heading South)
    Quaternion attitude = Quaternion::from_euler(0, 0, M_PI);

    Vector3 result = model.fuse(spool, direction, attitude);

    // Body X should become -NED X (South = -North)
    CHECK(result.x == doctest::Approx(-10.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.FrameRotationPitchDown") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    Vector3 direction = {1.0, 0.0, 0.0};

    // 45° pitch down (negative pitch in aerospace convention)
    Quaternion attitude = Quaternion::from_euler(0, -M_PI / 4, 0);

    Vector3 result = model.fuse(spool, direction, attitude);

    double expected = 10.0 / std::sqrt(2.0);

    // Forward velocity should split into North and Down
    CHECK(result.x == doctest::Approx(expected).epsilon(1e-6));  // North
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));       // East
    CHECK(result.z == doctest::Approx(expected).epsilon(1e-6));  // Down (positive in NED)
}

TEST_CASE("Fusion.DiagonalDirection") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 10.0;
    // 45° to the right in body frame
    double sqrt2 = std::sqrt(2.0);
    Vector3 direction = {1.0/sqrt2, 1.0/sqrt2, 0.0};

    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};  // Identity

    Vector3 result = model.fuse(spool, direction, attitude);

    double expected = 10.0 / sqrt2;
    CHECK(result.x == doctest::Approx(expected).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(expected).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.ZeroVelocity") {
    FusionModel model;

    double spool = 0.0;
    Vector3 direction = {1.0, 0.0, 0.0};
    Quaternion attitude = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.fuse(spool, direction, attitude);

    CHECK(result.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("Fusion.VelocityMagnitudePreserved") {
    FusionModel model;
    model.slack_factor = 1.0;

    double spool = 15.0;
    Vector3 direction = {0.6, 0.8, 0.0};  // Unit vector

    // Random-ish attitude
    Quaternion attitude = Quaternion::from_euler(0.1, 0.2, 0.3);

    Vector3 result = model.fuse(spool, direction, attitude);

    // Rotation should preserve magnitude
    CHECK(result.norm() == doctest::Approx(15.0).epsilon(1e-6));
}
