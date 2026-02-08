#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

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

// --- ZUPT model tests ---

struct ZuptModel {
    double threshold = 0.05;
    double zupt_variance = 0.001;
    double normal_variance = 0.01;
    double slack_factor = 1.05;

    bool is_zupt(double speed) const { return speed < threshold; }

    Vector3 velocity(double speed, Vector3 dir, Quaternion att) const {
        if (is_zupt(speed)) return {0, 0, 0};
        double corrected = speed / slack_factor;
        Vector3 v_body = dir * corrected;
        return rotate(att, v_body);
    }
};

TEST_CASE("ZUPT.ZeroWhenStopped") {
    ZuptModel model;
    Vector3 dir = {1.0, 0.0, 0.0};
    Quaternion att = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.velocity(0.01, dir, att);
    CHECK(result.x == doctest::Approx(0.0));
    CHECK(result.y == doctest::Approx(0.0));
    CHECK(result.z == doctest::Approx(0.0));
}

TEST_CASE("ZUPT.NormalWhenMoving") {
    ZuptModel model;
    Vector3 dir = {1.0, 0.0, 0.0};
    Quaternion att = {1.0, 0.0, 0.0, 0.0};

    Vector3 result = model.velocity(5.0, dir, att);
    double expected = 5.0 / 1.05;
    CHECK(result.x == doctest::Approx(expected).epsilon(1e-6));
    CHECK(result.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(result.z == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("ZUPT.ThresholdBoundary") {
    ZuptModel model;
    Vector3 dir = {1.0, 0.0, 0.0};
    Quaternion att = {1.0, 0.0, 0.0, 0.0};

    // Just below threshold → ZUPT
    Vector3 below = model.velocity(0.04999, dir, att);
    CHECK(below.x == doctest::Approx(0.0));

    // Just above threshold → normal
    Vector3 above = model.velocity(0.05001, dir, att);
    CHECK(above.x > 0.0);
}

TEST_CASE("ZUPT.IndependentOfVisionDirection") {
    ZuptModel model;

    // ZUPT should produce zeros regardless of direction vector value
    // This models the case where vision direction is stale/unavailable
    Vector3 stale_dir = {0.5, 0.5, 0.707};  // Arbitrary stale direction
    Quaternion att = Quaternion::from_euler(0.1, 0.2, 0.3);  // Arbitrary attitude

    // Speed below threshold → ZUPT ignores direction entirely
    Vector3 result = model.velocity(0.01, stale_dir, att);
    CHECK(result.x == doctest::Approx(0.0));
    CHECK(result.y == doctest::Approx(0.0));
    CHECK(result.z == doctest::Approx(0.0));

    // Even with zero direction (no vision), ZUPT still works
    Vector3 no_dir = {0.0, 0.0, 0.0};
    result = model.velocity(0.0, no_dir, att);
    CHECK(result.x == doctest::Approx(0.0));
    CHECK(result.y == doctest::Approx(0.0));
    CHECK(result.z == doctest::Approx(0.0));
}

// --- DragBow model tests ---

struct DragBowModel {
    double k_drag = 0.0005;
    double tunnel_heading_deg = 90.0;
    double var_long = 1.0;
    double var_lat = 100.0;

    Vector3 position(double total_length, double speed) const {
        double x_est = total_length * (1.0 - k_drag * speed * speed);
        double h = tunnel_heading_deg * M_PI / 180.0;
        return {x_est * std::cos(h), x_est * std::sin(h), 0.0};
    }

    Vector3 variance() const {
        double h = tunnel_heading_deg * M_PI / 180.0;
        double cos_h = std::cos(h);
        double sin_h = std::sin(h);
        return {
            var_long * cos_h * cos_h + var_lat * sin_h * sin_h,
            var_long * sin_h * sin_h + var_lat * cos_h * cos_h,
            1e6
        };
    }
};

TEST_CASE("DragBow.StaticPosition") {
    DragBowModel model;
    model.tunnel_heading_deg = 0.0;  // North

    Vector3 pos = model.position(100.0, 0.0);
    CHECK(pos.x == doctest::Approx(100.0).epsilon(1e-6));  // North
    CHECK(pos.y == doctest::Approx(0.0).epsilon(1e-6));     // East
}

TEST_CASE("DragBow.DragCorrection") {
    DragBowModel model;
    model.tunnel_heading_deg = 0.0;

    // L=100, v=10, k=0.0005 → X_est = 100*(1 - 0.05) = 95
    Vector3 pos = model.position(100.0, 10.0);
    CHECK(pos.x == doctest::Approx(95.0).epsilon(1e-6));
}

TEST_CASE("DragBow.TunnelEast") {
    DragBowModel model;
    model.tunnel_heading_deg = 90.0;

    Vector3 pos = model.position(100.0, 0.0);
    CHECK(pos.x == doctest::Approx(0.0).epsilon(1e-6));     // North ≈ 0
    CHECK(pos.y == doctest::Approx(100.0).epsilon(1e-6));    // East = X_est
}

TEST_CASE("DragBow.TunnelNorth") {
    DragBowModel model;
    model.tunnel_heading_deg = 0.0;

    Vector3 pos = model.position(100.0, 0.0);
    CHECK(pos.x == doctest::Approx(100.0).epsilon(1e-6));   // North = X_est
    CHECK(pos.y == doctest::Approx(0.0).epsilon(1e-6));      // East ≈ 0
}

TEST_CASE("DragBow.VarianceEast") {
    DragBowModel model;
    model.tunnel_heading_deg = 90.0;

    Vector3 var = model.variance();
    // heading=90°: cos=0, sin=1
    // var_n = long*0 + lat*1 = 100 (lateral)
    // var_e = long*1 + lat*0 = 1 (longitudinal)
    CHECK(var.x == doctest::Approx(100.0).epsilon(1e-6));  // North = lateral
    CHECK(var.y == doctest::Approx(1.0).epsilon(1e-6));    // East = longitudinal
    CHECK(var.z == doctest::Approx(1e6).epsilon(1.0));
}
