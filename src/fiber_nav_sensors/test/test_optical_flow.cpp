#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <fiber_nav_sensors/optical_flow_direction.hpp>

#include <cmath>
#include <numeric>
#include <vector>

using namespace fiber_nav_sensors;

// --- compute_dominant_flow tests ---

TEST_CASE("OpticalFlow.EmptyInput") {
    std::vector<cv::Point2f> prev, curr;
    std::vector<uchar> status;

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    CHECK(result.quality == doctest::Approx(0.0f));
    CHECK(result.magnitude == doctest::Approx(0.0f));
}

TEST_CASE("OpticalFlow.AllLost") {
    std::vector<cv::Point2f> prev = {{10, 10}, {20, 20}, {30, 30}};
    std::vector<cv::Point2f> curr = {{15, 10}, {25, 20}, {35, 30}};
    std::vector<uchar> status = {0, 0, 0};  // all lost

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    CHECK(result.quality == doctest::Approx(0.0f));
}

TEST_CASE("OpticalFlow.UniformRightwardFlow") {
    // All points move 5px right, 0px down
    std::vector<cv::Point2f> prev = {{10, 10}, {20, 20}, {30, 30}, {40, 40}, {50, 50}};
    std::vector<cv::Point2f> curr = {{15, 10}, {25, 20}, {35, 30}, {45, 40}, {55, 50}};
    std::vector<uchar> status = {1, 1, 1, 1, 1};

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    CHECK(result.quality == doctest::Approx(1.0f));
    CHECK(result.dir_x == doctest::Approx(5.0f));
    CHECK(result.dir_y == doctest::Approx(0.0f));
    CHECK(result.magnitude == doctest::Approx(5.0f));
}

TEST_CASE("OpticalFlow.DiagonalFlow") {
    // All points move 3px right, 4px down
    std::vector<cv::Point2f> prev = {{10, 10}, {20, 20}, {30, 30}};
    std::vector<cv::Point2f> curr = {{13, 14}, {23, 24}, {33, 34}};
    std::vector<uchar> status = {1, 1, 1};

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    CHECK(result.quality == doctest::Approx(1.0f));
    CHECK(result.dir_x == doctest::Approx(3.0f));
    CHECK(result.dir_y == doctest::Approx(4.0f));
    CHECK(result.magnitude == doctest::Approx(5.0f));
}

TEST_CASE("OpticalFlow.PartialTracking") {
    // 5 points, 3 tracked, 2 lost
    std::vector<cv::Point2f> prev = {{10, 10}, {20, 20}, {30, 30}, {40, 40}, {50, 50}};
    std::vector<cv::Point2f> curr = {{15, 10}, {20, 20}, {35, 30}, {40, 40}, {55, 50}};
    std::vector<uchar> status = {1, 0, 1, 0, 1};

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    CHECK(result.quality == doctest::Approx(3.0f / 5.0f));
    CHECK(result.dir_x == doctest::Approx(5.0f));
    CHECK(result.dir_y == doctest::Approx(0.0f));
}

TEST_CASE("OpticalFlow.OutlierRobustness") {
    // 5 uniform points + 2 outliers — median should reject outliers
    std::vector<cv::Point2f> prev = {
        {10, 10}, {20, 20}, {30, 30}, {40, 40}, {50, 50}, {60, 60}, {70, 70}};
    std::vector<cv::Point2f> curr = {
        {15, 10}, {25, 20}, {35, 30}, {45, 40}, {55, 50},
        {60, 160}, {170, 70}};  // last 2 are outliers
    std::vector<uchar> status = {1, 1, 1, 1, 1, 1, 1};

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    // Median of flow_x: [5, 5, 5, 5, 5, 0, 100] → sorted: [0, 5, 5, 5, 5, 5, 100] → median=5
    CHECK(result.dir_x == doctest::Approx(5.0f));
    // Median of flow_y: [0, 0, 0, 0, 0, 100, 0] → sorted: [0, 0, 0, 0, 0, 0, 100] → median=0
    CHECK(result.dir_y == doctest::Approx(0.0f));
}

TEST_CASE("OpticalFlow.BelowMinDisplacement") {
    // Tiny sub-pixel flow
    std::vector<cv::Point2f> prev = {{10, 10}, {20, 20}, {30, 30}};
    std::vector<cv::Point2f> curr = {{10.1f, 10.1f}, {20.1f, 20.1f}, {30.1f, 30.1f}};
    std::vector<uchar> status = {1, 1, 1};

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    // Quality is still 1.0 (all tracked), but magnitude is small
    CHECK(result.quality == doctest::Approx(1.0f));
    CHECK(result.magnitude < 0.5f);
}

TEST_CASE("OpticalFlow.SizeMismatch") {
    std::vector<cv::Point2f> prev = {{10, 10}, {20, 20}};
    std::vector<cv::Point2f> curr = {{15, 10}};  // wrong size
    std::vector<uchar> status = {1, 1};

    auto result = compute_dominant_flow(prev, curr, status, 0.5f);

    CHECK(result.quality == doctest::Approx(0.0f));
}

// --- flow_to_body_direction tests ---

TEST_CASE("BodyDirection.RightwardFlow") {
    // Image +X (right) → body +Y
    auto dir = flow_to_body_direction(5.0f, 0.0f);

    CHECK(dir[0] == doctest::Approx(0.0f));
    CHECK(dir[1] == doctest::Approx(1.0f));
    CHECK(dir[2] == doctest::Approx(0.0f));
}

TEST_CASE("BodyDirection.DownwardFlow") {
    // Image +Y (down) → body +Z
    auto dir = flow_to_body_direction(0.0f, 5.0f);

    CHECK(dir[0] == doctest::Approx(0.0f));
    CHECK(dir[1] == doctest::Approx(0.0f));
    CHECK(dir[2] == doctest::Approx(1.0f));
}

TEST_CASE("BodyDirection.DiagonalFlow") {
    auto dir = flow_to_body_direction(3.0f, 4.0f);

    CHECK(dir[0] == doctest::Approx(0.0f));
    CHECK(dir[1] == doctest::Approx(3.0f / 5.0f));
    CHECK(dir[2] == doctest::Approx(4.0f / 5.0f));

    // Must be unit vector (in YZ plane)
    float norm = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    CHECK(norm == doctest::Approx(1.0f));
}

TEST_CASE("BodyDirection.ZeroFlow") {
    auto dir = flow_to_body_direction(0.0f, 0.0f);

    CHECK(dir[0] == doctest::Approx(0.0f));
    CHECK(dir[1] == doctest::Approx(0.0f));
    CHECK(dir[2] == doctest::Approx(0.0f));
}

TEST_CASE("BodyDirection.NegativeFlow") {
    // Leftward image flow → body -Y
    auto dir = flow_to_body_direction(-5.0f, 0.0f);

    CHECK(dir[0] == doctest::Approx(0.0f));
    CHECK(dir[1] == doctest::Approx(-1.0f));
    CHECK(dir[2] == doctest::Approx(0.0f));
}
