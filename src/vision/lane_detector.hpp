#pragma once
#include <opencv2/core.hpp>

namespace our_car {
// IMAGE coordinates: y=m*x+b. Independent of GroundLine and its centimetre units.
struct ImageLine { bool valid = false; double m = 0, b = 0; };
struct LaneObservation {
    bool frame_valid = false;
    ImageLine left, right;
};
struct LaneConfig {
    int hough_threshold = 15;
    double min_length = 55;
    double min_abs_slope = 0.25;
};
class LaneDetector {
public:
    explicit LaneDetector(const LaneConfig& config = LaneConfig());
    LaneObservation detect(const cv::Mat& edges) const;
private:
    LaneConfig config_;
};
}  // namespace our_car
