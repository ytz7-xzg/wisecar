#pragma once
#include <opencv2/core.hpp>

namespace our_car {
class Undistorter {
public:
    // K/D must be calibrated for this exact image size; no legacy hardcoded calibration.
    Undistorter(const cv::Mat& camera_matrix, const cv::Mat& distortion,
                const cv::Size& calibrated_size);
    bool apply(const cv::Mat& input, cv::Mat& output) const;
private:
    cv::Size size_;
    cv::Mat map_x_, map_y_;
};
}  // namespace our_car
