#pragma once
#include <opencv2/core.hpp>

namespace our_car {
struct BarrierConfig {
    cv::Scalar lower = cv::Scalar(95,30,40);
    cv::Scalar upper = cv::Scalar(135,255,255);
    double min_blue_ratio = 0.60;
};
struct BarrierResult {
    bool valid = false;
    bool present = false;
    double blue_ratio = 0;
};
// Caller supplies the frame or chosen ROI. Invalid frame != absent barrier.
BarrierResult detect_blue_barrier(const cv::Mat& bgr,
                                 const BarrierConfig& config = BarrierConfig());
}  // namespace our_car
