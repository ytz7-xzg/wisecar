#include "vision/blue_barrier.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace our_car {
BarrierResult detect_blue_barrier(const cv::Mat& bgr, const BarrierConfig& config) {
    if (!std::isfinite(config.min_blue_ratio) || config.min_blue_ratio <= 0 ||
        config.min_blue_ratio > 1) throw std::invalid_argument("Invalid blue ratio");
    for (int i=0;i<3;++i) {
        if (!std::isfinite(config.lower[i]) || !std::isfinite(config.upper[i]) ||
            config.lower[i] < 0 || config.upper[i] > (i == 0 ? 179 : 255) ||
            config.lower[i] > config.upper[i]) throw std::invalid_argument("Invalid HSV range");
    }
    BarrierResult result;
    if (bgr.empty() || bgr.type() != CV_8UC3) return result;
    cv::Mat hsv, mask;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,config.lower,config.upper,mask);
    cv::erode(mask,mask,cv::Mat(),cv::Point(-1,-1),2);
    cv::GaussianBlur(mask,mask,cv::Size(3,3),0);
    // Restore binary values so the blur's nonzero halo is not counted as blue.
    cv::threshold(mask,mask,127,255,cv::THRESH_BINARY);
    result.valid = true;
    result.blue_ratio = double(cv::countNonZero(mask))/double(mask.total());
    result.present = result.blue_ratio >= config.min_blue_ratio;
    return result;
}
}  // namespace our_car
