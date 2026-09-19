#include "vision/lane_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace our_car {
LaneDetector::LaneDetector(const LaneConfig& config) : config_(config) {
    if (config.hough_threshold <= 0 || !std::isfinite(config.min_length) ||
        config.min_length <= 0 || !std::isfinite(config.min_abs_slope) ||
        config.min_abs_slope < 0) throw std::invalid_argument("Invalid Hough configuration");
}
LaneObservation LaneDetector::detect(const cv::Mat& edges) const {
    LaneObservation result;  // Every call starts invalid; never reuse a lost boundary.
    if (edges.empty() || edges.type() != CV_8UC1) return result;
    result.frame_valid = true;
    std::vector<cv::Vec4i> segments;
    cv::HoughLinesP(edges,segments,1,std::acos(-1.0)/180.0,
                   config_.hough_threshold,config_.min_length);
    double weight[2] = {0,0}, slopes[2] = {0,0}, intercepts[2] = {0,0};
    for (const cv::Vec4i& s : segments) {
        const double dx = double(s[2])-s[0], dy = double(s[3])-s[1];
        if (std::abs(dx) < 3 || std::abs(dy) < 3) continue;
        const double m = dy/dx;
        if (std::abs(m) <= config_.min_abs_slope) continue;
        const int side = m < 0 ? 0 : 1;
        const double length = std::hypot(dx,dy);
        weight[side] += length;
        slopes[side] += m*length;
        intercepts[side] += (double(s[1])-m*s[0])*length;
    }
    ImageLine* lines[2] = {&result.left,&result.right};
    for (int side=0;side<2;++side) {
        if (weight[side] <= 0) continue;
        lines[side]->m = slopes[side]/weight[side];
        lines[side]->b = intercepts[side]/weight[side];
        lines[side]->valid = std::isfinite(lines[side]->m) && std::isfinite(lines[side]->b);
    }
    return result;
}
}  // namespace our_car
