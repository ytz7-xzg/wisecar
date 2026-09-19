#pragma once
#include <opencv2/core.hpp>

namespace our_car {
// Input: CV_8UC3 BGR. Output: CV_8UC1 edge mask; cleared on invalid input.
bool road_edges(const cv::Mat& bgr, cv::Mat& edges);
}  // namespace our_car
