#include "vision/preprocess.hpp"
#include <opencv2/imgproc.hpp>

namespace our_car {
bool road_edges(const cv::Mat& bgr, cv::Mat& edges) {
    if (bgr.empty() || bgr.type() != CV_8UC3) { edges.release(); return false; }
    cv::Mat gray, output;
    cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray,gray,cv::Size(5,5),0.52);
    cv::Canny(gray,gray,50,140);
    cv::dilate(gray,output,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)));
    edges = output;
    return true;
}
}  // namespace our_car
