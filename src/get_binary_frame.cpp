#include "get_binary_frame.hpp"

const cv::Size const_size(5, 5);
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::Mat bin_temp, mask;

void get_binary_frame_track(cv::Mat &input_frame, cv::Mat &bin_frame){
    cv::cvtColor(input_frame, bin_temp, cv::COLOR_BGR2GRAY);

    //    clahe->apply(bin_temp, bin_temp);
    cv::GaussianBlur(bin_temp, bin_temp, const_size, 0.52);
    cv::Canny(bin_temp, bin_temp, 50, 140);
    cv::dilate(bin_temp, bin_frame, kernel, cv::Point(-1, -1), 1);
}
