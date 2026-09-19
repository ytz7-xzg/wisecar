#include "vision/undistorter.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace our_car {
Undistorter::Undistorter(const cv::Mat& camera_matrix, const cv::Mat& distortion,
                         const cv::Size& calibrated_size) : size_(calibrated_size) {
    const std::size_t n = distortion.total();
    if (size_.width <= 0 || size_.height <= 0 || camera_matrix.rows != 3 ||
        camera_matrix.cols != 3 || camera_matrix.channels() != 1 ||
        distortion.channels() != 1 || (distortion.rows != 1 && distortion.cols != 1) ||
        !(n == 4 || n == 5 || n == 8 || n == 12 || n == 14))
        throw std::invalid_argument("Invalid camera calibration shape");
    cv::Mat k,d; camera_matrix.convertTo(k,CV_64F); distortion.convertTo(d,CV_64F);
    if (!cv::checkRange(k) || !cv::checkRange(d) || k.at<double>(0,0) <= 0 ||
        k.at<double>(1,1) <= 0 || k.at<double>(2,2) != 1)
        throw std::invalid_argument("Invalid camera calibration values");
    cv::initUndistortRectifyMap(k,d,cv::Mat(),k,size_,CV_32FC1,map_x_,map_y_);
}
bool Undistorter::apply(const cv::Mat& input, cv::Mat& output) const {
    if (input.empty() || input.size() != size_ ||
        (input.type() != CV_8UC1 && input.type() != CV_8UC3)) {
        output.release(); return false;
    }
    cv::Mat corrected;
    cv::remap(input,corrected,map_x_,map_y_,cv::INTER_LINEAR);
    output = corrected;  // Handles even apply(frame,frame) without in-place remap.
    return true;
}
}  // namespace our_car
