#include "blue_card_find.hpp"

cv::Scalar lowerBlue(95, 30, 40);
cv::Scalar upperBlue(135, 255, 255);

bool blue_card_find(const cv::Mat& frame) {
    cv::Size imageSize = frame.size();

    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, lowerBlue, upperBlue, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::GaussianBlur(mask, mask, cv::Size(3, 3), 0);

    double bluePixels = countNonZero(mask);
    double totalPixels = mask.rows * mask.cols;
    double blueRatio = bluePixels / totalPixels;
    std::cout << "蓝色区域占比: " << blueRatio << std::endl;
    if (blueRatio < 0.60) {
        return false;
    }
    return true;

}
