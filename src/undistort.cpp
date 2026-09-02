#include "undistort.hpp"

cv::Mat K = (cv::Mat_<double>(3, 3) << 156.8597960958496, 0, 156.5480777481064,
            0.0, 156.6560327205041, 127.6987415552855,
            0.0, 0.0, 1.0),
        D = (cv::Mat_<double>(1, 5) << -0.3495242471365975, 0.118682129385989, -0.0008891403272969765, 0.0008244924389640646, -0.01736317626728643),
        mapx, mapy,
        undistortedFrame, empty;

void undistort(const cv::Mat &frame) {
    cv::initUndistortRectifyMap(K, D, empty, K, frame.size(), CV_32FC1, mapx, mapy);
    cv::remap(frame, frame, mapx, mapy, cv::INTER_LINEAR);
}
