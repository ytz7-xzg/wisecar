#include "vision/preprocess.hpp"
#include "vision/blue_barrier.hpp"
#include "vision/lane_detector.hpp"
#include "vision/undistorter.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>

using namespace our_car;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

void barrier_inputs() {
    CHECK(!detect_blue_barrier(cv::Mat()).valid);
    CHECK(!detect_blue_barrier(cv::Mat::zeros(20,20,CV_8UC1)).valid);
    const cv::Mat blue(100,100,CV_8UC3,cv::Scalar(255,0,0));
    BarrierResult r = detect_blue_barrier(blue);
    CHECK(r.valid && r.present && r.blue_ratio > 0.99);
    r = detect_blue_barrier(cv::Mat::zeros(100,100,CV_8UC3));
    CHECK(r.valid && !r.present && r.blue_ratio == 0);
    cv::Mat partial = cv::Mat::zeros(100,100,CV_8UC3);
    partial(cv::Rect(0,0,80,100)).setTo(cv::Scalar(255,0,0));
    r = detect_blue_barrier(partial); CHECK(r.valid && r.present);
}
void lane_observations_are_fresh() {
    LaneDetector detector;
    cv::Mat edge = cv::Mat::zeros(240,320,CV_8UC1);
    cv::line(edge,cv::Point(20,220),cv::Point(130,30),cv::Scalar(255),2);
    cv::line(edge,cv::Point(300,220),cv::Point(190,30),cv::Scalar(255),2);
    LaneObservation r = detector.detect(edge);
    CHECK(r.frame_valid && r.left.valid && r.right.valid);
    CHECK(r.left.m < 0 && r.right.m > 0);
    r = detector.detect(cv::Mat::zeros(240,320,CV_8UC1));
    CHECK(r.frame_valid && !r.left.valid && !r.right.valid);
    CHECK(!detector.detect(cv::Mat()).frame_valid);
}
void preprocessing_and_undistortion() {
    cv::Mat output(2,2,CV_8UC1,cv::Scalar(255));
    CHECK(!road_edges(cv::Mat(),output)); CHECK(output.empty());
    cv::Mat input(48,64,CV_8UC3,cv::Scalar(10,20,30));
    cv::Mat before = input.clone();
    CHECK(road_edges(input,output)); CHECK(output.type() == CV_8UC1);
    const cv::Mat k = (cv::Mat_<double>(3,3) << 100,0,32,0,100,24,0,0,1);
    Undistorter u(k,cv::Mat::zeros(1,5,CV_64F),input.size());
    CHECK(u.apply(input,output)); CHECK(input.data != output.data);
    CHECK(cv::norm(input,before,cv::NORM_INF) == 0);
    CHECK(cv::norm(output,input,cv::NORM_INF) == 0);
    CHECK(u.apply(input,input)); CHECK(cv::norm(input,before,cv::NORM_INF) == 0);
    CHECK(!u.apply(cv::Mat::zeros(24,32,CV_8UC3),output)); CHECK(output.empty());
}
int main() {
    try {
        barrier_inputs(); lane_observations_are_fresh(); preprocessing_and_undistortion();
        std::cout << "PASS: 3 vision test groups\n";
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
