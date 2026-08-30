#include "check_cones.hpp"
#include "utils.hpp"
#include <string>

std::vector<std::vector<cv::Point>> cnts;
cv::Size imageSize;
float minRadius = 3.0f;
static int cone_save_counter = 0;
static int s_low_cached = 130;
static int v_low_cached = 230;
static int64 last_update_tick = 0;
static double tick_freq = 0.0;

void cones_image_process(const cv::Mat& lowerRegion, int& check_count) {
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(lowerRegion, hsv, cv::COLOR_BGR2HSV);
    if (tick_freq == 0.0) tick_freq = cv::getTickFrequency();
    int64 now = cv::getTickCount();
    if (last_update_tick == 0 && cone_weather_mode == 1) { s_low_cached = 110; v_low_cached = 220; }
    if (last_update_tick == 0 || (double)(now - last_update_tick) / tick_freq > 0.2) {
        cv::Scalar m = cv::mean(hsv);
        int s_target, v_target;
        if (cone_weather_mode == 1) {
            s_target = std::max(110, (int)(m[1] * 0.72));
            v_target = std::max(220, (int)(m[2] * 0.88));
        } else {
            s_target = std::max(130, (int)(m[1] * 0.78));
            v_target = std::max(230, (int)(m[2] * 0.90));
        }
        s_low_cached = int(0.6 * s_low_cached + 0.4 * s_target);
        v_low_cached = int(0.6 * v_low_cached + 0.4 * v_target);
        last_update_tick = now;
    }
    cv::inRange(hsv, cv::Scalar(94, s_low_cached, v_low_cached), cv::Scalar(118, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(118, s_low_cached, v_low_cached), cv::Scalar(135, 255, 255), mask2);
    cv::Mat mask3;
    cv::inRange(hsv, cv::Scalar(100, 115, 248), cv::Scalar(110, 255, 255), mask3);
    cv::Mat mask4;
    cv::inRange(hsv, cv::Scalar(101, 130, 180), cv::Scalar(108, 255, 255), mask4);
    cv::Mat mask5;
    if (cone_weather_mode == 1) {
        cv::inRange(hsv, cv::Scalar(98, 70, 190), cv::Scalar(116, 170, 255), mask5);
    } else {
        cv::inRange(hsv, cv::Scalar(99, 85, 200), cv::Scalar(114, 180, 255), mask5);
    }
    mask = mask1 | mask2 | mask3 | mask4 | mask5;
    int erode_iters = (cone_weather_mode == 1 ? 1 : 2);
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), erode_iters);
    cv::GaussianBlur(mask, mask, cv::Size(3, 3), 0);

    std::vector<std::vector<cv::Point>> cnts;
    cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    check_count = 0;
    cv::Mat vis = lowerRegion.clone();
    for (const auto& contour : cnts) {
        double area = cv::contourArea(contour);
        if (area < cone_blue_min_area || area > cone_blue_max_area) continue;
        cv::RotatedRect rr = cv::minAreaRect(contour);
        double w = rr.size.width, h = rr.size.height;
        if (w <= 1.0 || h <= 1.0) continue;
        double ratio = (h > w) ? (h / w) : (w / h);
        if (ratio >= 0.7 && ratio <= 3) {
            check_count++;
            cv::Point2f pts[4]; rr.points(pts);
            for (int i = 0; i < 4; ++i) cv::line(vis, pts[i], pts[(i+1)%4], cv::Scalar(0, 255, 0), 2);
        }
    }
    if (check_count > 0) {
        // DEBUG_AVOID: 保存圈出蓝色锥桶的截图，便于调试与后期注释
        std::string fname = std::string("debug_avoid/cone_") + std::to_string(cone_save_counter++) + std::string(".png");
        try { cv::imwrite(fname, vis); } catch (...) {}
        // DEBUG_AVOID END
    }
}

 
