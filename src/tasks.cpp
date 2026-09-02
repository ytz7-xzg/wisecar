#include "tasks.hpp"
#include <string>
#include <ctime>
#include <cstdio>
#include <vector>
#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

pthread_mutex_t mmutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cam0_mutex = PTHREAD_MUTEX_INITIALIZER;
bool check_result = false; // false: left; true: right

// ROI 参数化（百分比）：从底部25%到底部60%，总高约35%
static const double ROI_BOTTOM_START_PCT = 0.35; // 从底部起算的开始位置
static const double ROI_BOTTOM_END_PCT   = 0.65; // 从底部起算的结束位置

// 调试输出控制（由 config.txt 中的 debug_save_images 控制）
static int frame_counter = 0;
static const int save_every_n_frames = 10; // 约每秒保存一次（30FPS）
static const char* debug_dir_canny = "debug_canny";
static const char* debug_dir_lines = "debug_lines";
static const char* debug_dir_crosswalk = "debug_crosswalk";
static const char* debug_dir_direction = "debug_direction";
static const char* debug_dir_stoparea = "debug_stoparea";

static void ensure_dir(const char* path) {
    #if defined(_WIN32)
    _mkdir(path);
    #else
    mkdir(path, 0777);
    #endif
}

static inline bool cam0_read_locked(cv::Mat& out){
    pthread_mutex_lock(&cam0_mutex);
    bool ok = cam0.read(out);
    pthread_mutex_unlock(&cam0_mutex);
    return ok;
}

void *trace(void*) {
    int *ret = new int; *ret = -1;
    pthread_mutex_lock(&mmutex);
    pthread_detach(pthread_self());
    if (!cam2.read(input_frame2) || input_frame2.empty()) {
        std::cout << "Failed to read image from Camera2!" << std::endl;
        pthread_exit((void *)ret);
    }
    pthread_mutex_unlock(&mmutex);
    std::cout << "running task1" << std::endl;
    
    do {
        cam2.read(input_frame2);
        undistort(input_frame2);
        // 按百分比裁剪：自下 25% 到 自下 60%，总高约 35%（带越界保护）
        {
            int rows = input_frame2.rows;
            int cols = input_frame2.cols;
            int y_top = int(rows * (1.0 - ROI_BOTTOM_END_PCT));
            int h = int(rows * (ROI_BOTTOM_END_PCT - ROI_BOTTOM_START_PCT));
            if (y_top < 0) y_top = 0;
            if (h < 0) h = 0;
            if (y_top + h > rows) h = rows - y_top;
            int x = 0;
            int w = cols;
            input_frame2 = cv::Mat(input_frame2, cv::Rect(x, y_top, w, h));
            
        }
        cv::Mat roi_for_draw = input_frame2.clone();
        get_binary_frame_track(input_frame2, binary_frame2);
        track_seq++;
        // 保存Canny结果（二值图），每秒一次（需确保目录存在）
        if (debug_save_images == 1 && !binary_frame2.empty() && (frame_counter % save_every_n_frames == 0)) {
            ensure_dir(debug_dir_canny);
            std::string fname = std::string(debug_dir_canny) + "/canny_" + std::to_string(frame_counter) + ".png";
            try { cv::imwrite(fname, binary_frame2); } catch (...) {}
        }
        hough_result = get_lines(binary_frame2, left_screen_line, right_screen_line);
        int mode = hough_result;
        if (auto_mode) {
            if (target_threshold == left_threshold) {
                if (hough_result & 1) mode = 1;
            } else if (target_threshold == right_threshold) {
                if (hough_result & 2) mode = 2;
            }
        }
        map_lines(left_screen_line, right_screen_line,
                  left_real_line, right_real_line, mode);
        if (auto_mode) servo_control(get_error_angle(left_real_line, right_real_line, target_threshold));
        
        frame_counter++;
    } while (cam2.read(input_frame2) && !input_frame2.empty());
    pthread_exit((void *)ret);
}
 
void *check_crosswalk(void*){
    int cnt = 0, *ret = new int; *ret = -1;
    std::vector<std::vector<cv::Point> > contours;
    static int64 last_save_tick = 0;
    static int64 last_print_tick = 0;
    int DETECT_PERIOD_US = crosswalk_detect_period_us;
    int last_seq = -1;
    while (!binary_frame2.empty()){
        int64 t0 = cv::getTickCount();
        if (track_seq == last_seq) { usleep(DETECT_PERIOD_US); continue; }
        last_seq = track_seq;
        cv::Mat bw = binary_frame2.clone();
        int rows = bw.rows;
        int cols = bw.cols;
        int cut = int(cols * 0.15);
        if (cut < 0) cut = 0;
        if (2 * cut >= cols) cut = cols / 2 - 1;
        int bottom_cut = int(rows * 0.06);
        if (bottom_cut < 0) bottom_cut = 0;
        if (bottom_cut >= rows) bottom_cut = rows - 1;
        int top_cut = int(rows * 0.30);
        if (top_cut < 0) top_cut = 0;
        if (top_cut >= rows) top_cut = rows - 1;
        int roi_h = rows - bottom_cut - top_cut;
        if (roi_h <= 0) roi_h = 1;
        cv::Mat roi_bw = cv::Mat(bw, cv::Rect(cut, top_cut, cols - 2 * cut, roi_h));
        findContours(roi_bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cnt = 0;
        for (std::vector<cv::Point> vec : contours){
            int pts = (int)vec.size();
            if (pts > crosswalk_min_points && pts < crosswalk_max_points) {
                double area = cv::contourArea(vec);
                cv::Rect r = cv::boundingRect(vec);
                double rect_area = double(r.width) * double(r.height);
                double fill_ratio = rect_area > 0.0 ? area / rect_area : 0.0;
                if (fill_ratio >= crosswalk_min_fill_ratio) cnt++;
            }
        }
        std::cout << "斑点数量: " << cnt << std::endl;
        if (debug_save_images) {
            ensure_dir(debug_dir_crosswalk);
            cv::Mat vis;
            cv::cvtColor(roi_bw, vis, cv::COLOR_GRAY2BGR);
            for (size_t i = 0; i < contours.size(); ++i) {
                int pts = (int)contours[i].size();
                double area = cv::contourArea(contours[i]);
                cv::Rect r = cv::boundingRect(contours[i]);
                double rect_area = double(r.width) * double(r.height);
                double fill_ratio = rect_area > 0.0 ? area / rect_area : 0.0;
                bool valid_pts = (pts > crosswalk_min_points && pts < crosswalk_max_points);
                bool valid_fill = (fill_ratio >= crosswalk_min_fill_ratio);
                if (valid_pts && valid_fill) {
                    cv::drawContours(vis, contours, (int)i, cv::Scalar(0, 255, 0), -1);
                } else {
                    cv::drawContours(vis, contours, (int)i, cv::Scalar(0, 255, 255), 1);
                }
                cv::rectangle(vis, r, cv::Scalar(255, 0, 255), 1);
                cv::putText(vis, std::to_string((int)std::round(fill_ratio * 100.0)) + "%", cv::Point(r.x, std::max(0, r.y - 5)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
            }
            cv::putText(vis, std::string("count: ") + std::to_string(cnt), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
            std::string fname = std::string(debug_dir_crosswalk) + "/contours_" + std::to_string(frame_counter) + ".png";
            try { cv::imwrite(fname, vis); } catch (...) {}
            frame_counter++;
        }
        if (cnt >= cross_cnt) {
            *ret = 0;
	    break;
        }
        

    }
    pthread_exit((void *)ret);
}

void *check_stoparea(void*){
    int *ret = new int; *ret = -1;
    double area_threshold = stoparea_area_threshold;
    int consecutive = 0;
    for (int warm = 0; warm < 2; ++warm) cam0_read_locked(input_frame0);
    while (!input_frame0.empty()){
        if (!cam0_read_locked(input_frame0) || input_frame0.empty()) { continue; }
        cv::Mat img = input_frame0;
        int rows = img.rows, cols = img.cols;
        int y0 = int(rows * 0.33);
        int roi_h = rows - y0;
        if (roi_h <= 0 || y0 < 0 || y0 + roi_h > rows) { continue; }
        cv::Mat roi = cv::Mat(img, cv::Rect(0, y0, cols, roi_h));
        if (debug_save_images) std::cout << "Stop ROI: " << cols << "x" << roi_h << ", area_th=" << area_threshold << std::endl;

        cv::Mat hsv, mask;
        cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(100, 50, 150), cv::Scalar(130, 255, 255), mask);
        cv::Mat k1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k1);
        cv::Mat k2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k2);

        std::vector<std::vector<cv::Point>> cnts;
        cv::findContours(mask, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (debug_save_images) std::cout << "Contours: " << cnts.size() << std::endl;
        double bestRectArea = -1.0; cv::Rect bestRect; double bestRatio = 0.0;
        for (const auto& c : cnts){
            cv::Rect r = cv::boundingRect(c);
            if (r.width <= 0 || r.height <= 0) continue;
            double ratio = double(r.width) / double(r.height);
            if (ratio < 1.5 || ratio > 8.0) continue;
            double rectArea = double(r.width) * double(r.height);
            if (rectArea > bestRectArea){ bestRectArea = rectArea; bestRect = r; bestRatio = ratio; }
        }
        int rect_pixels = 0;
        if (bestRectArea > 0) {
            cv::Mat rectMask = mask(bestRect);
            rect_pixels = cv::countNonZero(rectMask);
        }
        if (debug_save_images) std::cout << "BestRect area=" << (bestRectArea<0?0:bestRectArea) << ", ratio=" << bestRatio << " rect=" << bestRect.width << "x" << bestRect.height << ", rect_pixels=" << rect_pixels << std::endl;

        if (debug_save_images){
            ensure_dir(debug_dir_stoparea);
            cv::Mat vis_roi = roi.clone();
            if (bestRectArea > 0) {
                cv::rectangle(vis_roi, bestRect, cv::Scalar(0,255,0), 2);
                std::string info = std::string("pix:") + std::to_string(rect_pixels) + std::string(" ratio:") + std::to_string(bestRatio);
                cv::putText(vis_roi, info, cv::Point(std::max(0,bestRect.x-5), std::max(0,bestRect.y-8)), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
            }
            std::string fvis = std::string(debug_dir_stoparea) + "/cam0_stop_vis_" + std::to_string(frame_counter) + ".png";
            try { cv::imwrite(fvis, vis_roi); } catch (...) {}
            frame_counter++;
        }

        if (debug_save_images) std::cout << "StopArea decision: " << (bestRectArea > area_threshold ? "BRAKE" : "WAIT") << std::endl;
        if (bestRectArea > area_threshold) { consecutive++; } else { consecutive = 0; }
        if (consecutive >= 2) { *ret = 0; break; }
        
    }
    pthread_exit((void *)ret);
}

void *check_stoparea2(void*){
    int *ret = new int; *ret = -1;
    int consecutive = 0;
    for (int warm = 0; warm < 2; ++warm) { if (!binary_frame2.empty()) break; usleep(30000); }
    while (!binary_frame2.empty()){
        cv::Mat bw = binary_frame2.clone();
        int rows = bw.rows, cols = bw.cols;
        int cut = int(cols * 0.15);
        if (cut < 0) cut = 0;
        if (2 * cut >= cols) cut = cols / 2 - 1;
        int bottom_cut = int(rows * 0.06);
        if (bottom_cut < 0) bottom_cut = 0;
        if (bottom_cut >= rows) bottom_cut = rows - 1;
        int top_cut = int(rows * 0.20);
        if (top_cut < 0) top_cut = 0;
        if (top_cut >= rows) top_cut = rows - 1;
        int roi_h = rows - bottom_cut - top_cut;
        if (roi_h <= 0) roi_h = 1;
        cv::Mat roi_bw = cv::Mat(bw, cv::Rect(cut, top_cut, cols - 2 * cut, roi_h));

        std::vector<std::vector<cv::Point>> cnts;
        cv::findContours(roi_bw, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double bestRectArea = -1.0; cv::Rect bestRect; double best_fill_ratio = 0.0; int best_pixels = 0; double best_ratio_wh = 0.0;
        for (const auto& c : cnts){
            double area = cv::contourArea(c);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(c, approx, 0.025 * cv::arcLength(c, true), true);
            if (approx.size() == 4 && cv::isContourConvex(approx)){
                cv::Rect r = cv::boundingRect(approx);
                if (r.width <= 0 || r.height <= 0) continue;
                double rectArea = double(r.width) * double(r.height);
                if (rectArea <= 0.0) continue;
                double fill_ratio = area / rectArea;
                int pixels = cv::countNonZero(roi_bw(r));
                double ratio_wh = double(r.width) / double(r.height);
                if (fill_ratio >= 0.40 && ratio_wh >= 1.5 && ratio_wh <= 11.0){
                    if (rectArea > bestRectArea){ bestRectArea = rectArea; bestRect = r; best_fill_ratio = fill_ratio; best_pixels = pixels; best_ratio_wh = ratio_wh; }
                }
            }
        }

        bool detected = (bestRectArea > 0 && best_pixels > 320);
        if (detected) consecutive++; else consecutive = 0;
        if (consecutive >= 2){ *ret = 0; break; }

        if (debug_save_images){
            ensure_dir(debug_dir_stoparea);
            cv::Mat vis; cv::cvtColor(roi_bw, vis, cv::COLOR_GRAY2BGR);
            if (bestRectArea > 0){
                cv::rectangle(vis, bestRect, cv::Scalar(0,255,0), 2);
                std::string info = std::string("pix:") + std::to_string(best_pixels) + std::string(" ratio:") + std::to_string(best_ratio_wh) + std::string(" fill:") + std::to_string(best_fill_ratio);
                cv::putText(vis, info, cv::Point(std::max(0,bestRect.x-5), std::max(0,bestRect.y-8)), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2);
            }
            std::string fvis = std::string(debug_dir_stoparea) + "/cam2_stop2_vis_" + std::to_string(frame_counter) + ".png";
            try { cv::imwrite(fvis, vis); } catch (...) {}
            frame_counter++;
        }

        
    }
    pthread_exit((void *)ret);
}

void change_track(){
    if (check_result) turn_angle *= -1;
    kick_start(11500);
    usleep(100000);
    gpioPWM(12,847 + turn_angle);
    usleep(change_track_turn_sleep);
    gpioPWM(12,847);
    usleep(change_track_straight_sleep);
}

void change_track_return(){
    // 执行与初始变道方向相反的动作
    kick_start(11400);
    usleep(100000);
    gpioPWM(12,847 - turn_angle);
    usleep(change_track_turn_sleep - 250000);
    gpioPWM(12,847);
    usleep(change_track_straight_sleep - 210000);
}

int check_return_lane_cones(){
    int check_count = 0;
    if (cam0_read_locked(input_frame0) && !input_frame0.empty()){
        int h = input_frame0.rows;
        int w = input_frame0.cols;
        int y = int(h * 0.34);
        int hh = int(h * 0.4);
        cv::Mat roi = cv::Mat(input_frame0, cv::Rect(0, y, w, hh));
        cones_image_process(roi, check_count);
        std::cout << "返回变道检测: 发现 " << check_count << " 个锥桶." << std::endl;
    }
    cam0_read_locked(input_frame0);
    return check_count;
}

void wait_and_return_lane(){
    usleep(800000); // 等待0.8秒稳定

    int consecutive_detections = 0;
    while (true) {
        if (check_return_lane_cones() <= 1) {
            consecutive_detections++;
            std::cout << "返回变道连续检测: " << consecutive_detections << " / 3" << std::endl;
        } else {
            if (consecutive_detections > 0) {
                std::cout << "返回变道连续检测中断, 计数器重置." << std::endl;
            }
            consecutive_detections = 0;
        }

        if (consecutive_detections >= 3) {
            std::cout << "!!! 触发返回变道 !!!" << std::endl;
            auto_mode = false;
            gpioPWM(12,847);
            usleep(100000);
            change_track_return();
            auto_mode = true;
            break;
        }

        usleep(100000);
    }
}

void *check_cones(void *){
    int *ret = new int, check_count = 0, step = 0; *ret = -1;
    double targets[4] = {left_threshold, right_threshold, left_threshold, 0.34};
    target_threshold = targets[0];
    for (int i = 0; i < 4; i++) cam0_read_locked(input_frame0);
    while (!input_frame0.empty() && cam0_read_locked(input_frame0)){
        cones_image_process(cv::Mat(input_frame0, cv::Rect(0, 260, 640, 220)), check_count);
	std::cout << step << ' ' << check_count << std::endl;
        if (check_count >= 1) {
	    usleep(500000);
            for (int i = 0; i < 4; i++) cam0_read_locked(input_frame0);
	    std::cout << step << std::endl;
            target_threshold = targets[++step];
            check_count = 0;
            if (step == 3) {
		usleep(100000);
		*ret = 0;
                break;
            }
        }
        cam0_read_locked(input_frame0);
    }
    pthread_exit((void *)ret);
    return ret;
}


void *check_direction(void *){
    int *ret = new int; *ret = -1;
    check_result = false;
    int left_count = 0;
    int right_count = 0;
    int total_runs = 10;
    for (int warm = 0; warm < 2; ++warm) cam0_read_locked(input_frame0);
    for (int i = 1; i <= total_runs; ++i) {
        if (!cam0_read_locked(input_frame0) || input_frame0.empty()) {
            --i;
            continue;
        }
        char dir = check_direction_arrow(input_frame0);
        std::string msg;
        if (dir == 1) { left_count++; msg = "左转"; }
        else if (dir == 2) { right_count++; msg = "右转"; }
        else { msg = "未识别"; }
        std::cout << "识别结果: " << msg << ", 左转 " << left_count << " 次, 右转 " << right_count << " 次, 进度 " << i << " / " << total_runs << std::endl;
        usleep(33333);
    }
    check_result = (right_count > left_count);
    *ret = 0;
    pthread_exit((void *)ret);
}


char check_direction_arrow(const cv::Mat& input){
    if (input.empty()) return 0;
    cv::Mat frame = input.clone();
    int h = frame.rows;
    int w = frame.cols;
    int y = int(h * 0.45);
    int hh = int(h * 0.35);
    if (y + hh > h) hh = h - y;
    cv::Mat roi = cv::Mat(frame, cv::Rect(0, y, w, hh));
    cv::Mat hsv; cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::Mat blueMask; cv::inRange(hsv, cv::Scalar(100, 55, 155), cv::Scalar(125, 255, 255), blueMask);
    if (cv::countNonZero(blueMask) == 0) return 0;
    std::vector<std::vector<cv::Point>> contours; cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return 0;
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b){ return cv::contourArea(a) > cv::contourArea(b); });
    double area_threshold = 1250.0;
    std::vector<cv::Point> merged;
    if (contours.size() == 1) {
        merged = contours[0];
    } else {
        if (cv::contourArea(contours[0]) >= area_threshold) {
            merged = contours[0];
        } else {
            merged = contours[0];
            merged.insert(merged.end(), contours[1].begin(), contours[1].end());
        }
    }
    cv::Rect bbox = cv::boundingRect(merged);
    bbox.x = std::max(0, bbox.x);
    bbox.y = std::max(0, bbox.y);
    if (bbox.x + bbox.width > roi.cols) bbox.width = roi.cols - bbox.x;
    if (bbox.y + bbox.height > roi.rows) bbox.height = roi.rows - bbox.y;
    if (bbox.width <= 0 || bbox.height <= 0) return 0;
    cv::Mat cropped = cv::Mat(roi, bbox);
    std::vector<cv::Point> hull; cv::convexHull(merged, hull);
    double eps = 0.02 * cv::arcLength(hull, true);
    std::vector<cv::Point> quad; cv::approxPolyDP(hull, quad, eps, true);
    cv::Point tl, tr, bl, br; int dstW = 0, dstH = 0; cv::Mat M;
    if (quad.size() == 4) {
        std::vector<cv::Point> pts = quad;
        std::sort(pts.begin(), pts.end(), [](const cv::Point& a, const cv::Point& b){ return a.y < b.y; });
        std::vector<cv::Point> top = {pts[0], pts[1]};
        std::vector<cv::Point> bottom = {pts[2], pts[3]};
        if (top[0].x < top[1].x) { tl = top[0]; tr = top[1]; } else { tl = top[1]; tr = top[0]; }
        if (bottom[0].x < bottom[1].x) { bl = bottom[0]; br = bottom[1]; } else { bl = bottom[1]; br = bottom[0]; }
        dstW = int(std::max(cv::norm(tr - tl), cv::norm(br - bl)));
        dstH = int(std::max(cv::norm(bl - tl), cv::norm(br - tr)));
        if (dstW > 0 && dstH > 0) {
            std::vector<cv::Point2f> srcPts = {cv::Point2f(tl.x, tl.y), cv::Point2f(tr.x, tr.y), cv::Point2f(br.x, br.y), cv::Point2f(bl.x, bl.y)};
            std::vector<cv::Point2f> dstPts = {cv::Point2f(0, 0), cv::Point2f(dstW - 1, 0), cv::Point2f(dstW - 1, dstH - 1), cv::Point2f(0, dstH - 1)};
            M = cv::getPerspectiveTransform(srcPts, dstPts);
        }
    }
    if (!M.empty() && dstW > 0 && dstH > 0) {
        cv::warpPerspective(roi, cropped, M, cv::Size(dstW, dstH));
    }
    int rw = cropped.cols;
    int rh = cropped.rows;
    if (rw <= 0 || rh <= 0) return 0;
    int cutX = int(rw * 0.15);
    int cutTop = int(rh * 0.10);
    int cutBottom = int(rh * 0.35);
    int cw = rw - 2 * cutX;
    int ch = rh - cutTop - cutBottom;
    if (cutX < 0) cutX = 0;
    if (cutTop < 0) cutTop = 0;
    if (cutBottom < 0) cutBottom = 0;
    if (cw <= 0 || ch <= 0) return 0;
    cv::Mat rectROI = cv::Mat(cropped, cv::Rect(cutX, cutTop, cw, ch));
    cv::Mat gray, bin;
    cv::cvtColor(rectROI, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat kbin = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(bin, bin, kbin, cv::Point(-1,-1), 2);
    int W = bin.cols;
    int H = bin.rows;
    if (W <= 0 || H <= 0) return 0;
    int mid = W / 2;
    int left_white = cv::countNonZero(bin(cv::Rect(0, 0, mid, H)));
    int right_white = cv::countNonZero(bin(cv::Rect(mid, 0, W - mid, H)));
    if (debug_save_images) {
        ensure_dir(debug_dir_direction);
        static int arrow_counter = 0;
        int idx = ++arrow_counter;
        cv::Mat roi_vis = roi.clone();
        cv::rectangle(roi_vis, bbox, cv::Scalar(0, 255, 0), 2);
        std::string f1 = std::string(debug_dir_direction) + "/dir_blue_bbox_" + std::to_string(idx) + ".png";
        try { cv::imwrite(f1, roi_vis); } catch (...) {}
        if (!M.empty() && cropped.cols > 0 && cropped.rows > 0) {
            std::string f0 = std::string(debug_dir_direction) + "/dir_rectified_" + std::to_string(idx) + ".png";
            try { cv::imwrite(f0, cropped); } catch (...) {}
        }
        cv::Mat dbg = bin.clone();
        std::string f2 = std::string(debug_dir_direction) + "/dir_bin_crop_" + std::to_string(left_white) + "_" + std::to_string(right_white) + "_" + std::to_string(idx) + ".png";
        try { cv::imwrite(f2, dbg); } catch (...) {}
    }
    if (left_white > right_white) return 1;
    if (right_white > left_white) return 2;
    return 0;
}

void *check_direction2(void *){
    int *ret = new int; *ret = -1;
    int left_count = 0;
    int right_count = 0;
    int total_runs = 10;
    static int dir_counter = 0;

    for (int warm = 0; warm < 2; ++warm) cam0_read_locked(input_frame0);

    for (int i = 1; i <= total_runs; ++i) {
        if (!cam0_read_locked(input_frame0) || input_frame0.empty()) {
            usleep(50000);
            --i; // 重试当前轮次
            continue;
        }
        char dir = check_direction_arrow(input_frame0);
        std::string msg;
        if (dir == 1) { left_count++; msg = "左转"; }
        else if (dir == 2) { right_count++; msg = "右转"; }
        else { msg = "未识别"; }
        std::cout << "识别结果: " << msg << ", 左转 " << left_count << " 次, 右转 " << right_count << " 次, 进度 " << i << " / " << total_runs << std::endl;
        
        usleep(50000);
    }

    std::string final_dir;
    if (left_count > right_count) final_dir = "左转";
    else if (right_count > left_count) final_dir = "右转";
    else final_dir = "未识别";

    std::cout << "最终方向: " << final_dir << ", 左转总计 " << left_count << " 次, 右转总计 " << right_count << " 次" << std::endl;
    if (right_count > left_count) {
        check_result = true;  // 右转
    } else if (left_count > right_count) {
        check_result = false; // 左转
    }
    *ret = 0;
    pthread_exit((void *)ret);
}

void *check_yellowline(void *){
    int *ret = new int; *ret = -1;
    for (int i = 0; i < 4; i++) cam0_read_locked(input_frame0);
    while (!input_frame0.empty()) {
        if (detect_yellow_line(input_frame0)){
            *ret = 0;
            break;
        }
        cam0_read_locked(input_frame0);
    }
    pthread_exit((void *)ret);
}



bool get_symbol() {
    int b_count = 0;
    int a_count = 0;
    int total_runs = 15;
    for (int warm = 0; warm < 2; ++warm) cam0_read_locked(input_frame0);
    for (int i = 1; i <= total_runs; ++i) {
        if (!cam0_read_locked(input_frame0) || input_frame0.empty()) {
            usleep(50000);
            --i;
            continue;
        }
        int result = check_symbol1(input_frame0);
        if (result == 1) {
            b_count++;
            std::cout << "识别结果: B, 进度 " << i << " / " << total_runs << std::endl;
        } else if (result == 0) {
            a_count++;
            std::cout << "识别结果: A, 进度 " << i << " / " << total_runs << std::endl;
        } else {
            std::cout << "？？？无法识别？？？" << std::endl;
        }
        usleep(50000);
    }
    std::cout << "A次数: " << a_count << ", B次数: " << b_count << std::endl;
    if (b_count >= a_count) {
        std::cout << "识别成功: B" << std::endl;
        if (debug_save_images == 1) {
            cv::Mat vis = input_frame0.clone();
            std::string msg = std::string("A:") + std::to_string(a_count) + " B:" + std::to_string(b_count) + " => B";
            cv::putText(vis, msg, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
            try { cv::imwrite(std::string("debug_symbol") + "/final.png", vis); } catch (...) {}
        }
        return true;
    } else {
        std::cout << "识别成功: A" << std::endl;
        if (debug_save_images == 1) {
            cv::Mat vis = input_frame0.clone();
            std::string msg = std::string("A:") + std::to_string(a_count) + " B:" + std::to_string(b_count) + " => A";
            cv::putText(vis, msg, cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
            try { cv::imwrite(std::string("debug_symbol") + "/final.png", vis); } catch (...) {}
        }
        return false;
    }
}

int check_blue_plain(){
    cam0_read_locked(input_frame0);//
    while (!input_frame0.empty()) {
        if (!blue_card_find(input_frame0)) {
            return 0;
        }
        cam0_read_locked(input_frame0);
    }
    return -1;
}

void *check_blue_cone(void *){
    int *ret = new int, check_count = 0, consecutive_detections = 0, avoidance_count = 0; *ret = -1;
    double original_target_threshold = target_threshold;
    double cone_avoid_target_threshold;
    bool avoiding = false;
    time_t avoidance_start_time; // 用于避障计时

    cone_avoid_target_threshold = 0.585;

    time_t start_time = time(nullptr);
    while (difftime(time(nullptr), start_time) < cone_check_time) {
        // 优先处理避障状态的结束
        if (avoiding && difftime(time(nullptr), avoidance_start_time) > 1) { // 避障状态持续1.5秒
            std::cout << "避障时间到, 恢复正常巡线和速度." << std::endl;
            target_threshold = original_target_threshold;
            set_speed(normal_speed);
            avoiding = false;
            consecutive_detections = 0; // 避障完成后重置检测计数
        }

        if (cam0_read_locked(input_frame0) && !input_frame0.empty()){
            // 仅在非避障状态下检测新锥桶
            if (!avoiding) {
                int rows = input_frame0.rows;
                int cols = input_frame0.cols;
                int y = 170;
                int h = 220;
                if (y < 0) y = 0;
                if (h < 0) h = 0;
                if (y + h > rows) h = rows - y;
                int cut = int(cols * 0.15);
                if (cut < 0) cut = 0;
                if (2 * cut >= cols) cut = cols / 2 - 1;
                cv::Mat roi = cv::Mat(input_frame0, cv::Rect(cut, y, cols - 2 * cut, h));
                cones_image_process(roi, check_count);
                if (check_count >= 1) { // 识别到锥桶
                    consecutive_detections++;
                    std::cout << "连续识别到锥桶: " << consecutive_detections << " 次" << std::endl;
                } else {
                    if (consecutive_detections > 0) {
                        std::cout << "锥桶丢失, 连续识别计数重置." << std::endl;
                    }
                    consecutive_detections = 0;
                }

                // 触发条件：非避障状态 && 连续识别次数达标
                if (consecutive_detections >= cone_detection_threshold) {
                    std::cout << "!!! 触发避障 #" << avoidance_count + 1 << " !!!" << std::endl;
                    target_threshold = cone_avoid_target_threshold;
                    kick_stop(11000); // 减速
                    avoiding = true;
                    avoidance_start_time = time(nullptr); // 启动避障计时器
                    avoidance_count++;
                }
            }
        }
        // 降低CPU占用率，约20Hz
    }

    std::cout << "锥桶检测结束. 总计触发避障 " << avoidance_count << " 次." << std::endl;
    // 线程结束时，如果仍在避障状态，强制恢复
    if (avoiding) {
        std::cout << "检测时间到, 强制恢复正常巡线和速度." << std::endl;
    }
    target_threshold = original_target_threshold; // 恢复之前的巡线目标
    set_speed(normal_speed); // 恢复正常速度
    *ret = 0;
    pthread_exit((void *)ret);
    return ret;
}

 

int check_yellow_cones(){
    int check_count = 0;
    static int s_low_cached = 60;
    static int v_low_cached = 90;
    static int64 last_update_tick = 0;
    static double tick_freq = 0.0;
    static int save_counter = 0;

    {
        pthread_mutex_lock(&cam0_mutex);
        bool ok = cam0.read(input_frame0);
        pthread_mutex_unlock(&cam0_mutex);
        if (!input_frame0.empty() && ok){
            int h = input_frame0.rows, w = input_frame0.cols;
            int y = int(h * 0.35), hh = int(h * 0.4);
            cv::Mat roi = cv::Mat(input_frame0, cv::Rect(0, y, w, hh));
            cv::Mat hsv; cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

            // Adaptive threshold update
            if (tick_freq == 0.0) tick_freq = cv::getTickFrequency();
            int64 now = cv::getTickCount();
            if (last_update_tick == 0 || (double)(now - last_update_tick) / tick_freq > 0.2) {
                cv::Scalar m = cv::mean(hsv);
                // Adaptive S/V targets
                int s_target = std::max(40, (int)(m[1] * 0.7));
                int v_target = std::max(40, (int)(m[2] * 0.7));
                s_low_cached = int(0.6 * s_low_cached + 0.4 * s_target);
                v_low_cached = int(0.6 * v_low_cached + 0.4 * v_target);
                last_update_tick = now;
            }

            int s_low = std::max(28, s_low_cached);
            int v_low = std::max(180, v_low_cached);
            cv::Scalar lower(15, s_low, v_low);
            cv::Scalar upper(40, 255, 255);
            cv::Mat m; cv::inRange(hsv, lower, upper, m);
            cv::morphologyEx(m, m, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
            cv::dilate(m, m, cv::Mat(), cv::Point(-1,-1), 1);

            std::vector<std::vector<cv::Point>> cnts; 
            cv::findContours(m, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            cv::Mat vis;
            if (debug_save_images) vis = roi.clone();

            for (const auto& c : cnts){ 
                double area = cv::contourArea(c);
                if (area < cone_yellow_min_area || area > cone_yellow_max_area) continue;
                
                cv::RotatedRect rr = cv::minAreaRect(c);
                double w_r = rr.size.width, h_r = rr.size.height;
                if (w_r <= 1.0 || h_r <= 1.0) continue;
                double ratio = (h_r > w_r) ? (h_r / w_r) : (w_r / h_r);
                
                // Relaxed aspect ratio for robustness
                if (ratio >= 1.0 && ratio <= 4.0) { 
                    check_count++; 
                    if (debug_save_images) {
                        cv::Point2f pts[4]; rr.points(pts);
                        for (int i = 0; i < 4; ++i) cv::line(vis, pts[i], pts[(i+1)%4], cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
            int blue_cone_count = 0;
            cones_image_process(roi, blue_cone_count);

            bool permit = (check_count >= 1) || (blue_cone_count >= 3);
            std::cout << "黄锥桶检测: 发现 " << check_count << " 个锥桶." << std::endl;
            if (permit) std::cout << "准许变道" << std::endl;
            
            if (permit && debug_save_images) {
                std::string fname = std::string("debug_avoid/yellow_cone_") + std::to_string(save_counter++) + std::string(".png");
                try { cv::imwrite(fname, vis); } catch (...) {}
            }
            if (permit) {
                pthread_mutex_lock(&cam0_mutex);
                cam0.read(input_frame0);
                pthread_mutex_unlock(&cam0_mutex);
                return 1;
            }
        }
    }
    pthread_mutex_lock(&cam0_mutex);
    cam0.read(input_frame0);
    pthread_mutex_unlock(&cam0_mutex);
    return 0;
}

void *save_timestamp_images(void *){
    int *ret = new int; *ret = -1;
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, nullptr);
    ensure_dir("time_stamp");
    std::vector<cv::String> old_files; cv::glob("time_stamp/*.png", old_files, false);
    for (size_t i = 0; i < old_files.size(); ++i) std::remove(old_files[i].c_str());
    while (true){
        cv::Mat frame; if (cam0_read_locked(frame) && !frame.empty()){
            time_t t = time(nullptr) + 8 * 3600; tm *tmv = gmtime(&t);
            char txt[64]; snprintf(txt, sizeof(txt), "%04d-%02d-%02d %02d:%02d:%02d", tmv->tm_year+1900, tmv->tm_mon+1, tmv->tm_mday, tmv->tm_hour, tmv->tm_min, tmv->tm_sec);
            cv::putText(frame, std::string(txt), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            char fn[128]; snprintf(fn, sizeof(fn), "time_stamp/%04d%02d%02d_%02d%02d%02d.png", tmv->tm_year+1900, tmv->tm_mon+1, tmv->tm_mday, tmv->tm_hour, tmv->tm_min, tmv->tm_sec);
            try { cv::imwrite(std::string(fn), frame); } catch (...) {}
        }
        usleep(30000000);
    }
    pthread_exit((void *)ret);
}
