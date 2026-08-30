#include "check_symbol.hpp"
#include "utils.hpp"
#include <string>

using namespace cv;
using namespace std;
static int symbol_save_idx = 0;

int check_symbol1(cv::Mat image) {
    int height = image.rows;
    int width = image.cols;

    Mat cropped_image = image(Rect(0, int(height * 0.45), width, height - int(height * 0.45)));

    // Vec3b color_rgb(203, 115, 153);
    // Mat color_hsv;
    // cvtColor(Mat(1, 1, CV_8UC3, color_rgb), color_hsv, COLOR_BGR2HSV);
    // Vec3b target_hsv = color_hsv.at<Vec3b>(0, 0);
    // cout << "HSV_now: (" << (int)target_hsv[0] << ", " << (int)target_hsv[1] << ", " << (int)target_hsv[2] << ")" << endl;

    Scalar lower_bound(95, 20, 110);
    Scalar upper_bound(145, 255, 255);

    Mat hsv_image;
    Mat blur;
    GaussianBlur(cropped_image, blur, Size(3,3), 0.6);
    cvtColor(blur, hsv_image, COLOR_BGR2HSV);

    Mat mask;
    inRange(hsv_image, lower_bound, upper_bound, mask);

    // 查找连通区域（轮廓）
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        if (debug_save_images == 1) {
            try { cv::imwrite(std::string("debug_symbol") + "/mask_" + std::to_string(symbol_save_idx) + ".png", mask); } catch (...) {}
            Mat vis = cropped_image.clone();
            putText(vis, std::string("inside:") + std::to_string(0) + " Unknown", Point(10,25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);
            try { cv::imwrite(std::string("debug_symbol") + "/vis_" + std::to_string(symbol_save_idx) + ".png", vis); } catch (...) {}
            symbol_save_idx++;
        }
        return -1;
    }

    // 初始化最大连通区域的面积和外轮廓的面积
    double max_connected_area = 0;
    int max_contour_index = -1;
    vector<Point> max_c;

    // 遍历所有轮廓，找到最大连通区域
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > max_connected_area && area > 100.0) {
            max_connected_area = area;
            max_contour_index = i;
            max_c = contours[i];
        }
    }

    if (max_contour_index < 0 || (int)max_c.size() < 3) {
        if (debug_save_images == 1) {
            try { cv::imwrite(std::string("debug_symbol") + "/mask_" + std::to_string(symbol_save_idx) + ".png", mask); } catch (...) {}
            Mat vis = cropped_image.clone();
            putText(vis, std::string("inside:") + std::to_string(0) + " Unknown", Point(10,25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);
            try { cv::imwrite(std::string("debug_symbol") + "/vis_" + std::to_string(symbol_save_idx) + ".png", vis); } catch (...) {}
            symbol_save_idx++;
        }
        return -1;
    }

    int count_inside = 0;
    

    // 在蓝色框选区域内使用 Canny 边缘识别闭合曲线数量
    Rect r_full = boundingRect(max_c);
    r_full.x = std::max(0, r_full.x);
    r_full.y = std::max(0, r_full.y);
    if (r_full.x + r_full.width > cropped_image.cols) r_full.width = cropped_image.cols - r_full.x;
    if (r_full.y + r_full.height > cropped_image.rows) r_full.height = cropped_image.rows - r_full.y;
    if (r_full.width > 0 && r_full.height > 0) {
        int cx = r_full.x + int(r_full.width * 0.20);
        int cy = r_full.y + int(r_full.height * 0.15);
        int cw = int(r_full.width * 0.60);
        int ch = int(r_full.height * 0.70);
        if (cx < 0) cx = 0; if (cy < 0) cy = 0;
        if (cx + cw > cropped_image.cols) cw = cropped_image.cols - cx;
        if (cy + ch > cropped_image.rows) ch = cropped_image.rows - cy;
        if (cw <= 0 || ch <= 0) { count_inside = 0; }
        Mat roiImg = Mat(cropped_image, Rect(cx, cy, cw, ch));
        Mat gray, edges;
        cvtColor(roiImg, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5,5), 0.8);
        Canny(gray, edges, 50, 140);
        Mat kEdge = getStructuringElement(MORPH_RECT, Size(3,3));
        dilate(edges, edges, kEdge, Point(-1,-1), 2);
        std::vector<std::vector<Point>> cnts; std::vector<Vec4i> hier;
        findContours(edges, cnts, hier, RETR_TREE, CHAIN_APPROX_SIMPLE);
        int inner_count = 0;
        for (size_t i = 0; i < cnts.size(); ++i) {
            if (hier.empty()) break;
            // 仅统计“有父且无子”的叶子内部轮廓，避免把外层B轮廓计入
            if (hier[i][3] == -1 || hier[i][2] != -1) continue;
            double area = contourArea(cnts[i]);
            if (area < 70.0) continue;
            Rect rb = boundingRect(cnts[i]);
            if (rb.x <= 0 || rb.y <= 0 || rb.x + rb.width >= edges.cols-1 || rb.y + rb.height >= edges.rows-1) continue; // 贴边轮廓忽略
            double ar = double(rb.width) / double(rb.height);
            if (ar < 0.2 || ar > 7) continue; // 过滤细长闭合区域
            double fill = area / double(std::max(1, rb.width * rb.height));
            if (fill < 0.30 || fill > 0.95) continue; // 收紧填充比，增强过滤强度
            inner_count++;
        }
        count_inside = inner_count;
        if (debug_save_images == 1) {
            try { imwrite(std::string("debug_symbol") + "/edges_roi_" + std::to_string(symbol_save_idx) + ".png", edges); } catch (...) {}
        }
    }
    cout << count_inside << endl;
    if (debug_save_images == 1) {
        try { cv::imwrite(std::string("debug_symbol") + "/mask_" + std::to_string(symbol_save_idx) + ".png", mask); } catch (...) {}
        Mat vis = cropped_image.clone();
        if (max_contour_index >= 0 && max_c.size() >= 3) {
            drawContours(vis, vector<vector<Point>>{max_c}, 0, Scalar(255,0,255), 2);
            Rect r = boundingRect(max_c);
            rectangle(vis, r, Scalar(0,255,0), 2);
            if (r.width > 0 && r.height > 0) {
                int cx = r.x + int(r.width * 0.20);
                int cy = r.y + int(r.height * 0.15);
                int cw = int(r.width * 0.60);
                int ch = int(r.height * 0.70);
                if (cx < 0) cx = 0; if (cy < 0) cy = 0;
                if (cx + cw > cropped_image.cols) cw = cropped_image.cols - cx;
                if (cy + ch > cropped_image.rows) ch = cropped_image.rows - cy;
                if (cw <= 0 || ch <= 0) { /* skip invalid crop */ } else {
                Rect ir(cx, cy, cw, ch);
                Mat roiImg = Mat(cropped_image, ir);
                Mat grayR, edgesR; cvtColor(roiImg, grayR, COLOR_BGR2GRAY);
                GaussianBlur(grayR, grayR, Size(5,5), 0.8);
                Canny(grayR, edgesR, 50, 140);
                Mat kEdgeR = getStructuringElement(MORPH_RECT, Size(5,5));
                dilate(edgesR, edgesR, kEdgeR, Point(-1,-1), 2);
                std::vector<std::vector<Point>> cnts_dbg; std::vector<Vec4i> hier_dbg;
                findContours(edgesR, cnts_dbg, hier_dbg, RETR_TREE, CHAIN_APPROX_SIMPLE);
                for (size_t i = 0; i < cnts_dbg.size(); ++i) {
                    if (hier_dbg.empty()) break;
                    // 可视化仅标注“有父且无子”的叶子内部轮廓
                    if (hier_dbg[i][3] == -1 || hier_dbg[i][2] != -1) continue;
                    double area = contourArea(cnts_dbg[i]);
                    if (area < 70.0) continue;
                    Rect rb = boundingRect(cnts_dbg[i]);
                    if (rb.x <= 0 || rb.y <= 0 || rb.x + rb.width >= edgesR.cols-1 || rb.y + rb.height >= edgesR.rows-1) continue;
                    double ar = double(rb.width) / double(rb.height);
                    if (ar < 0.6 || ar > 1.8) continue;
                    double fill = area / double(std::max(1, rb.width * rb.height));
                    if (fill < 0.25 || fill > 0.80) continue;
                    std::vector<Point> drawC; drawC.reserve(cnts_dbg[i].size());
                    for (const auto &p : cnts_dbg[i]) drawC.emplace_back(p.x + ir.x, p.y + ir.y);
                    drawContours(vis, std::vector<std::vector<Point>>{drawC}, -1, Scalar(0,0,255), 2);
                }
                }
            }
        }
        std::string tag;
        if (count_inside == 1) tag = "A"; else tag = "B";
        putText(vis, std::string("inside:") + std::to_string(count_inside) + " " + tag, Point(10,25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,255,0), 2);
        try { cv::imwrite(std::string("debug_symbol") + "/vis_" + std::to_string(symbol_save_idx) + ".png", vis); } catch (...) {}
        symbol_save_idx++;
    }
    if (count_inside <= 1) {
        cout << "识别为A" << endl;
        return 0;
    } else {
        cout << "识别为B" << endl;
        return 1;
    }
}

int check_symbol2(const Mat& inputImage) {
    Mat lowerQuarter = inputImage;
    Mat hsvImage;
    cv::Mat redMask, contoursImage;
    cvtColor(lowerQuarter, hsvImage, COLOR_BGR2HSV);

    // 设置红色的HSV范围
    Scalar lowerRed1(0, 50, 50);
    Scalar upperRed1(10, 255, 255);
    Scalar lowerRed2(160, 50, 50);
    Scalar upperRed2(180, 255, 255);

    // 创建掩膜
    Mat mask1, mask2;
    inRange(hsvImage, lowerRed1, upperRed1, mask1);
    inRange(hsvImage, lowerRed2, upperRed2, mask2);
    redMask = mask1 | mask2;

    // 进行轮廓检测
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(redMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 创建轮廓图像并绘制轮廓
    // contoursImage = lowerQuarter.clone();
    // drawContours(contoursImage, contours, -1, Scalar(0, 255, 0), 1);

    double leftArea = 0;
    double rightArea = 0;
    double leftWidth = 0; // 左边轮廓的宽度
    double rightWidth = 0; // 右边轮廓的宽度

   for (const auto& contour : contours) {
        if (contour.size() < 260) continue; // 忽略小轮廓

        // 获取轮廓的最左、最右、上和下边界点
        Point leftEdge = contour[0];
        Point rightEdge = contour[0];
        Point topEdge = contour[0];
        Point bottomEdge = contour[0];

        for (const auto& point : contour) {
            if (point.x < leftEdge.x) {
                leftEdge = point;
            }
            if (point.x > rightEdge.x) {
                rightEdge = point;
            }
            if (point.y < topEdge.y) {
                topEdge = point;
            }
            if (point.y > bottomEdge.y) {
                bottomEdge = point;
            }
        }

        // 获取轮廓的宽度
        double width = rightEdge.x - leftEdge.x;

        // 计算左边界和右边界的中点
        double midpointX = (leftEdge.x + rightEdge.x) / 2.0;

        // 输出轮廓的上下左右边界点和宽度
        // std::cout << "轮廓边界点:\n";
        // std::cout << "左边界: (" << leftEdge.x << ", " << leftEdge.y << ")\n";
        // std::cout << "右边界: (" << rightEdge.x << ", " << rightEdge.y << ")\n";
        // std::cout << "上边界: (" << topEdge.x << ", " << topEdge.y << ")\n";
        // std::cout << "下边界: (" << bottomEdge.x << ", " << bottomEdge.y << ")\n";
        // std::cout << "轮廓宽度: " << width << "\n";

        // 初始化最大交点数量及对应的x值
        int maxIntersections = 0;
        double bestX = midpointX;

        // 计算中点左右10个像素范围内的直线
        for (int offset = -30; offset <= 30; ++offset) {
            double currentX = midpointX + offset;
            std::vector<int> intersectionXMidpointLine;

            for (size_t i = 0; i < contour.size(); ++i) {
                Point p1 = contour[i];
                Point p2 = contour[(i + 1) % contour.size()]; // 获取下一个点

                // 检查是否与指定的垂直线相交
                if ((p1.x < currentX && p2.x >= currentX) || (p1.x >= currentX && p2.x < currentX) || (p1.x == currentX || p2.x == currentX)) {
                    // 线性插值计算交点
                    double t = (currentX - p1.x) / (p2.x - p1.x);
                    int intersectY = static_cast<int>(p1.y + t * (p2.y - p1.y));
                    intersectionXMidpointLine.push_back(intersectY);
                }
            }

            // 更新最大交点数量及对应的x值
            int numIntersections = intersectionXMidpointLine.size();
            if (numIntersections > maxIntersections) {
                maxIntersections = numIntersections;
                bestX = currentX;
            }
        }

        // 输出最大交点数量
        // std::cout << "最大交点数量: " << maxIntersections << " 在 x = " << bestX << "\n";

        // 判断交点数量
        // 假设A左转 B右转 1: right; 0: left
        if (maxIntersections == 6 || maxIntersections == 7) {
            return 0;
        } else if (maxIntersections >= 9) {
            return 1;
        }
    }

    // 默认返回
    return -1;
}
