#include "check_directions.hpp"

using namespace std;
using namespace cv;

// 处理图像的函数
char check_directions(const cv::Mat& inputImage, cv::Mat& redMask, cv::Mat& contoursImage) {
    Mat lowerQuarter = inputImage;
    Mat hsvImage;
    cvtColor(lowerQuarter, hsvImage, COLOR_BGR2HSV);

    Scalar lowerBlue(90, 30, 90);
    Scalar upperBlue(130, 255, 255);
    inRange(hsvImage, lowerBlue, upperBlue, redMask);

    // 进行轮廓检测
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(redMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 创建轮廓图像并绘制轮廓
    contoursImage = lowerQuarter.clone();
    drawContours(contoursImage, contours, -1, Scalar(0, 255, 0), 1);
	//imshow("out", contoursImage);
    //waitKey(0);
    double leftArea = 0;
    double rightArea = 0;
    double leftWidth = 0; // 左边轮廓的宽度
    double rightWidth = 0; // 右边轮廓的宽度

   for (const auto& contour : contours) {
        if (contour.size() < 80) continue; // 忽略小轮廓

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

        // 计算轮廓的高度
        double height = bottomEdge.y - topEdge.y;

        // 计算与 topEdge.y + width / 2 的平行线的y值
        double lineY = topEdge.y + height * 0.4;

        // 计算与该平行线的交点
        std::vector<int> intersectionXLineY;
        for (size_t i = 0; i < contour.size(); ++i) {
            Point p1 = contour[i];
            Point p2 = contour[(i + 1) % contour.size()]; // 获取下一个点

            // 检查是否与指定的平行线相交
            if ((p1.y < lineY && p2.y >= lineY) || (p1.y >= lineY && p2.y < lineY)) {
                // 线性插值计算交点
                double t = (lineY - p1.y) / (p2.y - p1.y);
                int intersectX = static_cast<int>(p1.x + t * (p2.x - p1.x));
                intersectionXLineY.push_back(intersectX);
            }
        }

        // 如果没有与指定平行线的交点，则跳过此轮廓
        if (intersectionXLineY.empty()) {
            continue; 
        }

        // 计算所有交点的横坐标平均值
        double averageIntersectX = 0.0;
        for (int x : intersectionXLineY) {
            averageIntersectX += x;
        }
        averageIntersectX /= intersectionXLineY.size();

        // 获取轮廓的宽度
        double width = rightEdge.x - leftEdge.x;

        // 输出边界点和高度信息
         /*std::cout << "左边界: (" << leftEdge.x << ", " << leftEdge.y << "), "
                 << "右边界: (" << rightEdge.x << ", " << rightEdge.y << "), "
                 << "上边界: (" << topEdge.x << ", " << topEdge.y << "), "
                 << "下边界: (" << bottomEdge.x << ", " << bottomEdge.y << "), "
                 << "高度: " << height << "\n";*/

        // 判断方向
        if (averageIntersectX < (leftEdge.x + width / 2)) {
            return 0;
        } else {
            return 1;
        }
    }

    // 默认返回
    return -1;
}
