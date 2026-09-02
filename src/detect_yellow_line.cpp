#include "detect_yellow_line.hpp"
#include "utils.hpp"

using namespace cv;
using namespace std;

bool detect_yellow_line(Mat img) { //red line, actually
    Mat cropped_image, mask;
    // Scalar lowerb = Scalar(26, 43, 46); // 黄色的HSV下界
    // Scalar upperb = Scalar(34, 255, 255); // 黄色的HSV上界
    Scalar lowerb = Scalar(155, 30, 40); // 黄色的HSV下界
    Scalar upperb = Scalar(180, 250, 255); // 黄色的HSV上界

    int height = img.rows;
    int width = img.cols;
    int half_height = int(height / 2);

    // 裁剪图像下半部分
    cropped_image = img(Rect(0, half_height, width, half_height)).clone();
    cvtColor(cropped_image, cropped_image, COLOR_BGR2HSV);
    inRange(cropped_image, lowerb, upperb, mask); // 生成黄色部分的掩膜
    morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));

    // 检测轮廓
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    bool yellow_detected = false;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
	cout << "area: " << area << endl;
        if (area > yellow_area_threshold) {
            Point2f center;
            float radius;
            minEnclosingCircle(contours[i], center, radius);
	    cout << "r: " << radius << endl;
            if (radius > yellow_min_radius) {
                yellow_detected = true;
            }
        }
    }

    return yellow_detected;
}
