#include "hardware/camera.hpp"
#include "vision/blue_barrier.hpp"
#include "vision/preprocess.hpp"
#include "vision/lane_detector.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

static void draw_line(cv::Mat& image, const our_car::ImageLine& line, cv::Scalar color) {
    if (!line.valid || std::abs(line.m) < 1e-9) return;
    cv::Point p1(int(-line.b/line.m),0);
    cv::Point p2(int((image.rows-1-line.b)/line.m),image.rows-1);
    if (cv::clipLine(image.size(),p1,p2)) cv::line(image,p1,p2,color,2);
}
static void show(const cv::Mat& frame, const our_car::LaneDetector& detector) {
    const our_car::BarrierResult barrier = our_car::detect_blue_barrier(frame);
    cv::Mat edges, view = frame.clone();
    if (!our_car::road_edges(frame,edges)) throw std::runtime_error("Invalid BGR frame");
    const our_car::LaneObservation lane = detector.detect(edges);
    draw_line(view,lane.left,cv::Scalar(0,255,0));
    draw_line(view,lane.right,cv::Scalar(0,255,255));
    const std::string text = std::string("blue=")+std::to_string(barrier.blue_ratio)+
                            (barrier.present ? " PRESENT" : " ABSENT");
    cv::putText(view,text,cv::Point(10,25),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,0,255),2);
    cv::imshow("our_car: camera and detections",view);
    cv::imshow("our_car: road edges",edges);
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "vision_debug --camera ID | --image PATH\nDisplays images only; no GPIO. Press q to exit.\n";
        return 0;
    }
    if (argc != 3) { std::cerr << "Usage: vision_debug --camera ID | --image PATH\n"; return 1; }
    try {
        our_car::LaneDetector detector;
        if (std::string(argv[1]) == "--image") {
            cv::Mat image = cv::imread(argv[2]);
            if (image.empty()) throw std::runtime_error("Cannot read image");
            show(image,detector); cv::waitKey(0);
        } else if (std::string(argv[1]) == "--camera") {
            std::istringstream value(argv[2]); int id; std::string extra;
            if (!(value >> id) || id < 0 || (value >> extra)) throw std::runtime_error("Invalid camera ID");
            our_car::Camera camera;
            if (!camera.open(id)) throw std::runtime_error("Cannot open camera");
            our_car::FramePacket frame;
            while (true) {
                if (!camera.read(frame)) throw std::runtime_error("Camera frame read failed");
                show(frame.image,detector);
                if ((cv::waitKey(1)&0xff) == 'q') break;
            }
        } else throw std::runtime_error("Unknown option");
        cv::destroyAllWindows();
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
