#pragma once
#include <opencv2/videoio.hpp>
#include <chrono>
#include <cstdint>

namespace our_car {
struct FramePacket {
    cv::Mat image;
    int camera_id = -1;
    std::uint64_t sequence = 0;
    std::chrono::steady_clock::time_point captured_at;
};
// One owner/calling thread per camera. Share captured packets with other consumers.
class Camera {
public:
    Camera() = default;
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    bool open(int device, int width = 640, int height = 480, double fps = 30);
    bool read(FramePacket& packet);
    void close();
private:
    cv::VideoCapture capture_;
    int device_ = -1;
    std::uint64_t sequence_ = 0;
};
}  // namespace our_car
