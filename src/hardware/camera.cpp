#include "hardware/camera.hpp"
#include <cmath>

namespace our_car {
bool Camera::open(int device, int width, int height, double fps) {
    close();
    if (device < 0 || width <= 0 || height <= 0 || !std::isfinite(fps) || fps <= 0) return false;
    if (!capture_.open(device)) return false;
    capture_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH,width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    capture_.set(cv::CAP_PROP_FPS,fps);
    // Camera drivers can negotiate different dimensions; consumers inspect packet.image.size().
    device_ = device; sequence_ = 0; return true;
}
bool Camera::read(FramePacket& packet) {
    packet = FramePacket();
    cv::Mat image;
    if (!capture_.isOpened() || !capture_.read(image) || image.empty()) return false;
    packet.image = image;
    packet.camera_id = device_;
    packet.sequence = ++sequence_;
    packet.captured_at = std::chrono::steady_clock::now();
    return true;
}
void Camera::close() { capture_.release(); device_ = -1; sequence_ = 0; }
}  // namespace our_car
