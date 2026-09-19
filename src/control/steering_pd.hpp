#pragma once

namespace our_car {
struct PDConfig {
    double kp = 0.4;
    double kd = 0.27;
    double angle_per_pwm = 0.18;
    int center = 847;
    int range_delta = 130;
    bool valid() const;
};

// Legacy discrete PD: kd multiplies per-sample error change. Use a fixed period.
class SteeringPD {
public:
    explicit SteeringPD(const PDConfig& config = PDConfig());
    bool compute(double error_degrees, int& pwm);
    void reset() { last_error_ = 0.0; }
private:
    PDConfig config_;
    double last_error_ = 0.0;
};
}  // namespace our_car
