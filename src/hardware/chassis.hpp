#pragma once
#include "hardware/pwm_driver.hpp"

namespace our_car {
struct ChassisConfig {
    // Legacy vehicle values; verify wiring and ESC/servo calibration on your car.
    PwmChannel motor = PwmChannel(6,200,40000,7000,10000,14500);
    PwmChannel servo = PwmChannel(12,50,10000,717,847,977);
    bool valid() const { return motor.valid() && servo.valid() && motor.pin != servo.pin; }
};

// Single owner, single calling thread. The driver must outlive this object.
class Chassis {
public:
    explicit Chassis(PwmDriver& driver, const ChassisConfig& config = ChassisConfig());
    ~Chassis();
    Chassis(const Chassis&) = delete;
    Chassis& operator=(const Chassis&) = delete;
    bool open();
    bool set_motor_pwm(int pwm);
    bool set_steering_pwm(int pwm);
    bool stop();
    void close() noexcept;
    bool is_open() const { return open_; }
private:
    bool output(int pin, int pwm);
    PwmDriver& driver_;
    ChassisConfig config_;
    bool acquired_ = false;
    bool open_ = false;
    bool motor_configured_ = false;
    bool servo_configured_ = false;
};
}  // namespace our_car
