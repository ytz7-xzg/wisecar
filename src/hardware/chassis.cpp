#include "hardware/chassis.hpp"
#include <algorithm>

namespace our_car {
Chassis::Chassis(PwmDriver& driver, const ChassisConfig& config)
    : driver_(driver), config_(config) {}
Chassis::~Chassis() { close(); }

bool Chassis::open() {
    if (open_) return true;
    if (!config_.valid()) return false;
    try {
        acquired_ = true;  // Also clean up a backend that throws during initialize().
        if (!driver_.initialize()) { close(); return false; }
        // Neutral values are meaningful only after the channel's PWM range is configured.
        motor_configured_ = driver_.configure(config_.motor);
        if (!motor_configured_ || !driver_.write(config_.motor.pin,config_.motor.neutral)) {
            close(); return false;
        }
        servo_configured_ = driver_.configure(config_.servo);
        if (!servo_configured_ || !driver_.write(config_.servo.pin,config_.servo.neutral)) {
            close(); return false;
        }
    } catch (...) {
        close();
        return false;
    }
    open_ = true;
    return true;
}

bool Chassis::output(int pin, int pwm) {
    if (!open_) return false;
    try { if (driver_.write(pin,pwm)) return true; } catch (...) {}
    close();  // Latch closed on I/O failure; caller must explicitly reopen.
    return false;
}
bool Chassis::set_motor_pwm(int pwm) {
    if (!open_) return false;
    if (pwm < config_.motor.minimum || pwm > config_.motor.maximum) {
        stop();
        return false;
    }
    return output(config_.motor.pin,pwm);
}
bool Chassis::set_steering_pwm(int pwm) {
    return output(config_.servo.pin,
                  std::max(config_.servo.minimum,std::min(config_.servo.maximum,pwm)));
}
bool Chassis::stop() { return output(config_.motor.pin,config_.motor.neutral); }
void Chassis::close() noexcept {
    open_ = false;
    if (!acquired_) return;
    // Best effort only: power loss or hardware failure cannot be handled in software.
    if (motor_configured_) {
        try { driver_.write(config_.motor.pin,config_.motor.neutral); } catch (...) {}
    }
    if (servo_configured_) {
        try { driver_.write(config_.servo.pin,config_.servo.neutral); } catch (...) {}
    }
    driver_.shutdown();
    acquired_ = false;
    motor_configured_ = false;
    servo_configured_ = false;
}
}  // namespace our_car
