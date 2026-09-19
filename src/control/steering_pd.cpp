#include "control/steering_pd.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace our_car {
bool PDConfig::valid() const {
    return std::isfinite(kp) && std::isfinite(kd) && kp >= 0 && kd >= 0 &&
           std::isfinite(angle_per_pwm) && angle_per_pwm > 0 &&
           center >= 0 && center <= 40000 && range_delta >= 0 &&
           range_delta <= center && range_delta <= 40000-center;
}
SteeringPD::SteeringPD(const PDConfig& config) : config_(config) {
    if (!config_.valid()) throw std::invalid_argument("Invalid PD configuration");
}
bool SteeringPD::compute(double error, int& pwm) {
    pwm = config_.center;
    if (!std::isfinite(error)) { reset(); return false; }
    const double correction = (config_.kp*error + config_.kd*(error-last_error_)) /
                              config_.angle_per_pwm;
    if (!std::isfinite(correction)) { reset(); return false; }
    last_error_ = error;
    // Clamp before integer conversion, preserving legacy truncation within bounds.
    const double limited = std::max(-double(config_.range_delta),
                                   std::min(double(config_.range_delta),correction));
    pwm = config_.center - static_cast<int>(limited);
    return true;
}
}  // namespace our_car
