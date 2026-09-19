#pragma once
#include "hardware/pwm_driver.hpp"

namespace our_car {
// One pigpio owner per process; do not run a second GPIO writer alongside it.
class PigpioDriver : public PwmDriver {
public:
    PigpioDriver() = default;
    ~PigpioDriver() override;
    PigpioDriver(const PigpioDriver&) = delete;
    PigpioDriver& operator=(const PigpioDriver&) = delete;
    bool initialize() override;
    bool configure(const PwmChannel& channel) override;
    bool write(int pin, int pwm) override;
    void shutdown() noexcept override;
private:
    bool active_ = false;
};
}  // namespace our_car
