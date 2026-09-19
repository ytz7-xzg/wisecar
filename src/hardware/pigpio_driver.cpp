#include "hardware/pigpio_driver.hpp"
#include <pigpio.h>

namespace our_car {
PigpioDriver::~PigpioDriver() { shutdown(); }
bool PigpioDriver::initialize() {
    if (active_) return true;
    active_ = gpioInitialise() >= 0;
    return active_;
}
bool PigpioDriver::configure(const PwmChannel& c) {
    if (!active_ || !c.valid()) return false;
    return gpioSetMode(c.pin,PI_OUTPUT) == 0 &&
           gpioSetPWMfrequency(c.pin,c.frequency) == c.frequency &&
           gpioSetPWMrange(c.pin,c.range) > 0;
}
bool PigpioDriver::write(int pin, int pwm) {
    if (!active_ || pin < 0 || pin > 31 || pwm < 0) return false;
    return gpioPWM(pin,pwm) == 0;
}
void PigpioDriver::shutdown() noexcept {
    if (active_) gpioTerminate();
    active_ = false;
}
}  // namespace our_car
