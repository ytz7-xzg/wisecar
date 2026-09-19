#pragma once

namespace our_car {

// Values are gpioPWM duty units within range, not microseconds or vehicle speed.
struct PwmChannel {
    int pin, frequency, range, minimum, neutral, maximum;
    PwmChannel(int p, int f, int r, int lo, int mid, int hi)
        : pin(p), frequency(f), range(r), minimum(lo), neutral(mid), maximum(hi) {}
    bool valid() const {
        return pin >= 0 && pin <= 31 && frequency > 0 && range >= 25 &&
               range <= 40000 && minimum >= 0 && minimum <= neutral &&
               neutral <= maximum && maximum <= range;
    }
};

class PwmDriver {
public:
    virtual ~PwmDriver() {}
    virtual bool initialize() = 0;
    virtual bool configure(const PwmChannel& channel) = 0;
    virtual bool write(int pin, int pwm) = 0;
    virtual void shutdown() noexcept = 0;
};
}  // namespace our_car
