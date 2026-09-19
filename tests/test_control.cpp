#include "hardware/chassis.hpp"
#include "control/steering_pd.hpp"
#include "control/lane_geometry.hpp"
#include "common/config.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace our_car;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

class FakeDriver : public PwmDriver {
public:
    bool init_ok = true;
    int configure_fail_pin = -1;
    bool fail_next_write = false;
    bool throw_next_write = false;
    int throw_configure_pin = -1;
    int unconfigured_writes = 0;
    std::vector<int> configured;
    bool active = false;
    std::vector<std::pair<int,int> > writes;
    bool initialize() override { active = init_ok; return init_ok; }
    bool configure(const PwmChannel& c) override {
        if (c.pin == throw_configure_pin) throw std::runtime_error("Configure exception");
        if (c.pin == configure_fail_pin) return false;
        configured.push_back(c.pin); return true;
    }
    bool write(int pin, int pwm) override {
        if (throw_next_write) { throw_next_write = false; throw std::runtime_error("Write exception"); }
        if (std::find(configured.begin(),configured.end(),pin) == configured.end()) {
            ++unconfigured_writes; return false;
        }
        if (fail_next_write) { fail_next_write = false; return false; }
        writes.push_back(std::make_pair(pin, pwm)); return true;
    }
    void shutdown() noexcept override { active = false; configured.clear(); }
};

void chassis_lifecycle() {
    FakeDriver d;
    Chassis c(d);
    CHECK(d.writes.empty());
    CHECK(!c.set_motor_pwm(11000));
    CHECK(c.open());
    CHECK(d.writes.at(0) == std::make_pair(6,10000));
    CHECK(c.set_motor_pwm(11000));
    CHECK(c.set_steering_pwm(99999));
    CHECK(d.writes.back() == std::make_pair(12,977));
    CHECK(c.stop());
    CHECK(d.writes.back() == std::make_pair(6,10000));
    c.close(); CHECK(!d.active); CHECK(!c.is_open());
}
void failure_stops_output() {
    FakeDriver d; Chassis c(d);
    d.configure_fail_pin = 12;
    CHECK(!c.open()); CHECK(!d.active); CHECK(!c.set_motor_pwm(11000));
    d.configure_fail_pin = -1;
    CHECK(c.open());
    d.fail_next_write = true;
    CHECK(!c.set_motor_pwm(11000)); CHECK(!c.is_open()); CHECK(!d.active);
    CHECK(!c.set_motor_pwm(11000));
    CHECK(c.open());
    CHECK(!c.set_motor_pwm(999999));
    CHECK(d.writes.back() == std::make_pair(6,10000));
}
void invalid_configuration() {
    FakeDriver d; ChassisConfig cfg;
    cfg.servo.pin = cfg.motor.pin;
    Chassis c(d,cfg); CHECK(!c.open()); CHECK(!d.active);
    ControlConfig settings;
    std::istringstream good("# comment\nservo_center 850\nservo_range_delta 100\nkp 0.5\n");
    load_control_config(good,settings);
    CHECK(settings.chassis.servo.neutral == 850);
    CHECK(settings.chassis.servo.minimum == 750);
    CHECK(settings.pd.center == 850);
    bool rejected = false;
    try { std::istringstream bad("kp nan\n"); load_control_config(bad,settings); }
    catch (const std::exception&) { rejected = true; }
    CHECK(rejected);
    rejected = false;
    try { std::istringstream bad("servo_centre 850\n"); load_control_config(bad,settings); }
    catch (const std::exception&) { rejected = true; }
    CHECK(rejected);
}
void pd_limits_and_reset() {
    SteeringPD pd; int pwm = 0;
    CHECK(pd.compute(0,pwm)); CHECK(pwm == 847);
    CHECK(pd.compute(10,pwm)); CHECK(pwm < 847);
    CHECK(pd.compute(1e6,pwm)); CHECK(pwm == 717);
    pd.reset(); CHECK(pd.compute(0,pwm)); CHECK(pwm == 847);
    CHECK(!pd.compute(std::numeric_limits<double>::quiet_NaN(),pwm)); CHECK(pwm == 847);
    CHECK(pd.compute(0,pwm)); CHECK(pwm == 847);
}
void geometry_validity() {
    GroundLine left(0,-50), right(0,50);
    HeadingError e = heading_error(left,right,0.5,100);
    CHECK(e.valid); CHECK(std::abs(e.degrees) < 1e-9);
    CHECK(heading_error(left,right,0.8,100).degrees > 0);
    CHECK(heading_error(left,right,0.2,100).degrees < 0);
    CHECK(!heading_error(right,left,0.5,100).valid);
    CHECK(!heading_error(left,right,0.5,0).valid);
    CHECK(!heading_error(left,right,2,100).valid);
    CHECK(!heading_error(GroundLine(),right,0.5,100).valid);
    CHECK(!heading_error(GroundLine(0.5,-50),right,0.5,100).valid);
    CHECK(lookahead_from_pwm(10000) == 0);
    CHECK(std::abs(lookahead_from_pwm(13200)-122.0) < 1e-9);
}
void failed_init_never_writes_unconfigured_pins() {
    FakeDriver d; Chassis c(d);
    d.configure_fail_pin = 6;
    CHECK(!c.open()); CHECK(d.unconfigured_writes == 0); CHECK(!d.active);
    d.configure_fail_pin = 12;
    CHECK(!c.open()); CHECK(d.unconfigured_writes == 0); CHECK(!d.active);
}
void driver_exceptions_close_chassis() {
    FakeDriver d; Chassis c(d);
    CHECK(c.open()); CHECK(c.set_motor_pwm(11000));
    d.throw_next_write = true;
    bool escaped = false, ok = true;
    try { ok = c.set_motor_pwm(12000); } catch (...) { escaped = true; }
    CHECK(!escaped && !ok); CHECK(!c.is_open()); CHECK(!d.active);
    d.throw_configure_pin = 12;
    try { ok = c.open(); } catch (...) { escaped = true; }
    CHECK(!escaped && !ok); CHECK(!d.active); CHECK(d.unconfigured_writes == 0);
}
int main() {
    const std::pair<const char*,void(*)()> cases[] = {
        {"chassis lifecycle",chassis_lifecycle}, {"failure stops output",failure_stops_output},
        {"invalid configuration",invalid_configuration}, {"PD limits/reset",pd_limits_and_reset},
        {"geometry validity",geometry_validity},
        {"unconfigured pins",failed_init_never_writes_unconfigured_pins},
        {"driver exceptions",driver_exceptions_close_chassis}
    };
    int failures = 0;
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS: " << test.first << '\n'; }
        catch (const std::exception& e) { ++failures; std::cerr << "FAIL: " << test.first << ": " << e.what() << '\n'; }
    }
    return failures ? 1 : 0;
}
