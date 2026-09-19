#include "common/config.hpp"
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace our_car {
void load_control_config(std::istream& input, ControlConfig& output) {
    if (!input.good()) throw std::runtime_error("Configuration stream is not readable");
    ControlConfig c = output;
    std::map<std::string,int*> ints = {
        {"motor_pin",&c.chassis.motor.pin}, {"motor_frequency",&c.chassis.motor.frequency},
        {"motor_range",&c.chassis.motor.range}, {"motor_min",&c.chassis.motor.minimum},
        {"motor_stop",&c.chassis.motor.neutral}, {"motor_max",&c.chassis.motor.maximum},
        {"servo_pin",&c.chassis.servo.pin}, {"servo_frequency",&c.chassis.servo.frequency},
        {"servo_range",&c.chassis.servo.range}, {"servo_center",&c.pd.center},
        {"servo_range_delta",&c.pd.range_delta}
    };
    std::map<std::string,double*> doubles = {
        {"kp",&c.pd.kp},{"kd",&c.pd.kd},{"angle_per_pwm",&c.pd.angle_per_pwm}
    };
    std::set<std::string> seen;
    std::string line; int number = 0;
    while (std::getline(input,line)) {
        ++number;
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        std::istringstream row(line); std::string key, extra;
        if (!(row >> key)) continue;
        if (!seen.insert(key).second) throw std::runtime_error("Duplicate configuration key: "+key);
        bool ok = false;
        if (ints.count(key)) ok = bool(row >> *ints[key]);
        else if (doubles.count(key)) ok = bool(row >> *doubles[key]) && std::isfinite(*doubles[key]);
        else throw std::runtime_error("Unknown configuration key: "+key);
        if (!ok || (row >> extra)) throw std::runtime_error("Invalid configuration line "+std::to_string(number));
    }
    if (input.bad()) throw std::runtime_error("Configuration read failed");
    if (!c.pd.valid()) throw std::runtime_error("Invalid PD parameters");
    c.chassis.servo.neutral = c.pd.center;
    c.chassis.servo.minimum = c.pd.center-c.pd.range_delta;
    c.chassis.servo.maximum = c.pd.center+c.pd.range_delta;
    if (!c.chassis.valid()) throw std::runtime_error("Invalid PWM channel parameters");
    output = c;
}
ControlConfig load_control_config(const std::string& filename) {
    std::ifstream file(filename.c_str());
    if (!file) throw std::runtime_error("Cannot open configuration: "+filename);
    ControlConfig c; load_control_config(file,c); return c;
}
}  // namespace our_car
