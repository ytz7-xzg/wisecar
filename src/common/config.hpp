#pragma once
#include "hardware/chassis.hpp"
#include "control/steering_pd.hpp"
#include <istream>
#include <string>

namespace our_car {
struct ControlConfig {
    ChassisConfig chassis;
    PDConfig pd;
};
// Strict key/value parser; throws on missing file, unknown/duplicate key or invalid value.
// Updates output only when the whole configuration is valid.
void load_control_config(std::istream& input, ControlConfig& output);
ControlConfig load_control_config(const std::string& filename);
}  // namespace our_car
