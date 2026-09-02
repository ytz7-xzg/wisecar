#include <pigpio.h>
#include <unistd.h>

#include "utils.hpp"

void brake();
void kick_start(int target_speed);
void kick_stop(int target_speed);
void set_speed(int speed);
void lane_change_A(int target_angle);
void lane_change_B(int target_angle);
