#include "vector_control.hpp"

double  last_error = 0;
int pd;

void servo_control(double error) {
    pd = servo_center - int((kp * error + kd * (error - last_error)) / 0.18);
    last_error = error;
    pd = std::min(std::max(pd, servo_center - servo_range_delta), servo_center + servo_range_delta);
    gpioPWM(servo_pin, pd);
}
