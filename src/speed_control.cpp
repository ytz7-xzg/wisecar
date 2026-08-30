#include "speed_control.hpp"

const int stop_speed = 10000;

void brake(){
    gpioPWM(motor_pin, brake_speed);
    usleep(750000);
    gpioPWM(motor_pin, stop_speed);
}


void kick_start(int target_speed){
    gpioPWM(motor_pin, target_speed + 500);
    usleep(16667);
    set_speed(target_speed);
}

void kick_stop(int target_speed){
    gpioPWM(motor_pin, 8500);
    usleep(200000);
    set_speed(target_speed);
}


void set_speed(int speed){
    gpioPWM(motor_pin, speed);
    double ds = speed - 10000.0;
    threshold_l = (ds < 0.0 ? 0.0 : (ds > 3200.0 ? 3200.0 : ds)) / 3200.0 * 122.0;
}

void lane_change_A(int target_angle){
    gpioPWM(servo_pin, 847);
    kick_start(11500);
    usleep(400000);
    gpioPWM(servo_pin, 852 + 120);
    usleep(540000);
    gpioPWM(servo_pin, 852 - 120);
    usleep(480000);
    gpioPWM(servo_pin, 865);
    usleep(1600000);
    brake();
}

void lane_change_B(int target_angle){
    gpioPWM(servo_pin, 847);
    kick_start(11500);
    usleep(400000);
    gpioPWM(servo_pin, 852 - target_angle);
    usleep(470000);
    gpioPWM(servo_pin, 852 + target_angle);
    usleep(540000);
    gpioPWM(servo_pin, 845);
    usleep(1550000);
    brake();
}
