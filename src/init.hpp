#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <iostream>
#include <fstream>
#include <cstring>

#include "utils.hpp"
#include "transform.hpp"

double get_gamma(cv::Mat &frame);
void gamma_adjust(cv::Mat &frame, double gamma);
void get_config();
int init();
int camera_init();
int servo_motor_pwmInit();
