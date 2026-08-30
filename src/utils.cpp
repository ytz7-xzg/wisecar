#include "utils.hpp"

const int   servo_pin = 12,
            motor_pin = 6,
            cradlex_pin = 24,
            cradley_pin = 25;

int hough_result,
    hough_threshold,
    hough_length_limit,
    normal_speed,
    brake_speed,
    brake_sleep,
    change_track_turn_sleep,
    change_track_straight_sleep,
    turn_angle,
    cross_cnt;

double  target_threshold,
        kp,
        kd,
        threshold_l,
        slow_speed,
        left_threshold,
        right_threshold;;

cv::Mat input_frame0,
        input_frame2,
        binary_frame2;

cv::VideoCapture    cam0,
                    cam2;

cv::Ptr<cv::CLAHE> clahe;
Line    left_real_line, right_real_line,
        left_screen_line, right_screen_line;

char play_sound_command[1024];
bool auto_mode = true;
int track_seq = 0;

int servo_center = 736;
int servo_range_delta = 166;
int debug_save_images = 0;
int cone_avoid_direction = 0; // 0: left, 1: right
int cone_check_time = 30;      // in seconds
int cone_detection_threshold = 3;
int crosswalk_min_points = 55;
int crosswalk_max_points = 1200;
int crosswalk_detect_period_us = 50000;
double crosswalk_min_fill_ratio = 0.15;
int cone_blue_min_area = 250;
int cone_blue_max_area = 20000;
int cone_yellow_min_area = 250;
int cone_yellow_max_area = 20000;
int cone_weather_mode = 0;
int curve_speed = 11600;
int curve_sleep_time = 2000000;
int final_change_sleep = 200000;
double yellow_area_threshold = 59.0;
double yellow_min_radius = 10.0;
int stoparea_detect_period_us = 0;
double stoparea_min_fill_ratio = 0.6;
double stoparea_area_threshold = 8000.0;
int direction = 0;
int straight_time = 1000000;
int straight_time2 = 2400000;
