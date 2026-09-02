#include <opencv2/opencv.hpp>

#ifndef __INCLUDE_UTILS__
#define __INCLUDE_UTILS__

extern const int    servo_pin,
                    motor_pin,
                    cradlex_pin,
                    cradley_pin;

extern int  hough_result,
            hough_threshold,
            hough_length_limit,
            normal_speed,
            brake_speed,
            brake_sleep,
            change_track_turn_sleep,
            change_track_straight_sleep,
            turn_angle,
	    cross_cnt;

extern int  curve_speed,
            curve_sleep_time;
extern int  final_change_sleep;

extern double   target_threshold,
                kp,
                kd,
                threshold_l,
                slow_speed,
                left_threshold,
                right_threshold,
                yellow_area_threshold,
                yellow_min_radius;

extern int      servo_center,
                servo_range_delta;
extern int      debug_save_images; // 0: 关闭调试保存；1: 开启
extern int      cone_avoid_direction; // 0: left, 1: right
extern int      cone_check_time;      // in seconds
extern int      cone_detection_threshold; // consecutive detections to trigger avoidance
extern int      cone_blue_min_area;
extern int      cone_blue_max_area;
extern int      cone_yellow_min_area;
extern int      cone_yellow_max_area;
extern int      cone_weather_mode; // 0: normal, 1: cloudy
extern int      direction; // 0: 左转, 1: 右转
extern int      crosswalk_min_points; // min contour points to count a blob
extern int      crosswalk_max_points; // max contour points to reject long lanes/shadows
extern int      crosswalk_detect_period_us;
extern double   crosswalk_min_fill_ratio;
extern int      stoparea_detect_period_us;
extern double   stoparea_min_fill_ratio;
extern double   stoparea_area_threshold;
extern int      straight_time;
extern int      straight_time2;

struct Line{
    double m, b;
};

extern cv::Mat input_frame0, input_frame2;
extern cv::Mat binary_frame2;
extern cv::VideoCapture cam0,
                        cam2;

extern cv::Ptr<cv::CLAHE> clahe;
extern Line left_real_line, right_real_line,
            left_screen_line, right_screen_line;

extern char play_sound_command[1024];
extern bool auto_mode;
extern int track_seq;
extern double trace_extra_top_crop_pct;
#endif
