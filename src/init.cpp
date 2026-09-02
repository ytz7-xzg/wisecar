#include "init.hpp"
#include "tasks.hpp"

const float servo_pwm_range = 10000,
            servo_pwm_frequency = 50,
            servo_pwm_duty_cycle_unlock = 779,
            motor_pwm_range = 40000.0,
            motor_pwm_frequency = 200.0,
            motor_pwm_duty_cycle_unlock = 10000.0;


char temp[1024];

double get_gamma(cv::Mat &frame){
    cv::Mat tmp = cv::Mat(frame, cv::Rect(int(frame.cols * 0.3), int(frame.rows * 0.6), int(frame.cols * 0.3), int(frame.rows * 0.1)));
    double sum = 0;
    cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
    for (int i = 0; i < tmp.rows; i++)
        for (int j = 0; j < tmp.cols; j++){
            sum += tmp.at<uint8_t>(i, j);
        }
    sum /= tmp.rows * tmp.cols;
    return (std::log(sum) - std::log(255)) / (std::log(95) - std::log(255));
}

void gamma_adjust(cv::Mat &frame, double gamma){
    frame.convertTo(frame, CV_64F, 1.0/255, 0);
    cv::pow(frame, 1 / gamma, frame);
    frame.convertTo(frame, CV_8U, 255, 0);
}

void get_config(){
    std::map<std::string, double*> double_vars;
    double_vars["kp"] = &kp;
    double_vars["kd"] = &kd;
    double_vars["slow_speed"] = &slow_speed;
    double_vars["left_threshold"] = &left_threshold;
    double_vars["right_threshold"] = &right_threshold;
    double_vars["crosswalk_min_fill_ratio"] = &crosswalk_min_fill_ratio;
    double_vars["stoparea_min_fill_ratio"] = &stoparea_min_fill_ratio;
    double_vars["stoparea_area_threshold"] = &stoparea_area_threshold;
    double_vars["yellow_area_threshold"] = &yellow_area_threshold;
    double_vars["yellow_min_radius"] = &yellow_min_radius;

    std::map<std::string, int*> int_vars;
    int_vars["hough_threshold"] = &hough_threshold;
    int_vars["hough_length_limit"] = &hough_length_limit;
    int_vars["normal_speed"] = &normal_speed;
    int_vars["brake_speed"] = &brake_speed;
    int_vars["brake_sleep"] = &brake_sleep;
    int_vars["change_track_turn_sleep"] = &change_track_turn_sleep;
    int_vars["change_track_straight_sleep"] = &change_track_straight_sleep;
    int_vars["turn_angle"] = &turn_angle;
    int_vars["cross_cnt"] = &cross_cnt;
    int_vars["servo_center"] = &servo_center;
    int_vars["servo_range_delta"] = &servo_range_delta;
    int_vars["debug_save_images"] = &debug_save_images;
    int_vars["cone_avoid_direction"] = &cone_avoid_direction;
    int_vars["cone_check_time"] = &cone_check_time;
    int_vars["cone_detection_threshold"] = &cone_detection_threshold;
    int_vars["cone_blue_min_area"] = &cone_blue_min_area;
    int_vars["cone_blue_max_area"] = &cone_blue_max_area;
    int_vars["cone_yellow_min_area"] = &cone_yellow_min_area;
    int_vars["cone_yellow_max_area"] = &cone_yellow_max_area;
    int_vars["cone_weather_mode"] = &cone_weather_mode;
    int_vars["direction"] = &direction;
    int_vars["straight_time"] = &straight_time;
    int_vars["straight_time2"] = &straight_time2;
    int_vars["crosswalk_min_points"] = &crosswalk_min_points;
    int_vars["crosswalk_max_points"] = &crosswalk_max_points;
    int_vars["crosswalk_detect_period_us"] = &crosswalk_detect_period_us;
    int_vars["stoparea_detect_period_us"] = &stoparea_detect_period_us;
    int_vars["curve_speed"] = &curve_speed;
    int_vars["curve_sleep_time"] = &curve_sleep_time;
    int_vars["final_change_sleep"] = &final_change_sleep;

    std::fstream file("config.txt", std::ios::in);
    std::string key;
    while (file >> key) {
        if (double_vars.count(key)) {
            file >> *double_vars[key];
        } else if (int_vars.count(key)) {
            file >> *int_vars[key];
        } else if (key == "play_sound_command") {
            char temp[1024];
            file.getline(temp, 1024);
            // trim leading space
            char *p = temp;
            while (*p == ' ') p++;
            strncpy(play_sound_command, p, 1024);
        }
    }

    file.close();
    check_result = (direction == 1);
    {
        double ds0 = normal_speed - 10000.0;
        threshold_l = (ds0 < 0.0 ? 0.0 : (ds0 > 2800.0 ? 2800.0 : ds0)) / 2800.0 * 122.0;
    }
    clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
}

int init(){
    get_config();
    initialize_math_info();
    return servo_motor_pwmInit() & camera_init();
}

int camera_init(){
    cam0.open(0);
    cam2.open(2);
    if (!cam0.isOpened())
    {
        std::cout << "Can not open camera0!" << std::endl;
        return 1;
    }

    if (!cam2.isOpened())
    {
        std::cout << "Can not open camera2!" << std::endl;
        return 1;
    }
    
    cam2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cam2.set(cv::CAP_PROP_FPS, 30);
    cam2.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cam2.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    std::cout << "Camera2 FPS: " << cam2.get(cv::CAP_PROP_FPS) << std::endl;
    std::cout << "Camera2 Frame Width: " << cam2.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "Camera2 Frame Height: " << cam2.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    return 0;
}

int servo_motor_pwmInit()
{
    gpioTerminate();
    if (gpioInitialise() < 0)
    {
        std::cout << "GPIO failed" << std::endl;
        return 1;
    }
    else std::cout << "GPIO initialise ok" << std::endl;
    gpioSetMode(servo_pin, PI_OUTPUT);
    gpioSetPWMfrequency(servo_pin, servo_pwm_frequency);
    gpioSetPWMrange(servo_pin, servo_pwm_range);
    gpioPWM(servo_pin, servo_center);

    gpioSetMode(motor_pin, PI_OUTPUT);
    gpioSetPWMfrequency(motor_pin, motor_pwm_frequency);
    gpioSetPWMrange(motor_pin, motor_pwm_range);
    gpioPWM(motor_pin, motor_pwm_duty_cycle_unlock);

    gpioSetMode(cradlex_pin, PI_OUTPUT);
    gpioSetPWMfrequency(cradlex_pin, servo_pwm_frequency);
    gpioSetPWMrange(cradlex_pin, servo_pwm_range);
    gpioPWM(cradlex_pin, servo_pwm_duty_cycle_unlock);

    gpioSetMode(cradley_pin, PI_OUTPUT);
    gpioSetPWMfrequency(cradley_pin, servo_pwm_frequency);
    gpioSetPWMrange(cradley_pin, servo_pwm_range);
    gpioPWM(cradley_pin, 660);
    return 0;
}
