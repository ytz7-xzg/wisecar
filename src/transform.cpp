#include <transform.hpp>

double measured_height          = 29.70;        //camera to the ground  /centimeter
double measured_distance_near   = 65.30;        //camera to near point  /centimeter
double measured_distance_far    = 239.30;       //camera to far point   /centimeter
double measured_length_near     = 123.70;       //length of near line   /centimeter
double measured_length_far      = 375.20;       //length of far line    /centimeter

double bottom_angle;
double center_angle;
double tan_side_angle;
double depth;
cv::Point2d pos_map[60][320];

void initialize_math_info(){
    double middlex = 160, temp;
    bottom_angle = acos(measured_height / measured_distance_near);
    center_angle = acos(measured_height / measured_distance_far);
    depth = 60 / tanf64(center_angle - bottom_angle);
    tan_side_angle = tanf64((atan2f64(measured_length_near * 0.5, measured_distance_near)
        + atan2f64(measured_length_far * 0.5, measured_distance_far)) * 0.5);
    
    for (int i = 0; i < 60; i++) {
        for (int j = 0; j < 320; j++) {
            pos_map[i][j].y = measured_height * tanf64(center_angle - atan2f64(i, depth));
            pos_map[i][j].x = 
                sqrt(pow(measured_height, 2) + pow(pos_map[i][j].y, 2))
                / sqrt(pow(depth, 2) + pow(i, 2)) * (j - middlex);
        }
    }
}
