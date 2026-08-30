#include "vector_fix.hpp"

double  to_angle = 180 / CV_PI;
double get_error_angle(Line &l_line, Line &r_line, double threshold){
    // x = y*m + b
    double b = l_line.b + threshold * (r_line.b - l_line.b), k = r_line.m, 
           d = abs(b) / sqrt(1 + pow(k, 2)),
           h2 = pow(d, 2) + pow(threshold_l, 2);
    cv::Point2d vec(k, 1); vec *= sqrt(h2 / vec.ddot(vec)); vec.x += b;
    return atan2f64(vec.x, vec.y) * to_angle;
}