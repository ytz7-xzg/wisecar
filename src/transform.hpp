#include <cmath>
#include <opencv2/opencv.hpp>

extern double measured_height;          //camera to the ground  /millimeter
extern double measured_distance_near;   //camera to near point  /millimeter
extern double measured_distance_far;    //camera to far point   /millimeter
extern double measured_length_near;     //length of near line   /millimeter
extern double measured_length_far;      //length of far line    /millimeter

extern double center_angle; //rad
extern double bottom_angle; //rad
extern double tan_side_angle;
extern double depth;        //distance from view point to screen
extern cv::Point2d pos_map[60][320];

void initialize_math_info();
