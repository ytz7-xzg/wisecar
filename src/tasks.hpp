#include <pthread.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <climits>
#include <csignal>

#include "utils.hpp"
#include "undistort.hpp"
#include "get_binary_frame.hpp"
#include "get_lines.hpp"
#include "vector_fix.hpp"
#include "vector_control.hpp"
#include "transform.hpp"
#include "speed_control.hpp"
#include "check_cones.hpp"
#include "check_directions.hpp"
#include "detect_yellow_line.hpp"
#include "blue_card_find.hpp"
#include "check_symbol.hpp"

extern bool check_result;

void *trace(void *);
void *check_crosswalk(void *);
void change_track();
void change_track_return();
int check_return_lane_cones();
void wait_and_return_lane();
void *check_direction(void *);
char check_direction_arrow(const cv::Mat&);
void *check_direction2(void *);
void *check_cones(void *);
void *check_yellowline(void *);
void *check_stoparea(void *);
void *check_stoparea2(void *);
bool get_symbol();
int check_blue_plain();
void *check_blue_cone(void *);
int check_yellow_cones();
void *save_timestamp_images(void *);
