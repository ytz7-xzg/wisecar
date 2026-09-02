#include "utils.hpp"
#include "init.hpp"
#include "undistort.hpp"
#include "get_binary_frame.hpp"
#include "get_lines.hpp"
#include "transform.hpp"
#include "vector_fix.hpp"
#include "vector_control.hpp"
#include "speed_control.hpp"


using namespace cv;
using namespace std;
pthread_t tr;
int main(){
    init();
    kick_start();
    target_threshold = left_threshold;
    usleep(brake_sleep);
    brake();
    return 0;
}
