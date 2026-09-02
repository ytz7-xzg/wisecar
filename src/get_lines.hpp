#include <vector>
#include "transform.hpp"
#include "utils.hpp"

extern const double track_width;

//extern void (*map_modes[4])(const Line &, const Line &, Line &, Line &);

int get_lines(const cv::Mat& bin_frame, Line& l_line, Line& r_line);
void map_lines(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line, int mode);
void map_mode0(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line);
void map_mode1(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line);
void map_mode2(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line);
void map_mode3(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line);
