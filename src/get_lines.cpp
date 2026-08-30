#include "get_lines.hpp"

const double track_width = 137.273;

void (*map_modes[4])(const Line &, const Line &, Line &, Line &) = {
    &map_mode0, &map_mode1, &map_mode2, &map_mode3
};
std::vector<cv::Vec4i> tmp;
int get_lines(const cv::Mat& bin_frame, Line & l_line, Line & r_line) {
	tmp.clear();
    double k, l_b = 0, r_b = 0; cv::Point2d temp, vec, l_line_vec, r_line_vec;
    double l_line_cnt, r_line_cnt; l_line_cnt = r_line_cnt = 0;
    r_line_vec.x = r_line_vec.y = l_line_vec.x = l_line_vec.y = 0;
    int ret = 0;

    HoughLinesP(bin_frame, tmp, 1, M_PI / 180, hough_threshold, hough_length_limit);
    for (auto i : tmp) {
        if (abs(i[0] - i[2]) < 3 || abs(i[1] - i[3]) < 3) continue;

        k = double(i[1] - i[3]) / double(i[0] - i[2]);
        temp = cv::Point2d(double(i[0]), double(i[1]));
        if (abs(k) > 0.25) {
            vec = cv::Point2d(i[0] - i[2], i[1] - i[3]);
            if (k < 0){
                l_line_vec += vec / vec.ddot(vec);
                l_b += (temp.y - (vec.y / vec.x) * temp.x) * sqrt(vec.ddot(vec));
                l_line_cnt += sqrt(vec.ddot(vec));
            }
            else{
                r_line_vec += vec;
                r_b += (temp.y - (vec.y / vec.x) * temp.x) * sqrt(vec.ddot(vec));
                r_line_cnt += sqrt(vec.ddot(vec));
            }
        }
    }
    
    if (l_line_cnt > 1) {
        l_line.m = l_line_vec.y / l_line_vec.x;
        l_line.b = l_b / l_line_cnt;
        ret |= 1;
    }

    if (r_line_cnt > 1) {
        r_line.m = r_line_vec.y / r_line_vec.x;
        r_line.b = r_b / r_line_cnt;
        ret |= 2;
    }
    return ret;
}

void map_lines(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line, int mode) {
    (map_modes[mode])(l_s_line, r_s_line, l_r_line, r_r_line);
}

void map_mode0(const Line &, const  Line &, Line &, Line &){
    return;
}

void map_mode1(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line) {
    cv::Point2d *start, *end, vec;
    double k = l_s_line.m, b = l_s_line.b;
    start = &pos_map[5][int((5 - b) / k)];
    if (55 < k * 5 + b){
        end = &pos_map[55][int((55 - b) / k)];
    }
    else {
        end = &pos_map[int(5 * k + b)][5];
    }
    vec = *end - *start;
    l_r_line.m = vec.x / vec.y;
    l_r_line.b = end->x - end->y * l_r_line.m;
    r_r_line.m = l_r_line.m;
    r_r_line.b = l_r_line.b + track_width / cos(atanf64(r_r_line.m));
	
}

void map_mode2(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line) {
    cv::Point2d *start, *end, vec;
    double k = r_s_line.m, b = r_s_line.b;
    start = &pos_map[5][int((5 - b) / k)];
    if (55 < k * 305 + b) {
        end = &pos_map[55][int((55 - b) / k)];
    }
    else {
        end = &pos_map[int(305 * k + b)][305];
    }
    vec = *end - *start;
    l_r_line.m = r_r_line.m = vec.x / vec.y;
    r_r_line.b = end->x - end->y * r_r_line.m;
    l_r_line.b = r_r_line.b - track_width / cos(atanf64(l_r_line.m));

}

void map_mode3(const Line &l_s_line, const Line &r_s_line,
                Line &l_r_line, Line &r_r_line){
    cv::Point2d *start, *end, vec, mid;
    double k = l_s_line.m, b = l_s_line.b;
    start = &pos_map[5][int((5 - b) / k)];
    if (55 < k * 5 + b){
        end = &pos_map[55][int((55 - b) / k)];
    }
    else {
        end = &pos_map[int(5 * k + b)][5];
    }
    vec = *end - *start;
    mid = (*end + *start) * 0.5;

    k = r_s_line.m, b = r_s_line.b;
    start = &pos_map[5][int((5 - b) / k)];
    if (55 < k * 305 + b) {
        end = &pos_map[55][int((55 - b) / k)];
    }
    else {
        end = &pos_map[int(305 * k + b)][305];
    }
    
    vec += *end - *start;
    r_r_line.m = l_r_line.m = vec.x / vec.y;
    l_r_line.b = mid.x - l_r_line.m * mid.y;
    mid = (*end + *start) * 0.5;
    r_r_line.b = mid.x - r_r_line.m * mid.y;

}
