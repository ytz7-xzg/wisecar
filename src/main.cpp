#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <ctime>

#include "utils.hpp"
#include "init.hpp"
#include "undistort.hpp"
#include "speed_control.hpp"
#include "vector_control.hpp"
#include "vector_fix.hpp"
#include "get_binary_frame.hpp"
#include "get_lines.hpp"
#include "tasks.hpp"

pthread_t trace_thread, task_thread, timestamp_thread;
int *thread_ret;
int main(){
    if (init()) {
        // 初始化出错，退出并返回-1
        std::cout << "Init error (-1)" << std::endl;
        return -1;
    }
    // 检查蓝色挡板，阻塞，直到蓝色挡板移除
    if (check_blue_plain()) {
        // 蓝色挡板识别发生错误，退出并返回-9
        std::cout << "Check blue plain error (-9)" << std::endl;
        return -9;
    }
    std::cout << "蓝色挡板移除" << std::endl;
    if (    pthread_create(&trace_thread, nullptr, trace, nullptr)) {
	    std::cout << "trace error (-10)" << std::endl;
	    return -10;
    }
    usleep(500000);
    target_threshold = 0.34; //0.41
    kick_start(13400);
    usleep(500000);
    set_speed(curve_speed);
    usleep(curve_sleep_time);
    pthread_create(&timestamp_thread, nullptr, save_timestamp_images, nullptr);

    if (pthread_create(&task_thread, nullptr, check_blue_cone, nullptr)) {
        std::cout << "Start blue cone check error (-11)" << std::endl;
        return -11;
    }
    pthread_join(task_thread, (void **)&thread_ret);
    if (*thread_ret) {
        std::cout << "Check blue cone error (-12)" << std::endl;
        return -12;
    }
    delete thread_ret;
    //检测斑马线速度
    set_speed(12800);
    usleep(2000000);
    kick_stop(11600);
    //    kick_start(11600);
        
    if (pthread_create(&task_thread, nullptr, check_crosswalk, nullptr)) {
        // 启动斑马线检测线程出错，退出并返回-2
        std::cout << "Start error (-2)" << std::endl;
        return -2;
    }

    // 阻塞，等待check_crosswalk退出（即识别到斑马线）
    pthread_join(task_thread, (void **)&thread_ret);
    if (*thread_ret) {
        // 线程运行时出错，退出并返回-3
        std::cout << "Check crosswalk error (-3)" << std::endl;
        return -3;
    }
    //刹车，识别箭头标志，准备变道
    auto_mode = false;
    gpioPWM(12,890);
    brake();
    delete thread_ret;
    // // 原视觉识别方向逻辑（赛事规则变化，暂时停用）
    // time_t t0 = time(nullptr);
    // if (pthread_create(&task_thread, nullptr, check_direction2, nullptr)) {
    //     std::cout << "Change track error (-4)" << std::endl;
    //     return -4;
    // }
    // system(play_sound_command);
    // pthread_join(task_thread, (void **)&thread_ret);
    // delete thread_ret;
    // double elapsed = difftime(time(nullptr), t0);
    // if (elapsed < 3.0) usleep((useconds_t)((3.0 - elapsed) * 1000000));
    // // 根据识别线程的结果进行变道

    // 当前规则：刹车后静止三秒并播报语音
    system(play_sound_command);
    usleep(1000000);

    change_track();
    auto_mode = true;
    set_speed(12400);
    usleep(straight_time);
    set_speed(12000);
    usleep(500000);
    while (check_yellow_cones() < 1);
    kick_stop(11300);
    wait_and_return_lane(); // 等待并执行返回变道
    usleep(800000);
    set_speed(13000);
    usleep(straight_time2);
    set_speed(12400);
    usleep(1000000);
    kick_stop(11300);
    if (pthread_create(&task_thread, nullptr, check_stoparea2, nullptr)){
        std::cout << "Check stoparea error (-6)" << std::endl;
        return -6;
    }
    // set_speed(11000);
    // usleep(8500000);
    // set_speed(9500);
    // usleep(300000);
    // set_speed(11600);
    
    pthread_join(task_thread, (void **)&thread_ret);
    
    if (*thread_ret){
        std::cout << "Check stoparea runtime error (-8)" << std::endl;
        return -8;
    }
    
    
    // 识别标志，根据标志位置调整停车区域
    // 1: right; 0: left
    auto_mode = false;
    gpioPWM(12,870);
    brake();
     if (get_symbol()) {
        target_threshold = right_threshold;
        auto_mode = true;
        set_speed(11900);
        // 通过巡线找到停车位置
        usleep(brake_sleep);
        gpioPWM(12,842);
        usleep(300000);
        auto_mode = false;
        gpioPWM(12,875);
        brake();
    } else {
        target_threshold = left_threshold;
        auto_mode = true;
        set_speed(11900);
        // 通过巡线找到停车位置
        usleep(brake_sleep);
        gpioPWM(12,842);
        usleep(300000);
        auto_mode = false;
        gpioPWM(12,875);
        brake();
    }
    
    
    //target_threshold = 0.52;//0.62
    // if (get_symbol()) lane_change_B(servo_range_delta);
    // else lane_change_A(servo_range_delta);
    // 回收巡线线程，结束程序
    // pthread_kill 仅用于检查线程是否存在，要结束线程应使用 pthread_cancel 或设置标志位
    // 回收巡线线程，结束程序
    // pthread_kill 仅用于检查线程是否存在，要结束线程应使用 pthread_cancel 或设置标志位

    // 释放所有资源并运行 QR_detector.py 之前结束拍照线程
    pthread_cancel(timestamp_thread);
    pthread_join(timestamp_thread, NULL);
    cam0.release();
    system("sudo python3 QR.py");
    
    std::cout << "Finished." << std::endl;
    return 0;
}
