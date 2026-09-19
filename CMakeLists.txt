cmake_minimum_required(VERSION 3.10...3.27)
project(our_car LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OUR_CAR_WITH_VISION "Build OpenCV vision modules and viewer" ON)
option(OUR_CAR_WITH_PIGPIO "Build Raspberry Pi GPIO backend" OFF)
include(CTest)

add_library(our_car_control
    src/common/config.cpp
    src/hardware/chassis.cpp
    src/control/steering_pd.cpp
    src/control/lane_geometry.cpp)
target_include_directories(our_car_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(our_car_control PRIVATE -Wall -Wextra -Wpedantic)

add_executable(control_demo apps/control_demo.cpp)
target_link_libraries(control_demo PRIVATE our_car_control)

if(OUR_CAR_WITH_PIGPIO)
    find_path(PIGPIO_INCLUDE_DIR pigpio.h)
    find_library(PIGPIO_LIBRARY pigpio)
    if(NOT PIGPIO_INCLUDE_DIR OR NOT PIGPIO_LIBRARY)
        message(FATAL_ERROR "pigpio headers/library missing; install pigpio or disable OUR_CAR_WITH_PIGPIO")
    endif()
    find_package(Threads REQUIRED)
    add_library(our_car_pigpio src/hardware/pigpio_driver.cpp)
    target_include_directories(our_car_pigpio PRIVATE ${PIGPIO_INCLUDE_DIR})
    target_link_libraries(our_car_pigpio PUBLIC our_car_control PRIVATE ${PIGPIO_LIBRARY} Threads::Threads rt)
endif()

if(OUR_CAR_WITH_VISION)
    find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d videoio highgui imgcodecs)
    add_library(our_car_vision
        src/hardware/camera.cpp
        src/vision/preprocess.cpp
        src/vision/blue_barrier.cpp
        src/vision/lane_detector.cpp
        src/vision/undistorter.cpp)
    target_include_directories(our_car_vision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(our_car_vision PUBLIC ${OpenCV_LIBS})
    target_compile_options(our_car_vision PRIVATE -Wall -Wextra -Wpedantic)
    add_executable(vision_debug apps/vision_debug.cpp)
    target_link_libraries(vision_debug PRIVATE our_car_vision)
endif()

if(BUILD_TESTING)
    add_executable(test_control tests/test_control.cpp)
    target_link_libraries(test_control PRIVATE our_car_control)
    add_test(NAME control COMMAND test_control)
    if(OUR_CAR_WITH_VISION)
        add_executable(test_vision tests/test_vision.cpp)
        target_link_libraries(test_vision PRIVATE our_car_vision)
        add_test(NAME vision COMMAND test_vision)
    endif()
endif()
