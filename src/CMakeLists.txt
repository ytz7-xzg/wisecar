cmake_minimum_required(VERSION 3.10)
project(cpp)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -O2 -fvisibility=hidden -s")
include_directories(. ${OpenCV_INCLUDE_DIRS})
message(${OpenCV_INCLUDE_DIRS})

add_executable(main main.cpp
                    utils.cpp
                    init.cpp
                    tasks.cpp

                    undistort.cpp
                    get_binary_frame.cpp
                    transform.cpp
                    get_lines.cpp

                    vector_fix.cpp
                    vector_control.cpp
                    speed_control.cpp
                    
                    check_cones.cpp
		            check_directions.cpp
                    detect_yellow_line.cpp
                    blue_card_find.cpp
                    check_symbol.cpp)
target_link_libraries(main ${OpenCV_LIBS} pigpio)


