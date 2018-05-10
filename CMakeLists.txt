cmake_minimum_required(VERSION 2.8)
project(tof_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tf2_sensor_msgs
    nav_msgs
    cv_bridge
    tf
    )


find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(tof_mapping src/tof_mapping.cpp)
target_link_libraries(tof_mapping ${catkin_LIBRARIES}  ${OpenCV_LIBS})
