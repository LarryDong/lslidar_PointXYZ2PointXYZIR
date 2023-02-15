cmake_minimum_required(VERSION 2.8.0)
project(lslidar_conversion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        sensor_msgs
        pcl_conversions)

catkin_package()


include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

add_executable(lslidar_convertor src/main.cpp)
target_link_libraries(lslidar_convertor ${catkin_LIBRARIES})
