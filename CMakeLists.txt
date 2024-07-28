cmake_minimum_required(VERSION 3.14)
project(obstacle_lane_change)

find_package(autoware_cmake REQUIRED)

autoware_package()

ament_auto_add_executable(obstacle_lane_change
  src/obstacle_lane_change.cpp
)

ament_auto_package(
    INSTALL_TO_SHARE 
    launch
    config
)
