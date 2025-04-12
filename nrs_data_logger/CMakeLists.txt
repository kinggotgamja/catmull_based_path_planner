cmake_minimum_required(VERSION 3.0.2)
project(nrs_data_logger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  roslib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(nrs_logger_node src/nrs_logger_node.cpp)
target_link_libraries(nrs_logger_node ${catkin_LIBRARIES})
