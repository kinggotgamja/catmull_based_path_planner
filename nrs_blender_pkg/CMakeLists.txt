cmake_minimum_required(VERSION 3.0.2)
project(nrs_blender_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nrs_profiler
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nrs_profiler src/Profiler.cpp)

add_executable(main_node src/main.cpp)
target_link_libraries(main_node nrs_profiler)

