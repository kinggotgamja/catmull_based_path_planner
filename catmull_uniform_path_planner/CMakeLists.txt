cmake_minimum_required(VERSION 3.0.2)
project(catmull_uniform_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES catmull_uniform_path_planner
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(catmull_uniform_path_planner
  src/catmull_path.cpp
)
target_link_libraries(catmull_uniform_path_planner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(path_example_node src/path_example_node.cpp)
target_link_libraries(path_example_node catmull_uniform_path_planner ${catkin_LIBRARIES})

add_executable(power_example_node src/power_example_node.cpp)
target_link_libraries(power_example_node catmull_uniform_path_planner ${catkin_LIBRARIES})

