cmake_minimum_required(VERSION 3.0.2)
project(nrs_vr_catmull)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  catmull_uniform_path_planner
  pose_ukf_filter
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(path_regen_main src/path_regen_main.cpp)
target_link_libraries(path_regen_main
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(power_gmm_main src/power_gmm_main.cpp)
target_link_libraries(power_gmm_main
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)




