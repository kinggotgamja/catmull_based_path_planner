project(pose_ukf_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  tf_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_ukf
  CATKIN_DEPENDS roscpp geometry_msgs tf tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pose_ukf
  src/PoseUKF.cpp
)

target_link_libraries(pose_ukf
  ${catkin_LIBRARIES}
)

add_executable(pose_ukf_node src/pose_ukf_node.cpp)
target_link_libraries(pose_ukf_node
pose_ukf
  ${catkin_LIBRARIES}
)

install(TARGETS pose_ukf pose_ukf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
