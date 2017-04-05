cmake_minimum_required(VERSION 2.8.3)
project(mbot_republishers)

find_package(catkin REQUIRED
    roscpp
    geometry_msgs
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
)

### EXECUTABLES
add_executable(pose_with_covariance_republisher ros/src/pose_with_covariance_republisher.cpp)
target_link_libraries(pose_with_covariance_republisher ${catkin_LIBRARIES})
