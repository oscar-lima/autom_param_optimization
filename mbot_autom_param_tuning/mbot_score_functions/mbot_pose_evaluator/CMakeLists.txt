cmake_minimum_required(VERSION 2.8.3)
project(mbot_pose_evaluator)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    geometry_msgs
    mcr_manipulation_msgs
    tf
)

catkin_python_setup()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    mcr_manipulation_msgs
)

### LIBRARY
add_library(event_in_based_sm ros/src/event_in_based_sm.cpp)

### EXECUTABLES
add_executable(mean_pose_error_accumulator ros/src/mean_pose_error_accumulator.cpp)
target_link_libraries(mean_pose_error_accumulator ${catkin_LIBRARIES})

add_executable(tf_error_calculator ros/src/tf_error_calculator.cpp)
target_link_libraries(tf_error_calculator ${catkin_LIBRARIES} event_in_based_sm)
