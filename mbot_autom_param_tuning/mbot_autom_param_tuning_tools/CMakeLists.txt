cmake_minimum_required(VERSION 2.8.3)
project(mbot_autom_param_tuning_tools)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    rosgraph_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    rosgraph_msgs
)

include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES
add_executable(clock_pub ros/src/clock_pub.cpp)
target_link_libraries(clock_pub ${catkin_LIBRARIES})
