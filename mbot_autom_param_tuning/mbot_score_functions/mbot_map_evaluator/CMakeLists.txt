cmake_minimum_required(VERSION 2.8.3)
project(mbot_map_evaluator)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    geometry_msgs
    sensor_msgs
    tf
)

include_directories(
  common/include
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    geometry_msgs
    sensor_msgs
    tf
)

### LIBRARY
add_library(fitness_score common/src/fitness_score.cpp)

### EXECUTABLES
add_executable(occupancy_grid_to_pointcloud ros/src/occupancy_grid_to_pointcloud.cpp)
target_link_libraries(occupancy_grid_to_pointcloud ${catkin_LIBRARIES})

add_executable(iterative_closest_point_eval ros/src/iterative_closest_point_eval.cpp)
target_link_libraries(iterative_closest_point_eval ${catkin_LIBRARIES} fitness_score)
