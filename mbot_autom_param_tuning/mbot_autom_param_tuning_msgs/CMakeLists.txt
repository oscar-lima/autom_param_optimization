cmake_minimum_required(VERSION 2.8.3)
project(mbot_autom_param_tuning_msgs)

find_package(catkin REQUIRED 
    genmsg 
    actionlib_msgs 
    actionlib
)

add_action_files(DIRECTORY action FILES 
    RunAlgorithmInstance.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()
