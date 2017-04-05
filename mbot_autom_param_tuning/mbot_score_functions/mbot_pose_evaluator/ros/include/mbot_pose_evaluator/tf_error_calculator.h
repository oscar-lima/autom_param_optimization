/*
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Publishes the difference between two frames
 * in a component wise manner. (x1-x2, y1-y2, etc.)
 * 
 * mcr_manipulation_msgs/ComponentWiseCartesianDifference msg
 * 
 */

#ifndef MBOT_POSE_EVALUATOR_TF_ERROR_CALCULATOR_NODE_H
#define MBOT_POSE_EVALUATOR_TF_ERROR_CALCULATOR_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <mcr_manipulation_msgs/ComponentWiseCartesianDifference.h>
#include <mbot_pose_evaluator/event_in_based_sm.h>
#include <ios>
#include <fstream>

class TFErrorCalculator : public EventInBasedSM
{
    public:
        TFErrorCalculator();
        ~TFErrorCalculator();
        
        // get parameters from parameter server
        void getParams();
        
        // perform node operation
        void running_function();
        
        // the condition to enter the running function
        bool condition_to_process();

    private:
        // ros related variables
        ros::Publisher pub_cartesian_difference_;
        
        std::string reference_frame_id_;
        std::string target_frame_id_;
        
        // to store the path in which the path will be logged
        std::string path_log_file_;
        
        // the fixed global frame, usually map frame
        std::string global_frame_id_;
        
        tf::TransformListener tf_listener_;
        
        // to store current time and use can transform, wait for transform and 
        // lookup transform with the same time
        ros::Time now_;
};
#endif  // MBOT_POSE_EVALUATOR_TF_ERROR_CALCULATOR_NODE_H
