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

#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"

#define _USE_MATH_DEFINES

#include <mbot_pose_evaluator/tf_error_calculator.h>

TFErrorCalculator::TFErrorCalculator()
{
    pub_cartesian_difference_ = 
    nh_.advertise<mcr_manipulation_msgs::ComponentWiseCartesianDifference>("tf_error", 1);
    
    getParams();
    
    // delete tf path log files
    ROS_INFO("Deleting old path log file : %s", (path_log_file_.c_str() + std::string("/reference_frame_tf_log.csv")).c_str());
    std::remove((path_log_file_.c_str() + std::string("/reference_frame_tf_log.csv")).c_str());
    ROS_INFO("Deleting old path log file : %s", (path_log_file_.c_str() + std::string("/reference_frame_tf_log.csv")).c_str());
    std::remove((path_log_file_.c_str() + std::string("/reference_frame_tf_log.csv")).c_str());
}

TFErrorCalculator::~TFErrorCalculator()
{
    pub_cartesian_difference_.shutdown();
}

void TFErrorCalculator::getParams()
{
    // get required parameters from parameter server
    nh_.param("global_frame_id", global_frame_id_, std::string("global_frame"));
    nh_.param("reference_frame_id", reference_frame_id_, std::string("reference_frame"));
    nh_.param("target_frame_id", target_frame_id_, std::string("target_frame"));
    nh_.param("path_log_file", path_log_file_, std::string("/home/user/.ros"));
    
    // inform the user about the parameters that will be used
    ROS_INFO("global_frame_id : %s", global_frame_id_.c_str());
    ROS_INFO("reference_frame_id : %s", reference_frame_id_.c_str());
    ROS_INFO("target_frame_id : %s", target_frame_id_.c_str());
    ROS_INFO("path_log_file : %s", path_log_file_.c_str());
}

void TFErrorCalculator::running_function()
{
    try
    {
        //listen to latest available transform
        tf::StampedTransform transform;
        tf_listener_.lookupTransform(reference_frame_id_.c_str(), target_frame_id_.c_str(), now_, transform);
        
        double yaw, pitch, roll;
        transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Vector3 v = transform.getOrigin();
        tf::Quaternion q = transform.getRotation();

        // publish error between frames
        mcr_manipulation_msgs::ComponentWiseCartesianDifference tf_error_msg;
        
        tf_error_msg.header.stamp = now_;
        
        tf_error_msg.header.frame_id = reference_frame_id_.c_str();
        
        tf_error_msg.linear.x = v.getX();
        tf_error_msg.linear.y = v.getY();
        tf_error_msg.linear.z = v.getZ();
        
        tf_error_msg.angular.x = roll;
        tf_error_msg.angular.y = pitch;
        tf_error_msg.angular.z = yaw;
        
        pub_cartesian_difference_.publish(tf_error_msg);

        // get difference between transforms for logging purposes

        // add lines to file instead of deleting and creating a new one
        std::ofstream target_frame_file_handler((path_log_file_ +
            std::string("/target_frame_tf_log.csv")).c_str(), std::ios_base::app | std::ios_base::out);
        std::ofstream reference_frame_file_handler((path_log_file_.c_str() +
            std::string("/reference_frame_tf_log.csv")).c_str(), std::ios_base::app | std::ios_base::out);

        tf_listener_.lookupTransform(global_frame_id_.c_str(), target_frame_id_.c_str(), now_, transform);
        v = transform.getOrigin();

        target_frame_file_handler << global_frame_id_.c_str() << " to " <<
            target_frame_id_.c_str() << ", " << v.getX() << ", " << v.getY() << std::endl;

        tf_listener_.lookupTransform(global_frame_id_.c_str(), reference_frame_id_.c_str(), now_, transform);
        v = transform.getOrigin();

        reference_frame_file_handler << global_frame_id_.c_str() << " to " <<
            reference_frame_id_.c_str() << ", " << v.getX() << ", " << v.getY() << std::endl;

        target_frame_file_handler.close();
        reference_frame_file_handler.close();
    }
    catch(tf::TransformException& ex)
    {
        // since we are using previously can transform, this code should not get executed
        ROS_ERROR("Lookup transform failed with the following error msg : %s", ex.what());
    }
}

bool TFErrorCalculator::condition_to_process()
{
    // the condition to enter the running function
    
    // get the current time from ROS time api
    now_ = ros::Time::now();
    
    // wait 1 second for transform to become available
    tf_listener_.waitForTransform(reference_frame_id_.c_str(), target_frame_id_.c_str(),
                              now_, ros::Duration(1.0));
    
    std::string error = "";
    
    // check if transform is available
    if (tf_listener_.canTransform (target_frame_id_.c_str(), reference_frame_id_.c_str(), now_, &error))
    {
        ROS_DEBUG("transform is available");
        return true;
    }
    else
    {
        ROS_DEBUG("Transform between %s [source] and %s [target] not available", 
                 reference_frame_id_.c_str(), target_frame_id_.c_str());
        ROS_WARN("Can transform failed with the following error msg : %s", error.c_str());
    }
    
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_error_calculator");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class
    TFErrorCalculator derived_tf_error_calculator;
    EventInBasedSM *tf_error_calculator = &derived_tf_error_calculator;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        // main loop function
        tf_error_calculator->update();

        // sleep to control the node frequency with ROS time api
        loop_rate.sleep();
        
        // sleep to control the node frequency with wall time
        // usleep(int(1.0 / node_frequency));
    }

    return 0;
}
