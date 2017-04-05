/* 
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Listens to mcr_manipulation_msgs/ComponentWiseCartesianDifference msg
 * over time and computes the mean error along specified dimensions.
 * 
 * The time interval in which the node accepts new readings is controlled 
 * based on e_start and e_stop std_msgs/String msg that the node is listening
 * in node_name/even_in topic.
 * 
 * NOTE: You can ignore some dimensions by setting the node parameters
 * 
 * "use_<coordinate>_error" (i.e. use_z_error : false)
 * 
 * This is particularly useful for robots moving in 2D environment in which you
 * can ignore the changes in z, roll and pitch, and only accumulate the error
 * in x, y and yaw.
 * 
 */

#include <mbot_pose_evaluator/mean_pose_error_accumulator.h>

MeanPoseError::MeanPoseError() : nh_("~"), is_cartesian_difference_msg_received_(false),
use_x_error_(true), use_y_error_(true), use_z_error_(false),
use_roll_error_(false), use_pitch_error_(false), use_yaw_error_(true),
linear_error_multiplier_(1.0), angular_error_multiplier_(1.0),
number_of_received_cartesian_msgs_(0), accumulated_error_(0.0), state_(std::string("INIT"))
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &MeanPoseError::eventInCallBack, this);
    sub_cartesian_difference_ = nh_.subscribe("cartesian_difference_in", 1, &MeanPoseError::cartesianDifferenceCallBack, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);
    pub_pose_error_ = nh_.advertise<std_msgs::Float64>("pose_error_out", 2);
    pub_mean_pose_error_ = nh_.advertise<std_msgs::Float64>("mean_pose_error_out", 2);

    // init values
    mean_error_msg_.data = 0.0;

    // querying parameters from parameter server
    getParams();
}

MeanPoseError::~MeanPoseError()
{
    // shut down subscribers and publishers
    sub_event_in_.shutdown();
    sub_cartesian_difference_.shutdown();
    pub_event_out_.shutdown();
    pub_pose_error_.shutdown();
    pub_mean_pose_error_.shutdown();
}

void MeanPoseError::getParams()
{
    // get required parameters from parameter server
    nh_.param<bool>("use_x_error", use_x_error_, true);
    nh_.param<bool>("use_y_error", use_y_error_, true);
    nh_.param<bool>("use_z_error", use_z_error_, false);
    nh_.param<bool>("use_roll_error", use_roll_error_, true);
    nh_.param<bool>("use_pitch_error", use_pitch_error_, true);
    nh_.param<bool>("use_yaw_error", use_yaw_error_, true);
    nh_.param<double>("linear_error_multiplier", linear_error_multiplier_, 1.0);
    nh_.param<double>("angular_error_multiplier", angular_error_multiplier_, 1.0);

    // inform the user about which parameters will be used
    ROS_INFO("Node will run with the following parameters :");
    ROS_INFO("use_x_error : %s", use_x_error_ ? "true" : "false");
    ROS_INFO("use_y_error : %s", use_y_error_ ? "true" : "false");
    ROS_INFO("use_z_error : %s", use_z_error_ ? "true" : "false");
    ROS_INFO("use_roll_error : %s", use_roll_error_ ? "true" : "false");
    ROS_INFO("use_pitch_error : %s", use_pitch_error_ ? "true" : "false");
    ROS_INFO("use_yaw_error : %s", use_yaw_error_ ? "true" : "false");
    ROS_INFO("linear_error_multiplier : %lf", linear_error_multiplier_);
    ROS_INFO("angular_error_multiplier : %lf", angular_error_multiplier_);
}

void MeanPoseError::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
}

void MeanPoseError::cartesianDifferenceCallBack(const mcr_manipulation_msgs::ComponentWiseCartesianDifference::ConstPtr& msg)
{
    cartesian_difference_msg_ = *msg;
    is_cartesian_difference_msg_received_ = true;
    number_of_received_cartesian_msgs_ ++;
}

double MeanPoseError::accumulate_single_error_reading()
{
    double linear_and_angular_error_combined = 0.0;

    if (use_x_error_)
        linear_and_angular_error_combined += fabs(linear_error_multiplier_ * cartesian_difference_msg_.linear.x);

    if (use_y_error_)
        linear_and_angular_error_combined += fabs(linear_error_multiplier_ * cartesian_difference_msg_.linear.y);

    if (use_z_error_)
        linear_and_angular_error_combined += fabs(linear_error_multiplier_ * cartesian_difference_msg_.linear.z);

    if (use_roll_error_)
        linear_and_angular_error_combined += fabs(angular_error_multiplier_ * cartesian_difference_msg_.angular.x);
    
    if (use_pitch_error_)
        linear_and_angular_error_combined += fabs(angular_error_multiplier_ * cartesian_difference_msg_.angular.y);
    
    if (use_yaw_error_)
        linear_and_angular_error_combined += fabs(angular_error_multiplier_ * cartesian_difference_msg_.angular.z);

    return linear_and_angular_error_combined;
}

std::string MeanPoseError::init_state()
{
    if (event_in_msg_.data == "e_start")
    {
        even_out_msg_.data = std::string("e_started");
        pub_event_out_.publish(even_out_msg_);
        ROS_INFO("Node will start accumulating error readings now...");
        return std::string("IDLE");
    }

    return std::string("INIT");
}

std::string MeanPoseError::idle_state()
{
    if (event_in_msg_.data == "e_stop")
    {
        even_out_msg_.data = std::string("e_stopped");
        pub_event_out_.publish(even_out_msg_);
        ROS_INFO("Node will stop accumulating error readings...");

        // publish quality score, mean error between the two poses
        pub_mean_pose_error_.publish(mean_error_msg_);
        
        reset_component_data();
        
        return std::string("INIT");
    }
    else if (is_cartesian_difference_msg_received_)
    {
        is_cartesian_difference_msg_received_ = false;
        
        return std::string("RUNNING");
    }
    else
    {
        return std::string("IDLE");
    }
}

void MeanPoseError::reset_component_data()
{
    event_in_msg_.data = std::string("");
    accumulated_error_ = 0.0;
    mean_error_msg_.data = 0.0;
    number_of_received_cartesian_msgs_ = 0;
}

std::string MeanPoseError::running_state()
{
    // compute pose error
    error_msg_.data = accumulate_single_error_reading();

    // accumulate error
    accumulated_error_ += error_msg_.data;
    
    // compute the mean
    mean_error_msg_.data = accumulated_error_ / double(number_of_received_cartesian_msgs_);

    // publish pose error
    pub_pose_error_.publish(error_msg_);

    return std::string("IDLE");
}

void MeanPoseError::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (state_ == "INIT")
    {
        state_ = init_state();
    }
    else if (state_ == "IDLE")
    {
        state_ = idle_state();
    }
    else if (state_ == "RUNNING")
    {
        state_ = running_state();
    }

    ROS_DEBUG("State: %s", state_.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mean_pose_error_node");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (MeanPoseError)
    MeanPoseError mean_pose_error_node;

    // setup node frequency
    double node_frequency = 30.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 30.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        // main loop function
        mean_pose_error_node.update();

        // sleep to control the node frequency
        // loop_rate.sleep();
        usleep(int(1.0 / node_frequency));
    }

    return 0;
}
