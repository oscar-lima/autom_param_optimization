/*
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Listens to mcr_manipulation_msgs/ComponentWiseCartesianDifference msg
 * over time and computes the mean error along specific dimensions.
 * 
 * The time is computed based on e_trigger and e_stop std_msgs/String msg
 * that must be received in node_name/even_in topic.
 * 
 * You can ignore some dimensions by setting the node parameters.
 * 
 * i.e. for robots moving in 2D environment you can ignore the changes in z
 * roll and pitch, but take into account x, y and yaw.
 * 
 */

#ifndef MBOT_POSE_EVALUATOR_MEAN_POSE_ERROR_ACCUMULATOR_NODE_H
#define MBOT_POSE_EVALUATOR_MEAN_POSE_ERROR_ACCUMULATOR_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <mcr_manipulation_msgs/ComponentWiseCartesianDifference.h>
#include <math.h>
#include <unistd.h>

class MeanPoseError
{
    public:
        MeanPoseError();
        ~MeanPoseError();

        // get parameters from param server
        void getParams();

        // callback for event_in received msg
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);

        // callback to receive the cartesian difference (comming from component wise error calculator node)
        void cartesianDifferenceCallBack(const mcr_manipulation_msgs::ComponentWiseCartesianDifference::ConstPtr& msg);

        // function to sum the linear and angular error
        double accumulate_single_error_reading();

        // initial node state, before receiving e_start control signal
        std::string init_state();

        // do not perform any operation, just listen to control signal
        std::string idle_state();

        // set accumulated reading values back to zero
        void reset_component_data();
        
        // accumulate error readings
        std::string running_state();

        // ros node main loop
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_pose_error_;
        ros::Publisher pub_mean_pose_error_;
        ros::Subscriber sub_event_in_;
        ros::Subscriber sub_cartesian_difference_;

        // flag used to know when we have received a callback
        bool is_cartesian_difference_msg_received_;

        // to store which dimensions to use for the mean error calculation
        bool use_x_error_;
        bool use_y_error_;
        bool use_z_error_;

        bool use_roll_error_;
        bool use_pitch_error_;
        bool use_yaw_error_;

        // to store the amount of received cartesian msgs
        int number_of_received_cartesian_msgs_;

        // pre multiply the linear error by this value
        double linear_error_multiplier_;

        // pre multiply the angular error by this value
        double angular_error_multiplier_;

        // to store the accumulated error
        double accumulated_error_;

        // to control the state of the node
        std::string state_;

        // to store pose error msg
        std_msgs::Float64 error_msg_;

        // to store and publish the computed mean pose error accumulated value
        std_msgs::Float64 mean_error_msg_;

        // stores the received msg in event_in callback (eventInCallBack)
        std_msgs::String event_in_msg_;
        
        // store the msg received in cartesianDifferenceCallBack
        mcr_manipulation_msgs::ComponentWiseCartesianDifference cartesian_difference_msg_;

        // stores the message that will be published on event_out topic
        std_msgs::String even_out_msg_;
};
#endif  // MBOT_POSE_EVALUATOR_MEAN_POSE_ERROR_ACCUMULATOR_NODE_H
