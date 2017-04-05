/* 
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Listen to rosbag_clock msg and if available republish,
 * if not available then publish wall time instead. This is
 * useful when playing a rosbag data, "sometimes".
 * 
 */

#ifndef MBOT_AUTOM_PARAM_TUNING_TOOLS_CLOCK_PUB_H
#define MBOT_AUTOM_PARAM_TUNING_TOOLS_CLOCK_PUB_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <unistd.h>

class ClockTimePub
{
    public:
        ClockTimePub();
        ~ClockTimePub();

        // to receive control signal event_in
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);
        
        // publish ros wall time in clock topic
        void publishWallTime();
        
        // initial node state, before receiving e_start control signal
        std::string init_state();

        // do not perform any operation, just listen to control signal
        std::string idle_state();
        
        // publish wall time in clock topic
        std::string running_state();
        
        // set event_in as empty string
        void reset_component_data();
        
        // ros node main loop
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_clock_;
        ros::Subscriber sub_event_in_;
        
        // stores the received msg in event_in callback (runScriptCallBack)
        std_msgs::String event_in_msg_;

        // stores the message that will be published on event_out topic
        std_msgs::String even_out_msg_;
        
        // to store the current state of the state machine
        std::string state_;
};
#endif  // MBOT_AUTOM_PARAM_TUNING_TOOLS_CLOCK_PUB_H
