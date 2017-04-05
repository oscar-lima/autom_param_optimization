/*
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Generic class with event based state machine behavior.
 * This class is ment to be used as base class from which you can inherit
 * to create your own nodes with a event_based structure.
 * 
 */

#ifndef MBOT_POSE_EVALUATOR_EVENT_IN_BASED_SM_NODE_H
#define MBOT_POSE_EVALUATOR_EVENT_IN_BASED_SM_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

class EventInBasedSM
{
    public:
        EventInBasedSM();
        ~EventInBasedSM();
        
        // callback for event_in received msg
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);
        
        // initial node state, before receiving e_start control signal
        std::string init_state();
        
        // do not perform any operation, just listen to control signal
        std::string idle_state();
        
        // execute running function, then return to idle state
        std::string running_state();
        
        // ros node main loop, listens to callbacks then executes state as in state_ variable
        void update();
        
        // usually to set values to zero
        virtual void reset_component_data();
        
        // perform node operation
        virtual void running_function();
        
        // this function gets executed one time when e_stop control signal is received
        virtual void stop_condition_function();
        
        // the condition to enter the running function
        virtual bool condition_to_process();

        // private nodehandle
        ros::NodeHandle nh_;
        
    private:
        
        // control signal based publisher and subscriber interface
        ros::Publisher pub_event_out_;
        ros::Subscriber sub_event_in_;

        // to control the state of the sm
        std::string state_;

        // stores the received msg in event_in callback (eventInCallBack)
        std_msgs::String event_in_msg_;

        // stores the message that will be published on event_out topic
        std_msgs::String even_out_msg_;
};
#endif  // MBOT_POSE_EVALUATOR_EVENT_IN_BASED_SM_NODE_H
