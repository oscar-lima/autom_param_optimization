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

#include <mbot_pose_evaluator/event_in_based_sm.h>

EventInBasedSM::EventInBasedSM() : nh_("~"), state_(std::string("INIT"))
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &EventInBasedSM::eventInCallBack, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);

    // inform the user about which parameters will be used
    ROS_INFO("Node will run with the following parameters :");
}

EventInBasedSM::~EventInBasedSM()
{
    // shut down subscribers and publishers
    sub_event_in_.shutdown();
    pub_event_out_.shutdown();
}

void EventInBasedSM::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
}

std::string EventInBasedSM::init_state()
{
    if (event_in_msg_.data == "e_start")
    {
        even_out_msg_.data = std::string("e_started");
        pub_event_out_.publish(even_out_msg_);
        ROS_INFO("Node will start execution of running state now...");
        return std::string("IDLE");
    }

    return std::string("INIT");
}

std::string EventInBasedSM::idle_state()
{
    if (event_in_msg_.data == "e_stop")
    {
        even_out_msg_.data = std::string("e_stopped");
        pub_event_out_.publish(even_out_msg_);
        ROS_INFO("Node will stop execution of running state now...");
        
        stop_condition_function();
        
        event_in_msg_.data = std::string("");
        reset_component_data();
        
        return std::string("INIT");
    }
    else if (condition_to_process())
    {
        return std::string("RUNNING");
    }
    else
    {
        return std::string("IDLE");
    }
}

std::string EventInBasedSM::running_state()
{
    running_function();

    return std::string("IDLE");
}

void EventInBasedSM::update()
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

void EventInBasedSM::reset_component_data()
{
    // reset your class variables here
    
    // this function is meant to be overwritten by the derived class
}

void EventInBasedSM::running_function()
{
    // the operation that you need to do in running state
    
    // this function is meant to be overwritten by the derived class
    
    ROS_INFO("Running some code!");
}

void EventInBasedSM::stop_condition_function()
{
    // this function gets executed one time when e_stop control signal is received
    // you could do for example something like publishing an end result
    
    // this function is meant to be overwritten by the derived class
}

bool EventInBasedSM::condition_to_process()
{
    // the condition to enter the running function
    
    // this function is meant to be overwritten by the derived class
    return true;
}
