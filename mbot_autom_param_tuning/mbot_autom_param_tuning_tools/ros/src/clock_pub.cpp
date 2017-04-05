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

#include <mbot_autom_param_tuning_tools/clock_pub.h>

ClockTimePub::ClockTimePub() : nh_("~"), state_(std::string("INIT"))
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &ClockTimePub::eventInCallBack, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 2);
    pub_clock_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 1);
    
    // set event_in msg data to emtpy string
    reset_component_data();
}

ClockTimePub::~ClockTimePub()
{
    // shut down publishers and subscribers
    pub_clock_.shutdown();
    pub_event_out_.shutdown();
    sub_event_in_.shutdown();
}

void ClockTimePub::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
}

void ClockTimePub::publishWallTime()
{
    ros::WallTime wall_time_now = ros::WallTime::now();
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time(wall_time_now.sec, wall_time_now.nsec); // sec, nsec
    pub_clock_.publish(clock_msg);
}

std::string ClockTimePub::init_state()
{
    // look if e_start control signal was received
    if (event_in_msg_.data == "e_start")
    {
        even_out_msg_.data = std::string("e_started");
        pub_event_out_.publish(even_out_msg_);
        ROS_INFO("Node will start publishing wall time now...");
        return std::string("IDLE");
    }

    return std::string("INIT");
}

std::string ClockTimePub::idle_state()
{
    // look if e_stop control signal was received
    if (event_in_msg_.data == "e_stop")
    {
        even_out_msg_.data = std::string("e_stopped");
        pub_event_out_.publish(even_out_msg_);
        ROS_INFO("Node will stop publishing wall time...");
        
        reset_component_data();
        
        return std::string("INIT");
    }
    
    return std::string("RUNNING");
}

std::string ClockTimePub::running_state()
{
    publishWallTime();

    return std::string("IDLE");
}

void ClockTimePub::reset_component_data()
{
    event_in_msg_.data = std::string("");
}

void ClockTimePub::update()
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
    ros::init(argc, argv, "clock_publisher_node");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (ClockTimePub)
    ClockTimePub clock_publisher_node;
    
    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 20.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    double rate = (1.0 / node_frequency) * 500000;
    
    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        // main loop function
        clock_publisher_node.update();

        // sleep to control the node frequency
        usleep(rate);
    }

    return 0;
}
