/* 
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Republish geometry_msgs/PoseWithCovarianceStamped as geometry_msgs/PoseStamped
 * 
 */

#include <mbot_republishers/pose_with_covariance_republisher.h>

PoseWithCovarianceRepublisher::PoseWithCovarianceRepublisher() : nh_("~"), is_pose_w_covariance_msg_received_(false)
{
    // subscriptions
    sub_pose_with_covariance_ = nh_.subscribe("pose_stamped_with_covariance_in", 1, &PoseWithCovarianceRepublisher::poseStampedWithCovarianceCB, this);

    // publications
    pub_pose_stamped_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_stamped_out", 1);
}

PoseWithCovarianceRepublisher::~PoseWithCovarianceRepublisher()
{
    // shut down publishers and subscribers
    sub_pose_with_covariance_.shutdown();
    pub_pose_stamped_.shutdown();
}

void PoseWithCovarianceRepublisher::poseStampedWithCovarianceCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    pose_with_covariance_msg_ = *msg;
    is_pose_w_covariance_msg_received_ = true;
}

void PoseWithCovarianceRepublisher::update()
{   
    // listen to callbacks
    ros::spinOnce();

    if (!is_pose_w_covariance_msg_received_) return;

    // reset flag
    is_pose_w_covariance_msg_received_ = false;
    
    pose_stamped_msg_.header.frame_id = pose_with_covariance_msg_.header.frame_id;
    pose_stamped_msg_.header.stamp = pose_with_covariance_msg_.header.stamp;
    pose_stamped_msg_.pose = pose_with_covariance_msg_.pose.pose;

    // publish the corrected odom value
    pub_pose_stamped_.publish(pose_stamped_msg_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_stamped_republisher_node");

    ROS_INFO("Node is going to initialize ...");

    // create object of the node class (PoseWithCovarianceRepublisher)
    PoseWithCovarianceRepublisher pose_stamped_republisher_node;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 50.0);  // TODO: do rostopic hz on /odom topic and see the value, adjust this one accordingly
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");
    
    while (ros::ok())
    {
        // main loop function
        pose_stamped_republisher_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
