/*
 * Copyright [2016] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * Republish geometry_msgs/PoseWithCovarianceStamped as geometry_msgs/PoseStamped
 * 
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PoseWithCovarianceRepublisher
{
    public:
        PoseWithCovarianceRepublisher();
        ~PoseWithCovarianceRepublisher();

        // callback for event_in received msg
        void poseStampedWithCovarianceCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        
        // ros node main loop
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_pose_stamped_;
        ros::Subscriber sub_pose_with_covariance_;

        // flag used to know when we have received a callback
        bool is_pose_w_covariance_msg_received_;

        // stores the received msg in event_in callback (runScriptCallBack)
        geometry_msgs::PoseWithCovarianceStamped pose_with_covariance_msg_;

        // stores the callback msg received in faulty odom topic
        geometry_msgs::PoseStamped pose_stamped_msg_;

};
