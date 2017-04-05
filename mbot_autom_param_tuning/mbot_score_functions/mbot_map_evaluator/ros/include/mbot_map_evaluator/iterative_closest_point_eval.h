/*
 * Copyright [2016] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Listens to map topic (nav_msgs::OccupancyGrid) and publishes its occupied cells as a pointcloud.
 * 
 */

#ifndef MBOT_AUTOMATIC_PARAM_TUNING_ITERATIVE_CLOSEST_POINT_EVAL_H
#define MBOT_AUTOMATIC_PARAM_TUNING_ITERATIVE_CLOSEST_POINT_EVAL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <mbot_map_evaluator/fitness_score.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

class IterativeClosestPointEval
{
    public:
        IterativeClosestPointEval();
        ~IterativeClosestPointEval();
        
        // callback to receive the reference pointcloud
        void sourcePointcloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);

        // callback to receive the evaluation pointcloud
        void targetPointcloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);

        // to control moment of execution of action of this node
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);
        
        // to publish the matched pointcloud, product of rotating the eval cloud towards the reference cloud
        sensor_msgs::PointCloud2 publishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                   ros::Publisher &cloud_pub, std::string frame_id);

        // main loop function
        void update();

    private:

        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_evaluation_;
        ros::Publisher pub_match_orientation_;
        ros::Publisher pub_matched_cloud_;
        ros::Subscriber sub_event_in_;
        ros::Subscriber sub_reference_pcl_;
        ros::Subscriber sub_evaluation_pcl_;

        // flags used to know when we have received a callback
        bool is_event_in_received_;
        bool is_source_pcl_received_;
        bool is_target_pcl_received_;
        
        // flag to determine if pointclouds are going to be rotated and translated to match them
        bool match_pointclouds_;
        
        // the maximum distance after which points are ignored for error calculation
        double max_correspondance_distance_;
        
        // stores the messages received in the callbacks
        std_msgs::String event_in_msg_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_pointcloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_pointcloud_;
        
        // store frame_id of the pointclouds
        std::string source_pointcloud_frame_id_;
        std::string target_pointcloud_frame_id_;
        
        // calculate mean square error between 2 clouds without matching them
        FitnessScore fitness_score_calc_;
};
#endif  // MBOT_AUTOMATIC_PARAM_TUNING_ITERATIVE_CLOSEST_POINT_EVAL_H
