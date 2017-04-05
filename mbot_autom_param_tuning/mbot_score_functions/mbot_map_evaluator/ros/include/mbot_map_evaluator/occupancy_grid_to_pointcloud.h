/*
 * Copyright [2016] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Listens to map topic (nav_msgs::OccupancyGrid) and publishes its occupied cells as a pointcloud.
 * 
 */

#ifndef MBOT_AUTOMATIC_PARAM_TUNING_OCCUPANCY_GRID_TO_POINTCLOUD_H
#define MBOT_AUTOMATIC_PARAM_TUNING_OCCUPANCY_GRID_TO_POINTCLOUD_H

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

class OccupancyGridToPointcloud
{
    public:
        OccupancyGridToPointcloud();
        ~OccupancyGridToPointcloud();
        
        // to receive the map to be converted into a pointcloud
        void evalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);

        // to control the moment in which the map is going to be converted to a pointcloud
        void eventInCallBack(const std_msgs::String::ConstPtr& msg);

        // receives a map and converts into a pointcloud
        pcl::PointCloud<pcl::PointXYZ> mapToPointcloud(const nav_msgs::OccupancyGrid& map);

        // publish a pointcloud
        sensor_msgs::PointCloud2 publishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                             ros::Publisher &cloud_pub, std::string frame_id);
        
        // main loop function
        void update();

    private:
        
        // to get the value of the map at a certain coordinate
        int getMapValue(const nav_msgs::OccupancyGrid& map, int i, int j);

        // to convert cell to map, based on resolution
        void mapToWorld(const nav_msgs::OccupancyGrid& map, int i, int j, double& world_x, double& world_y);
        
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_map_cloud_;
        ros::Subscriber sub_event_in_;
        ros::Subscriber sub_eval_map_;

        // flag used to know when we have received a callback
        bool is_eval_map_received_;
        bool is_event_in_received_;
        
        // to store from parameter server the value that will be used to convert the map to pcl
        // i.e. 100 : only occupied cells, 0 : only free cells, -1 : only unknown cells
        int map_cost_to_convert_;

        // stores the map to be evaluated
        nav_msgs::OccupancyGrid eval_map_;
        
        // stores the event_in msg received data
        std_msgs::String event_in_msg_;
};
#endif  // MBOT_AUTOMATIC_PARAM_TUNING_OCCUPANCY_GRID_TO_POINTCLOUD_H
