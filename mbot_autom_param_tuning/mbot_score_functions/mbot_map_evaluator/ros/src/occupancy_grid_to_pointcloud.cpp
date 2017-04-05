/* 
 * Copyright [2016] <Instituto Superior Tecnico>  
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Listens to map topic (nav_msgs::OccupancyGrid) and publishes its occupied cells as a pointcloud.
 * 
 */

#include <mbot_map_evaluator/occupancy_grid_to_pointcloud.h>

OccupancyGridToPointcloud::OccupancyGridToPointcloud() : nh_("~"), is_eval_map_received_(false)
{
    // subscriptions
    sub_eval_map_ = nh_.subscribe("eval_map", 1, &OccupancyGridToPointcloud::evalMapCallBack, this);
    sub_event_in_ = nh_.subscribe("event_in", 1, &OccupancyGridToPointcloud::eventInCallBack, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    pub_map_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("map_cloud", 1);
    
    // getting parameters from parameter server
    nh_.param("map_cost_to_convert", map_cost_to_convert_, 100); // the cost value to be converted to pointcloud, 100 : occupied cells
}

OccupancyGridToPointcloud::~OccupancyGridToPointcloud()
{
    // shut down publishers and subscribers
    pub_event_out_.shutdown();
    pub_map_cloud_.shutdown();
    sub_event_in_.shutdown();
    sub_eval_map_.shutdown();
}

void OccupancyGridToPointcloud::evalMapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    eval_map_ = *msg;
    ROS_DEBUG("Received map to be converted into pointcloud ! (eval_map)");
    is_eval_map_received_ = true;
}

void OccupancyGridToPointcloud::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

int OccupancyGridToPointcloud::getMapValue(const nav_msgs::OccupancyGrid& map, int i, int j)
{
    // return element i, j of a flattened matrix into a vector
    return map.data.at(map.info.width * i + j);
}

void OccupancyGridToPointcloud::mapToWorld(const nav_msgs::OccupancyGrid& map, int i, int j, double& world_x, double& world_y)
{   
    // multiply by resolution will give you the map coordinates
    world_x = (double) j * map.info.resolution + map.info.resolution / 2.0 + map.info.origin.position.x;
    world_y = (double) i * map.info.resolution + map.info.resolution / 2.0 + map.info.origin.position.y;
}

pcl::PointCloud<pcl::PointXYZ> OccupancyGridToPointcloud::mapToPointcloud(const nav_msgs::OccupancyGrid& map)
{
    // for storing and return the pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int x_size = map.info.height;
    int y_size = map.info.width;
    
    // for transforming map to world coordinates
    double world_x;
    double world_y;

    for (int i = 0; i < x_size ; i++)
    {
        for (int j = 0; j < y_size ; j++)
        {
            // if cell is occupied by obstacle then add the centroid of the cell to the cloud
            if (getMapValue(map, i, j) == map_cost_to_convert_)
            {
                // get world coordinates of current occupied cell
                mapToWorld(map, i, j, world_x, world_y);

                // adding occupied cell centroid coordinates to cloud
                cloud.push_back(pcl::PointXYZ(world_x, world_y, 0));
            }
        }
    }

    return cloud;
}

sensor_msgs::PointCloud2 OccupancyGridToPointcloud::publishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                          ros::Publisher &cloud_pub, std::string frame_id)
{
    ROS_DEBUG("Publishing map cloud");

    // Print points of the cloud in terminal
    pcl::PointCloud<pcl::PointXYZ>::const_iterator cloud_iterator = cloud.begin();

    int numPoints = 0;

    while (cloud_iterator != cloud.end())
    {
        ROS_DEBUG("cloud [%d] = %lf, %lf, %lf ", numPoints, cloud_iterator->x, cloud_iterator->y, cloud_iterator->z);
        ++cloud_iterator;
        numPoints++;
    }

    ROS_DEBUG("total number of points in the cloud = %d", numPoints);

    // Creating a pointcloud2 data type
    pcl::PCLPointCloud2 cloud2;

    // Converting normal cloud to pointcloud2 data type
    pcl::toPCLPointCloud2(cloud, cloud2);

    // declaring a ros pointcloud data type
    sensor_msgs::PointCloud2 ros_cloud;

    // converting pointcloud2 to ros pointcloud
    pcl_conversions::fromPCL(cloud2, ros_cloud);

    // assigning a frame to ros cloud
    ros_cloud.header.frame_id = frame_id;

    // publish the cloud
    cloud_pub.publish(ros_cloud);

    // returning the cloud, it could be useful for other components
    return ros_cloud;
}

void OccupancyGridToPointcloud::update()
{   
    // listen to callbacks
    ros::spinOnce();

    if (!is_event_in_received_) return;

    // prepare feedback
    std_msgs::String event_out_msg;
    event_out_msg.data = "e_failure";
    
    // reset flag
    is_event_in_received_ = false;

    // checking for event in msg content
    if (event_in_msg_.data != "e_trigger")
    {
        ROS_ERROR("Received unsupported event: %s, valid event: e_trigger", event_in_msg_.data.c_str());
        pub_event_out_.publish(event_out_msg);
        return;
    }
    
    // we only proceed if map has been received
    if (!is_eval_map_received_)
    {
        ROS_ERROR("Map to be converted to pointcloud not received ! (eval_map)");
        pub_event_out_.publish(event_out_msg);
        return;
    }

    // ensure that map has no rotation (is not supported)
    bool is_map_having_rotation = false;
    // quaternion x must be 0.0
    if (fabs(eval_map_.info.origin.orientation.x) > std::numeric_limits<double>::epsilon()) is_map_having_rotation = true;
    // quaternion y must be 0.0
    if (fabs(eval_map_.info.origin.orientation.y) > std::numeric_limits<double>::epsilon()) is_map_having_rotation = true;
    // quaternion z must be 0.0
    if (fabs(eval_map_.info.origin.orientation.z) > std::numeric_limits<double>::epsilon()) is_map_having_rotation = true;
    // w = 1.0
    // quaternion w must be smaller than 1.0 + small number
    if (eval_map_.info.origin.orientation.w > (1.0 + std::numeric_limits<double>::epsilon())) is_map_having_rotation = true;
    // quaternion w must be greater than 1.0 - small number
    if (eval_map_.info.origin.orientation.w < (1.0 - std::numeric_limits<double>::epsilon())) is_map_having_rotation = true;
    
    if (is_map_having_rotation)
    {
        ROS_ERROR("Error, this node currently does not support map rotation ! (eval_map)");
        pub_event_out_.publish(event_out_msg);
        return;
    }
    
    // reset evaluation map flag
    is_eval_map_received_ = false;
    
    // convert map to pointcloud
    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    map_cloud = mapToPointcloud(eval_map_);
    
    // publish map cloud for visualization purposes
    publishCloud(map_cloud, pub_map_cloud_, eval_map_.header.frame_id);
    
    // feedback : reporting success
    ROS_INFO("map succesfully converted into pointcloud !");
    event_out_msg.data = "e_success";
    pub_event_out_.publish(event_out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupancy_grid_to_pointcloud_node");

    ROS_INFO("Node is going to initialize ...");

    // create object of the node class (OccupancyGridToPointcloud)
    OccupancyGridToPointcloud map_evaluator_node;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 30.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    // check if sim time is being used (i.e. comming from rosbag)
    bool sim_time;
    nh.param("/use_sim_time", sim_time, false);

    ROS_INFO("Node initialized.");
    
    while (ros::ok())
    {
        // main loop function
        map_evaluator_node.update();

        // sleep to control the node frequency
        if (sim_time)
        {
            usleep(1.0 / node_frequency);
        }
        else
        {
            loop_rate.sleep();
        }
    }

    return 0;
}
