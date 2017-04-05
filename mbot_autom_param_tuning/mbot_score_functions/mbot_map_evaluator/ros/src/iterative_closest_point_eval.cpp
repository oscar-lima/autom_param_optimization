/* 
 * Copyright [2016] <Instituto Superior Tecnico>
 * 
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 * 
 * Subscribes to two pointclouds and compares them using the method iterative closest point from pcl.
 * 
 */

#include <mbot_map_evaluator/iterative_closest_point_eval.h>

IterativeClosestPointEval::IterativeClosestPointEval() : nh_("~"), is_source_pcl_received_(false),
is_target_pcl_received_(false), source_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>),
target_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>), fitness_score_calc_(0.0), match_pointclouds_(false),
is_event_in_received_(false), source_pointcloud_frame_id_(""), target_pointcloud_frame_id_(""), 
max_correspondance_distance_(0.0)
{
    // subscriptions
    sub_event_in_ = nh_.subscribe("event_in", 1, &IterativeClosestPointEval::eventInCallBack, this);
    sub_reference_pcl_ = nh_.subscribe("source_pointcloud", 1, &IterativeClosestPointEval::sourcePointcloudCallBack, this);
    sub_evaluation_pcl_ = nh_.subscribe("target_pointcloud", 1, &IterativeClosestPointEval::targetPointcloudCallBack, this);

    // publications
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
    pub_evaluation_ = nh_.advertise<std_msgs::Float64>("cloud_quality", 1);
    pub_match_orientation_ = nh_.advertise<geometry_msgs::PoseStamped>("match_orientation_pose", 1);
    pub_matched_cloud_ = nh_.advertise<sensor_msgs::PointCloud2> ("matched_cloud", 1);
    
    // getting from parameter server the maximum range for the fitness score
    // the maximum value to search for nearest neighbor
    nh_.param("max_correspondance_distance", max_correspondance_distance_, 10.0);
    fitness_score_calc_.setMaxRange(max_correspondance_distance_);
    
    // getting for parameter server whether to match the pointclouds or not
    nh_.param("match_pointclouds", match_pointclouds_, false);
    
    // informing the user about which parameters will be used:
    ROS_INFO("Iterative closest point parameters : ");
    ROS_INFO("max_correspondance_distance_ : %lf", max_correspondance_distance_);
    if (match_pointclouds_) ROS_INFO("match_pointclouds : true");
    else ROS_INFO("match_pointclouds : false");
}

IterativeClosestPointEval::~IterativeClosestPointEval()
{
    // shut down publishers and subscribers
    pub_event_out_.shutdown();
    pub_evaluation_.shutdown();
    pub_match_orientation_.shutdown();
    pub_matched_cloud_.shutdown();
    sub_event_in_.shutdown();
    sub_reference_pcl_.shutdown();
    sub_evaluation_pcl_.shutdown();
}

void IterativeClosestPointEval::sourcePointcloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    source_pointcloud_frame_id_ = input->header.frame_id;
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*source_pointcloud_);
    
    ROS_INFO("Received source pointclouds !");
    is_source_pcl_received_ = true;
}

void IterativeClosestPointEval::targetPointcloudCallBack(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    target_pointcloud_frame_id_ = input->header.frame_id;
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*target_pointcloud_);
    
    ROS_INFO("Received evaluation pointcloud !");
    is_target_pcl_received_ = true;
}

void IterativeClosestPointEval::eventInCallBack(const std_msgs::String::ConstPtr& msg)
{
    event_in_msg_ = *msg;
    is_event_in_received_ = true;
}

sensor_msgs::PointCloud2 IterativeClosestPointEval::publishCloud(pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                          ros::Publisher &cloud_pub, std::string frame_id)
{
    ROS_DEBUG("Publishing matched cloud");

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

void IterativeClosestPointEval::update()
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
    if (event_in_msg_.data != "e_start")
    {
        ROS_ERROR("Received unsupported event: %s, valid event: e_start", event_in_msg_.data.c_str());
        pub_event_out_.publish(event_out_msg);
        return;
    }
    
    // we only proceed if the two pointcloud have been received
    if (!is_source_pcl_received_)
    {
        ROS_ERROR("Source pointcloud not received !");
        pub_event_out_.publish(event_out_msg);
        return;
    }
    
    // we only proceed if the two pointcloud have been received
    if (!is_target_pcl_received_)
    {
        ROS_ERROR("Target pointcloud not received !");
        pub_event_out_.publish(event_out_msg);
        return;
    }

    // reset received pointcloud flags
    is_source_pcl_received_ = false;
    is_target_pcl_received_ = false;
    
    // declare data type in which the node is going to expose the result of the comparison of the clouds
    std_msgs::Float64 cloud_evaluation_result;
    
    if (match_pointclouds_)
    {
        // compare clouds using iterative closest point from pcl
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance (max_correspondance_distance_);
        //icp.setMaximumIterations (50);
        icp.setInputSource(source_pointcloud_);
        icp.setInputTarget(target_pointcloud_);
        pcl::PointCloud<pcl::PointXYZ> matched_cloud;
        icp.align(matched_cloud);
        ROS_INFO_STREAM("ICP has converged : " << icp.hasConverged());
        ROS_INFO_STREAM("score: " << icp.getFitnessScore());
        ROS_INFO_STREAM("ICP final transformation : \n" << icp.getFinalTransformation());
        
        // display the transformation in rviz for visualization purposes as a pose
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        geometry_msgs::PoseStamped final_tf_as_pose;
        final_tf_as_pose.header.frame_id = target_pointcloud_frame_id_;
        final_tf_as_pose.header.stamp = ros::Time::now();
        final_tf_as_pose.pose.position.x = transformation(0,3);
        final_tf_as_pose.pose.position.y = transformation(1,3);
        final_tf_as_pose.pose.position.z = transformation(2,3);
        
        tf::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(transformation(0,0)), static_cast<double>(transformation(0,1)), static_cast<double>(transformation(0,2)), 
        static_cast<double>(transformation(1,0)), static_cast<double>(transformation(1,1)), static_cast<double>(transformation(1,2)), 
        static_cast<double>(transformation(2,0)), static_cast<double>(transformation(2,1)), static_cast<double>(transformation(2,2)));
        
        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        final_tf_as_pose.pose.orientation.x = tfqt.x();
        final_tf_as_pose.pose.orientation.y = tfqt.y();
        final_tf_as_pose.pose.orientation.z = tfqt.z();
        final_tf_as_pose.pose.orientation.w = tfqt.w();
        
        pub_match_orientation_.publish(final_tf_as_pose);
        
        // publish matched pointcloud for visualization purposes
        publishCloud(matched_cloud, pub_matched_cloud_, target_pointcloud_frame_id_);
        
        // fill message to be published
        cloud_evaluation_result.data = icp.getFitnessScore();
    }
    else
    {
        // compute fitness score without cloud matching
        cloud_evaluation_result.data = fitness_score_calc_.computeFitnessScore(source_pointcloud_, target_pointcloud_);
    }
    
    // publish the evaluation result
    pub_evaluation_.publish(cloud_evaluation_result);
        
    // feedback : reporting success
    ROS_INFO("Pointcloud succesfully evaluated !");
    event_out_msg.data = "e_success";
    pub_event_out_.publish(event_out_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iterative_closest_point_eval_node");

    ROS_INFO("Node is going to initialize ...");

    // create object of the node class (IterativeClosestPointEval)
    IterativeClosestPointEval iterative_closest_point_eval_node;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 30.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");
    
    // check if sim time is being used (i.e. comming from rosbag)
    bool sim_time;
    nh.param("/use_sim_time", sim_time, false);
    
    while (ros::ok())
    {
        // main loop function
        iterative_closest_point_eval_node.update();

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
