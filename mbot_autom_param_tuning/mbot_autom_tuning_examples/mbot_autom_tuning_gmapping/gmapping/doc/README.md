Iterative closest point pointcloud matching and fitness score
=============================================================

Matches 2 pointclouds to each other (see icp_animation.gif), a reference one and a evaluation one.
Prints the transformation needed for the best match.
Provides with the final error of the clouds after matching was performed,
as a mean squared calculation between all points of the source cloud and their
nearest neighbor in the target cloud.

The following information (below) was taken from PCL documentation, word by word. Specifically from the file:
pcl/registration/include/pcl/registration/icp.h
which can be found on: https://github.com/PointCloudLibrary/pcl

    \brief @b IterativeClosestPoint provides a base implementation of the Iterative Closest Point algorithm. 
    * The transformation is estimated based on Singular Value Decomposition (SVD).
    *
    * The algorithm has several termination criteria:
    *
    * <ol>
    * <li>Number of iterations has reached the maximum user imposed number of iterations (via \ref setMaximumIterations)</li>
    * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
    * <li>The sum of Euclidean squared errors is smaller than a user defined threshold (via \ref setEuclideanFitnessEpsilon)</li>
    * </ol>
    *
    *
    * Usage example:
    * \code
    * IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    * // Set the input source and target
    * icp.setInputCloud (cloud_source);
    * icp.setInputTarget (cloud_target);
    *
    * // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    * icp.setMaxCorrespondenceDistance (0.05);
    * // Set the maximum number of iterations (criterion 1)
    * icp.setMaximumIterations (50);
    * // Set the transformation epsilon (criterion 2)
    * icp.setTransformationEpsilon (1e-8);
    * // Set the euclidean distance difference epsilon (criterion 3)
    * icp.setEuclideanFitnessEpsilon (1);
    *
    * // Perform the alignment
    * icp.align (cloud_source_registered);
    *
    * // Obtain the transformation that aligned cloud_source to cloud_source_registered
    * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    * \endcode
    *
    * \author Radu B. Rusu, Michael Dixon
    * \ingroup registration

You can test this code by using the following terminal arrange:

roscore                                                                         rosrun rviz rviz
roslaunch mbot_automatic_param_tuning publish_map.launch
rosrun mbot_automatic_param_tuning occupancy_grid_to_pointcloud                 roslaunch mbot_automatic_param_tuning iterative_closest_point_eval.launch
roslaunch mbot_automatic_param_tuning publish_ground_truth_points.launch        rostopic echo /iterative_closest_point_eval/cloud_quality

triggers:
rostopic pub /occupancy_grid_to_pointcloud_node/event_in std_msgs/String "data: 'e_convert'"
rostopic pub /iterative_closest_point_eval/event_in std_msgs/String "data: 'e_compare'"

to save a pointcloud from a topic
=================================

rosrun pcl_ros pointcloud_to_pcd input:=/iterative_closest_point_eval/matched_cloud