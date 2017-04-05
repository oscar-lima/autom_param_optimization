/*  
* Copyright [2016] <Instituto Superior Tecnico>
*  
* Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
* 
* Computes the error of 2 pointclouds by summing the squared mean error, using the nearest neighbor
* code mainly taken from : pcl/registration/include/pcl/registration/impl/registration.hpp
* https://github.com/PointCloudLibrary/pcl
*   
*/

#include <mbot_map_evaluator/fitness_score.h>

FitnessScore::FitnessScore(double max_range) : max_range_(0.0) 
{
    max_range_ = max_range;
}

void FitnessScore::setMaxRange(double max_range)
{
    max_range_ = max_range;
}

double FitnessScore::computeFitnessScore(cloud_ptr& source_cloud, cloud_ptr& target_cloud)
{
    double fitness_score = 0.0;
    
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (target_cloud);
    
    // For each point in the source dataset
    int nr = 0;
    for (int i = 0; i < source_cloud->points.size (); i++)
    {   
        // Find its nearest neighbor in the target
        pcl::PointXYZ searchPoint;
        searchPoint.x = source_cloud->points[i].x;
        searchPoint.y = source_cloud->points[i].y;
        searchPoint.z = source_cloud->points[i].z;
        kdtree.nearestKSearch (searchPoint, 1, nn_indices, nn_dists);
        
        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range_)
        {   
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0)
        return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max ());
}
