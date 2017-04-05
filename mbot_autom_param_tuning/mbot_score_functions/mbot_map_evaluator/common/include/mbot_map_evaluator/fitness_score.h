/*  
* Copyright [2016] <Instituto Superior Tecnico>
*  
* Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
* 
* Computes the error of 2 pointclouds by summing the squared mean error, using the nearest neighbor
* code mainly taken from http://docs.pointclouds.org/1.5.1/registration_8hpp_source.html#l00072 72-110
* see also: http://pointclouds.org/documentation/tutorials/interactive_icp.php
*   
*/

#ifndef MBOT_AUTOMATIC_PARAM_TUNING_FITNESS_SCORE_H
#define MBOT_AUTOMATIC_PARAM_TUNING_FITNESS_SCORE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;

class FitnessScore
{
    public:
        // constructor
        FitnessScore(double max_range);
        
        // allow to change the value of the maximum value after which to search for the nearest neighbor
        void setMaxRange(double max_range);

        // calculates mean square error of two pointclouds
        double computeFitnessScore(cloud_ptr& source_cloud, cloud_ptr& target_cloud);

    private:
        // if there is no nearest neighbor beyond this value, then error is not taken into account
        double max_range_;
};
#endif  // MBOT_AUTOMATIC_PARAM_TUNING_FITNESS_SCORE_H
