// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
// ros messages 
#include <geometry_msgs/PoseArray.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "stdio.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

// Namespace matches ROS package name
namespace pcl_tutorial {

  class My_pcl {
    public:
     My_pcl(ros::NodeHandle& n, ros::NodeHandle& pn);
     
     

    private:
     ros::Publisher pub;
     ros::Subscriber sub;
     void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_msg);
     // KD search tree object for use by PCL functions
     pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;
     ros::Publisher pub_normals_; 
     geometry_msgs::PoseArray normals_;




  };

}
