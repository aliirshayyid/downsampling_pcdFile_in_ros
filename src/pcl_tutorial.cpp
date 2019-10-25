// Header file for the class
#include "pcl_tutorial.hpp"



// Namespace matches ROS package name
namespace pcl_tutorial
{  
  // Constructor with global and private node handle arguments (class name :: constructor name)
  My_pcl::My_pcl(ros::NodeHandle& n, ros::NodeHandle& pn):
   kd_tree_(new pcl::search::KdTree<pcl::PointXYZ>)
  {
  // Create a ROS subscriber for the input point cloud
  sub = n.subscribe("/cloud_pcd", 1, &My_pcl::cloud_cb,this);
  pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_normals_ = n.advertise<geometry_msgs::PoseArray>("normals", 1);

  }

void My_pcl::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
  // Create a container for the data.
  // sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);


  // Do data processing here...
  // output = *input;
  pcl::fromROSMsg(*input_msg, *input_cloud);
  filterRawCloud(input_cloud, input_cloud);
  // Compute normal vectors for the incoming point cloud
  pcl::PointCloud <pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  kd_tree_->setInputCloud(input_cloud);  
  normal_estimator.setSearchMethod(kd_tree_);
  normal_estimator.setInputCloud(input_cloud);
  normal_estimator.setKSearch(5);
  normal_estimator.compute(*cloud_normals);

    // TODO: Filter out near-vertical normals
    pcl::PointIndices non_vertical_normals;
    for (int i = 0; i < cloud_normals->points.size(); i++) {
      non_vertical_normals.indices.push_back(i);
    }


// Copy non-vertical normals into a separate cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*input_cloud, non_vertical_normals, *no_ground_cloud);

    // Populate PoseArray message to visualize normals
    normals_.header = pcl_conversions::fromPCL(input_cloud->header);
    normals_.poses.clear();
    for (int i = 0; i < non_vertical_normals.indices.size(); i++) {
      geometry_msgs::Pose p;
      p.position.x = input_cloud->points[non_vertical_normals.indices[i]].x;
      p.position.y = input_cloud->points[non_vertical_normals.indices[i]].y;
      p.position.z = input_cloud->points[non_vertical_normals.indices[i]].z;

      double nx = cloud_normals->points[non_vertical_normals.indices[i]].normal_x;
      double ny = cloud_normals->points[non_vertical_normals.indices[i]].normal_y;
      double nz = cloud_normals->points[non_vertical_normals.indices[i]].normal_z;

      // Start converting unit vector to quaternion
      tf2::Quaternion q;
      tf2::Matrix3x3 rot_mat;
      if (fabs(nz) < 0.01) {
        rot_mat[0] = tf2::Vector3(nx, ny, nz);
        rot_mat[1] = tf2::Vector3(0, -nz, ny);
        rot_mat[1].normalize();
        rot_mat[2] = rot_mat[0].cross(rot_mat[1]);
      } else {
        rot_mat[0] = tf2::Vector3(nx, ny, nz);
        rot_mat[1] = tf2::Vector3(-ny, nx, 0);
        rot_mat[1].normalize();
        rot_mat[2] = rot_mat[0].cross(rot_mat[1]);
      }
      rot_mat.getRotation(q);

      // Load quaternion into pose message
      tf2::convert(q.inverse(), p.orientation);
      normals_.poses.push_back(p);
    }
    // Publish normal vectors
    pub_normals_.publish(normals_);



  // Run through a voxel grid filter to downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> downsample;
    downsample.setInputCloud(input_cloud);
    downsample.setLeafSize(0.1, 0.1, 0.1);
    downsample.filter(*output);
  
  sensor_msgs::PointCloud2 filtered_output;
  pcl::toROSMsg(*output,filtered_output);
  filtered_output.header.frame_id = "filtered_cloud";
  // Publish the data.
  pub.publish (filtered_output);
}

  


  

  

}
