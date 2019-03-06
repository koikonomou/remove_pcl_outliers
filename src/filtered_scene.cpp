#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;
float x, y, z;

void callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 cloud2 ; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, cloud2);
  pcl::fromPCLPointCloud2(cloud2, *cloud);


  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 point_cloud2;

  pcl::toPCLPointCloud2(*cloud_filtered, point_cloud2);
  pcl_conversions::fromPCL(point_cloud2, output);

  // Publish the data
  pub.publish (output);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "filtered_scene");
  ros::NodeHandle nh;


  std::string in_topic, out_topic;
  nh.param("scene_filtered/input_topic", in_topic, std::string("zed/point_cloud/cloud_registered"));
  nh.param("scene_filtered/output_topic", out_topic, std::string("scene_filtered/pointcloud"));

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (in_topic, 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

  // Spin
  ros::spin ();
}