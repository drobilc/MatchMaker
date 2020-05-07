#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
double leaf_size_x, leaf_size_y, leaf_size_z;

void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {

  pcl::PCLPointCloud2 cloud_filtered;

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish(output);

  // pub.publish(cloud_filtered);
}

int main(int argc, char** argv) {
  ros::init (argc, argv, "voxelgrid");
  ros::NodeHandle nh;
  // Get parameters from launch file
  nh.getParam("/voxelgrid/leaf_size_x", leaf_size_x);
  nh.getParam("/voxelgrid/leaf_size_y", leaf_size_y);
  nh.getParam("/voxelgrid/leaf_size_z", leaf_size_z);

  std::cerr << "Leaf size: x = " << leaf_size_x << ", y = " << leaf_size_y << ", z = " << leaf_size_z << std::endl;

  // Publisher and subscriber
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);
  // pub = nh.advertise<pcl::PCLPointCloud2>("output", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  ros::spin ();
}
