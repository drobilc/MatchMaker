#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud_filtered);

  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0) return;

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_outliers);

  // Publish the found plane
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*cloud_outliers, outcloud);

  pub.publish (outcloud);
}

int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "find_plane");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("plane", 1);

  // Spin
  ros::spin ();
}
