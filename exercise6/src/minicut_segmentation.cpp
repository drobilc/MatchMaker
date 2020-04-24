#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>


ros::Publisher pub;


void
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::fromPCLPointCloud2 (blob, *cloud);
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);


  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  seg.setInputCloud (cloud);
  seg.setIndices (indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ point;
/////------------------specify the approximate center of the object---can this come from the joint location-----happy iterating :)
  point.x = 68.97;
  point.y = -18.55;
  point.z = 0.57;

  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);

  seg.setSigma (0.25);
  seg.setRadius (3.0433856);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);
//////-------------------***********-------------------------------
  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);

  std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*colored_cloud, outcloud);
  pub.publish (outcloud);
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "mini_segmenters");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("mini_segments", 1);

  // Spin
  ros::spin ();
}
