#include <iostream>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//...
int
main ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  //... populate cloud
  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }
  return 0;
}
