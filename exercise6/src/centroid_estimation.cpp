#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <vector>
//#include <Matrix.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include "pcl/common/transformation_from_correspondences.h"
//#include <boost/thread/thread.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCellArray.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <vector>
//#include <Matrix.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>

#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/PointIndices.h>
#include <pcl/cloud_iterator.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
//#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacle_projection");
  ros::NodeHandle nh;

  pub = nh.advertise<pcl::PCLPointCloud2> ("cloud_demean", 1);
    // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");

  
  pcl::PCDReader reader;
  reader.read ("cloud_cluster_2.pcd", *cluster);

 
 Eigen::Vector4f centroid;
 pcl::compute3DCentroid (*cluster, centroid);
 cout << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";
 /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
 pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
 
 pcl::console::print_highlight ("centroid\n", centroid);
 std::stringstream ss;
 ss << "cloud_centroid_" << j << ".pcd";
 writer.write<pcl::PointXYZ> (ss.str (), *cloud_xyz_demean, false);
 pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
 viewer.showCloud (cloud_xyz_demean);
 pcl::PCLPointCloud2 outcloud_tmp_visual;
 pcl::toPCLPointCloud2 (cloud_xyz_demean, outcloud_tmp_visual);
 pub.publish (outcloud_tmp_visual); */

  //ros::spin ();
  return (0);
}

