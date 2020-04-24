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
ros::Publisher pub;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  // Read in the cloud data

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  //cloud_filtered_blob = cloud_plane;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
      //uint8_t r = 0, g = 255, b = 0;
      //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      //cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);    
    
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //pcl_conversions::toPCL(*cloud_msg, *cloud);
    //pcl::toPCLPointCloud2 (*cloud_cluster, *cloud_filtered_blob);
    //pcl::concatenatePointCloud (cloud_cluster,cloud_filtered_blob, cloud_filtered_blob);
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud_cluster);
    while (!viewer.wasStopped ())
    {
    }
    //pcl::compute3DCentroid(*cloud_cluster, const pcl::PointIndices& *indices, Eigen::Matrix<float,4,1>& centroid); 	
    
    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2 (*cloud_cluster, outcloud);
    pub.publish (outcloud);
 //*
    j++;
  }
  
} 

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clusters");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("clusters", 1);

  // Spin
  ros::spin ();
}
