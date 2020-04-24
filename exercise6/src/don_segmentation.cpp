#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

//#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transformation_from_correspondences.h>
//#include <boost/thread/thread.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/transforms.h"

using namespace pcl;
using namespace std;
ros::Publisher pub;
void
cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  ///The smallest scale to use in the DoN filter.
  double scale1;

  ///The largest scale to use in the DoN filter.
  double scale2;

  ///The minimum DoN magnitude to threshold by
  double threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius;
  
  /*if (argc < 6)
  {
    cerr << "usage: " << argv[0] << " inputfile smallscale largescale threshold segradius" << endl;
    exit (EXIT_FAILURE);
  }*/

////--------------------variable values---------------------///:) happy iterating

  /// the file to read from.
  //string infile = argv[1];
  /// small scale
  //istringstream (argv[2]) >> scale1;
  scale1 = 0.05;
   
  /// large scale
  //istringstream (argv[3]) >> scale2;
  scale2 = 0.08;
  //istringstream (argv[4]) >> threshold;   // threshold for DoN magnitude
  threshold = 1.0;
  //istringstream (argv[5]) >> segradius;   // threshold for radius segmentation
  segradius = 0.1;


/////----------------------------------------------------------------------------------------


  // Load cloud in blob format
  //pcl::PCLPointCloud2 blob;
  //pcl::io::loadPCDFile (infile.c_str (), blob);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  //pcl::fromPCLPointCloud2 (blob, *cloud);
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud);
  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZRGB>::Ptr tree;
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

  // Build the condition for filtering
  pcl::ConditionOr<PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                               new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                             );
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

  pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
  segtree->setInputCloud (doncloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;

  ec.setClusterTolerance (segradius);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (segtree);
  ec.setInputCloud (doncloud);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;


    //Save cluster
    cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (cloud_cluster_don);
    //while (!viewer.wasStopped ())
    //{
    //}
    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2 (*cloud_cluster_don, outcloud);
    pub.publish (outcloud);
  }
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "don_segmenters");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("don_segments", 1);

  // Spin
  ros::spin ();
}
