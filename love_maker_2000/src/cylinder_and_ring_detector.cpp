#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <object_detection_msgs/ObjectDetection.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include "color_classification/ColorClassification.h"
#include "love_maker_2000/ApproachingPointCalculator.h"

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pub_torus_cloud;
ros::Publisher pub_cylinder;
ros::Publisher pub_torus;
ros::Publisher pub_testing;

ros::ServiceClient color_classifier;
ros::ServiceClient cylinder_approaching_point_calculator;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZ PointT;

ros::Time time_rec;

// Parameters for cylinder segmentation
double cylinder_normal_distance_weight;
double cylinder_max_iterations;
double cylinder_distance_threshold;
double cylinder_radius_max;
double cylinder_radius_min;
int cylinder_points_threshold;

// Parameters for planar segmentation
double plane_points_threshold;

/**
 * Logs the point cloud to /point_cloud/log topic.
 * 
 * Only use once in code, because all point clouds will be sent to the same topic.
 */
void log_pointcloud(pcl::PointCloud<PointT>::Ptr &cloud)
{
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2(*cloud, outcloud);
  pub_testing.publish(outcloud);
}

/**
 * Finds rings in a more sophisticated way, does not work yet. Use find_rings.
 */
void find_rings2(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  // Filter the point cloud at certain heights
  pcl::PassThrough<PointT> pass;
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5, -0.3);
  pass.filter(*cloud_filtered);
  std::cerr << "PointCloud for rings has: " << cloud_filtered->points.size() << " data points." << std::endl;

  log_pointcloud(cloud_filtered);

  // Select four random points, if they are on the same plane we can proceed
}

/**
 * Find rings in the given pint cloud.
 * 
 * First it filters out everything that is below or above the rings, what is left must be a ring
 * beacuse there is nothing else at that height.
 * 
 * @param cloud Input point cloud.
 */
void find_rings(pcl::PointCloud<PointT>::Ptr &cloud)
{
  pcl::PassThrough<PointT> pass;

  // Build a passthrough filter to leave only points at height, where rings are
  std::cerr << "\nStart filtering cloud for toruses" << std::endl;
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5, -0.3);
  pass.filter(*cloud_filtered);
  std::cerr << "PointCloud for toruses has: " << cloud_filtered->points.size() << " data points." << std::endl;

  // Publish the point cloud
  pcl::PCLPointCloud2 outcloud_torus;
  pcl::toPCLPointCloud2(*cloud_filtered, outcloud_torus);
  pub_torus_cloud.publish(outcloud_torus);

  float x, y, z;

  if (cloud_filtered->points.size() > 0)
  {
    x = cloud_filtered->points[1].x;
    y = cloud_filtered->points[1].y;
    z = cloud_filtered->points[1].z;
  }
  else
  {
    return;
  }

  float alpha = 0.25;
  for (int i = 0; i < cloud_filtered->points.size(); i++)
  {
    x = alpha * cloud_filtered->points[i].x + (1.0 - alpha) * x;
    y = alpha * cloud_filtered->points[i].y + (1.0 - alpha) * y;
    z = alpha * cloud_filtered->points[i].z + (1.0 - alpha) * z;
  }

  // Transform coordinates to map coordinate frame
  geometry_msgs::PointStamped point_camera;
  geometry_msgs::PointStamped point_map;
  visualization_msgs::Marker marker;
  geometry_msgs::PoseStamped pose_cylinder;
  geometry_msgs::TransformStamped tss;

  point_camera.header.frame_id = "camera_rgb_optical_frame";
  point_camera.header.stamp = ros::Time::now();

  point_map.header.frame_id = "map";
  point_map.header.stamp = ros::Time::now();

  point_camera.point.x = x;
  point_camera.point.y = y;
  point_camera.point.z = z;

  try
  {
    tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", ros::Time::now());
    //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Transform warning: %s\n", ex.what());
  }

  //std::cerr << tss ;

  tf2::doTransform(point_camera, point_map, tss);

  // Publish location of torus
  geometry_msgs::PoseStamped pose_torus;
  pose_torus.pose.position.x = point_map.point.x;
  pose_torus.pose.position.y = point_map.point.y;
  pose_torus.pose.position.z = point_map.point.z;
  pose_torus.header.stamp = ros::Time::now();
  pub_torus.publish(pose_torus);
}

/**
 * Remove all planes from the point cloud.
 * 
 * In find_torus_and_cylinder you can specifiy a threshold. All detected planes
 * that have more points than the threshold will be removed.
 * 
 * @param cloud_filtered Input point cloud.
 * @param cloud_normals Input point cloud normals.
 * @param cloud_filtered2 Output point cloud, all planes that have more points than the threshold have been removed from it.
 * @param cloud_normals2 Output normals of the cloud_filtered2 point cloud.
 */
void remove_all_planes(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<PointT>::Ptr cloud_filtered2, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2)
{
  // Objects needed
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

  // Datasets
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // Contains cloud of points that form a plane
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());

  do
  {
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // Write the planar inliers to disk
    extract.filter(*cloud_plane);
    // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    if (!cloud_plane->points.empty())
    {
      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered2);
      extract_normals.setNegative(true);
      extract_normals.setInputCloud(cloud_normals);
      extract_normals.setIndices(inliers_plane);
      extract_normals.filter(*cloud_normals2);
      cloud_filtered = cloud_filtered2;
      cloud_normals = cloud_normals2;
    }

  } while (cloud_plane->points.size() >= plane_points_threshold);
}

/**
 * Finds cylinders in the point cloud and publishes the detections. Removes found cylinders from the point cloud.
 * 
 * @param cloud Input point cloud.
 * @param cloud_normals Input point cloud normals.
 * @param cloud_filtered Output point cloud without cylinder points.
 * @param cloud_normals_filtered Output cloud without cylinder point normals.
 */
void find_cylinders(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_filtered_normals)
{
  ros::Time time_test;

  // Objects we need
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  Eigen::Vector4f centroid;

  // Datasets
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  std::cerr << "\n-- Cylinder Segmentation" << std::endl;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(cylinder_normal_distance_weight);
  seg.setMaxIterations(cylinder_max_iterations);
  seg.setDistanceThreshold(cylinder_distance_threshold);
  seg.setRadiusLimits(cylinder_radius_min, cylinder_radius_max);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.size() >= cylinder_points_threshold)
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;

    pcl::compute3DCentroid(*cloud_cylinder, centroid);
    std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

    //Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;
    visualization_msgs::Marker marker;
    object_detection_msgs::ObjectDetection cylinder_detection_message;
    geometry_msgs::TransformStamped tss;

    color_classification::ColorClassification color_classificator;
    love_maker_2000::ApproachingPointCalculator approaching_point_calculator;

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    std::cerr << "Centroids: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << std::endl;

    try
    {
      time_test = ros::Time::now();

      std::cerr << time_rec << std::endl;
      std::cerr << time_test << std::endl;
      tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
      //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform warning: %s\n", ex.what());
    }

    //std::cerr << tss ;

    tf2::doTransform(point_camera, point_map, tss);
    std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;
    std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

    // Calculate approaching point
    geometry_msgs::Pose detection_pose;
    detection_pose.position.x = point_map.point.x;
    detection_pose.position.y = point_map.point.y;
    detection_pose.position.z = point_map.point.z;

    approaching_point_calculator.request.detection = detection_pose;

    geometry_msgs::Pose approaching_point;

    if (cylinder_approaching_point_calculator.call(approaching_point_calculator))
    {
      approaching_point = approaching_point_calculator.response.approaching_point;
    }
    else
    {
      ROS_ERROR("Failed to call service for calculating approaching point");
    }

    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
    puby.publish(outcloud_cylinder);

    // Publish the message only if point is not nan
    if (point_map.point.x == point_map.point.x && point_map.point.z == point_map.point.z && point_map.point.y == point_map.point.y)
    {
      // Correctly configure the header
      cylinder_detection_message.header.stamp = ros::Time::now();
      cylinder_detection_message.header.frame_id = "map";
      // Set detection and approaching point
      cylinder_detection_message.approaching_point_pose = approaching_point;
      cylinder_detection_message.object_pose.position.x = point_map.point.x;
      cylinder_detection_message.object_pose.position.y = point_map.point.y;
      cylinder_detection_message.object_pose.position.z = point_map.point.z;
      // Set type of object message
      cylinder_detection_message.type = "cylinder";
      pub_cylinder.publish(cylinder_detection_message);
    }

    // Remove cylinder inliers and extract the rest
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_cylinder);
    extract_normals.filter(*cloud_filtered_normals);
  }
  else
  {
    std::cerr << "Can't find the cylindrical component." << std::endl;
    *cloud_filtered = *cloud;
    *cloud_filtered_normals = *cloud_normals;
  }
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
  time_rec = ros::Time::now();

  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::PCDWriter writer;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered3(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3(new pcl::PointCloud<pcl::Normal>);

  // Read in the cloud data
  pcl::fromPCLPointCloud2(*cloud_blob, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 2);
  pass.filter(*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Find rings
  // find_rings(cloud_filtered);

  // Remove all planes
  remove_all_planes(cloud_filtered, cloud_normals, cloud_filtered2, cloud_normals2);

  // Find cylinders and remove them from the point cloud
  find_cylinders(cloud_filtered2, cloud_normals2, cloud_filtered3, cloud_normals3);

  // log_pointcloud(cloud_filtered3);

  find_rings2(cloud_filtered3, cloud_normals3);
}

void get_parameters(ros::NodeHandle nh)
{
  // Get parameters for cylinder segmentation from the launch file
  nh.getParam("/cylinder_and_ring_detector/normal_distance_weight", cylinder_normal_distance_weight);
  nh.getParam("/cylinder_and_ring_detector/max_iterations", cylinder_max_iterations);
  nh.getParam("/cylinder_and_ring_detector/distance_threshold", cylinder_distance_threshold);
  nh.getParam("/cylinder_and_ring_detector/radius_max", cylinder_radius_max);
  nh.getParam("/cylinder_and_ring_detector/radius_min", cylinder_radius_min);
  nh.getParam("/cylinder_and_ring_detector/cylinder_points_threshold", cylinder_points_threshold);
  // Get parameters for plane segmentation from the launch file
  nh.getParam("/cylinder_and_ring_detector/plane_points_threshold", plane_points_threshold);
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // Get needed parameters
  get_parameters(nh);

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2>("point_cloud/planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2>("point_cloud/cylinder", 1);
  pub_torus_cloud = nh.advertise<pcl::PCLPointCloud2>("point_cloud/torus", 1);
  pub_testing = nh.advertise<pcl::PCLPointCloud2>("point_cloud/log", 1);

  pub_cylinder = nh.advertise<object_detection_msgs::ObjectDetection>("cylinder_detections_raw", 1);
  pub_torus = nh.advertise<object_detection_msgs::ObjectDetection>("torus_detections_raw", 1);

  color_classifier = nh.serviceClient<color_classification::ColorClassification>("color_classifier_server");
  cylinder_approaching_point_calculator = nh.serviceClient<love_maker_2000::ApproachingPointCalculator>("cylinder_approaching_point_calculator");

  // Spin
  ros::spin();
}
