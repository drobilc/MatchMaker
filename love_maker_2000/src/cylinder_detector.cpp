#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
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
#include <pcl/filters/random_sample.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include "color_classification/ColorClassification.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace Eigen;

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pub_torus_cloud;
ros::Publisher pub_cylinder;
ros::Publisher pub_torus;
ros::Publisher pub_testing;
ros::Publisher pub_markers;
visualization_msgs::MarkerArray markers;

ros::ServiceClient color_classifier;

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

// Parameters for ring segmentation
int torus_ransac_max_iterations;

// The last received image from camera source
cv_bridge::CvImagePtr lastImageMessage;

float absf(float x)
{
  return x >= 0 ? x : -x;
}

void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    // Convert received image to opencv image message
    lastImageMessage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

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
 * Works only for four points. Returns true if points are coplanar.
 */
bool are_points_coplanar(std::vector<Vector3f> points)
{
  Vector3f pA, pB, pC, pD;
  pA = points[0];
  pB = points[1];
  pC = points[2];
  pD = points[3];

  Vector3f vAB(pB[0] - pA[0], pB[1] - pA[1], pB[2] - pA[2]);
  Vector3f vAC(pC[0] - pA[0], pC[1] - pA[1], pC[2] - pA[2]);
  Vector3f vAD(pD[0] - pA[0], pD[1] - pA[1], pD[2] - pA[2]);

  // If result of following equation is 0, points are coplanar
  float result = vAD.dot(vAB.cross(vAC));
  std::cerr << "Result of AD * (AB x AC): " << result << std::endl;

  return abs(result) < 0.1 ? true : false;
}

void filter_point_cloud_height(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<PointT>::Ptr &cloud_filtered, float min_height, float max_height)
{
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(min_height, max_height);
  pass.filter(*cloud_filtered);
}

/**
 * Calculates point pa on one line and pb on the second line, that are closest to each other.
 */
bool two_lines_intersection(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4, Vector3f &pa, Vector3f &pb, float &mua, float &mub)
{
  std::cerr << "Calculating intersection of two lines" << std::endl;
  float eps = 0.0001;
  Vector3f p13, p43, p21;
  float d1343, d4321, d1321, d4343, d2121;
  float numer, denom;

  std::cerr << "p1: x = " << p1[0] << ", y = " << p1[1] << ", z = " << p1[2] << std::endl;
  std::cerr << "p2: x = " << p2[0] << ", y = " << p2[1] << ", z = " << p2[2] << std::endl;
  std::cerr << "p3: x = " << p3[0] << ", y = " << p3[1] << ", z = " << p3[2] << std::endl;
  std::cerr << "p4: x = " << p4[0] << ", y = " << p4[1] << ", z = " << p4[2] << std::endl;

  p13[0] = p1[0] - p3[0];
  p13[1] = p1[1] - p3[1];
  p13[2] = p1[2] - p3[2];
  p43[0] = p4[0] - p3[0];
  p43[1] = p4[1] - p3[1];
  p43[2] = p4[2] - p3[2];
  if (absf(p43[0]) < eps && absf(p43[1]) < eps && absf(p43[2]) < eps)
  {
    return false;
  }
  p21[0] = p2[0] - p1[0];
  p21[1] = p2[1] - p1[1];
  p21[2] = p2[2] - p1[2];
  if (absf(p21[0]) < eps && absf(p21[1]) < eps && absf(p21[2]) < eps)
  {
    return false;
  }

  d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
  d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
  d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
  d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
  d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

  denom = d2121 * d4343 - d4321 * d4321;
  if (absf(denom) < eps)
  {
    return false;
  }
  numer = d1343 * d4321 - d1321 * d4343;

  mua = numer / denom;
  mub = (d1343 + d4321 * mua) / d4343;

  pa[0] = p1[0] + mua * p21[0];
  pa[1] = p1[1] + mua * p21[1];
  pa[2] = p1[2] + mua * p21[2];
  pb[0] = p3[0] + mub * p43[0];
  pb[1] = p3[1] + mub * p43[1];
  pb[2] = p3[2] + mub * p43[2];

  return true;
}

/**
 * Calculates circle center. Based on this: http://www.mathopenref.com/constcirclecenter.html
 */
bool calculate_circle_center(std::vector<Vector3f> points, PointT &center)
{
  std::cerr << "Started calculating center of the circle" << std::endl;
  Vector3f pA, pB, pC, pD;
  Vector3f p1, p2, p3, p4, p_result_1, p_result_2;
  Vector3f intersection;
  float mua, mub;

  pA = points[0];
  pB = points[1];
  pC = points[2];
  pD = points[3];

  Vector3f plane_normal = (pB - pA).cross(pC - pA);
  plane_normal.normalize();
  Vector3f pAB_half = (pB - pA) / 2;
  Vector3f pCD_half = (pD - pC) / 2;
  p1 = pA + pAB_half;
  p3 = pC + pCD_half;

  Vector3f orthogonal_to_pAB = (pB - pA).cross(plane_normal);
  Vector3f orthogonal_to_pCD = (pD - pC).cross(plane_normal);
  orthogonal_to_pAB.normalize();
  orthogonal_to_pCD.normalize();
  p2 = p1 + orthogonal_to_pAB;
  p4 = p3 + orthogonal_to_pCD;

  // If the two lines do not intersect return
  if (!two_lines_intersection(p1, p2, p3, p4, p_result_1, p_result_2, mua, mub))
    return false;
  intersection = (p_result_1 - p_result_2) / 2 + p_result_2;

  std::cerr << "Intersection: x = " << intersection[0] << ", y = " << intersection[1] << ", z = " << intersection[2] << std::endl;

  center.x = intersection[0];
  center.y = intersection[1];
  center.z = intersection[2];

  return true;
}

void publish_marker(PointT point, float r_col, float g_col)
{
  // Convert to global coordinates and publishgeometry_msgs::PointStamped point_camera;
  geometry_msgs::PointStamped point_camera;
  geometry_msgs::PointStamped point_map;
  visualization_msgs::Marker marker;
  geometry_msgs::TransformStamped tss;

  point_camera.header.frame_id = "camera_rgb_optical_frame";
  point_camera.header.stamp = time_rec;

  point_map.header.frame_id = "map";
  point_map.header.stamp = time_rec;

  point_camera.point.x = point.x;
  point_camera.point.y = point.y;
  point_camera.point.z = point.z;

  try
  {
    tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
    //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Transform warning: %s\n", ex.what());
  }

  //std::cerr << tss ;

  tf2::doTransform(point_camera, point_map, tss);

  marker.header.frame_id = "map";
  marker.header.stamp = time_rec;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = markers.markers.size();
  marker.pose.position.x = point_map.point.x;
  marker.pose.position.y = point_map.point.y;
  marker.pose.position.z = point_map.point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = r_col;
  marker.color.g = g_col;
  marker.color.b = 0.0;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration();

  markers.markers.push_back(marker);
  visualization_msgs::MarkerArray marray;
  for (int i = 0; i < markers.markers.size(); i++)
  {
    marray.markers.push_back(markers.markers[i]);
  }
  pub_markers.publish(marray);
}

void publish_detection(PointT point)
{
  // Convert to global coordinates and publishgeometry_msgs::PointStamped point_camera;
  geometry_msgs::PointStamped point_camera;
  geometry_msgs::PointStamped point_map;
  visualization_msgs::Marker marker;
  geometry_msgs::TransformStamped tss;

  point_camera.header.frame_id = "camera_rgb_optical_frame";
  point_camera.header.stamp = time_rec;

  point_map.header.frame_id = "map";
  point_map.header.stamp = time_rec;

  point_camera.point.x = point.x;
  point_camera.point.y = point.y;
  point_camera.point.z = point.z;

  try
  {
    tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
    //tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Transform warning: %s\n", ex.what());
  }

  //std::cerr << tss ;

  tf2::doTransform(point_camera, point_map, tss);

  // Publish location of torus
  object_detection_msgs::ObjectDetection detection_torus;
  detection_torus.object_pose.position.x = point_map.point.x;
  detection_torus.object_pose.position.y = point_map.point.y;
  detection_torus.object_pose.position.z = point_map.point.z;
  detection_torus.approaching_point_pose.position.x = point_map.point.x;
  detection_torus.approaching_point_pose.position.y = point_map.point.y;
  detection_torus.approaching_point_pose.position.z = point_map.point.z;
  detection_torus.header.stamp = time_rec;
  detection_torus.header.frame_id = "map";
  detection_torus.type = "ring";
  pub_torus.publish(detection_torus);
}

/**
 * Finds rings in a more sophisticated way, does not work yet. Use find_rings.
 */
void find_rings2(pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT> four_points_cloud;
  std::vector<Vector3f> points;
  PointT best_center;
  float min_mse = std::numeric_limits<float>::max();
  float min_range = std::numeric_limits<float>::max();

  // Filter the point cloud at certain heights
  std::cerr << "\n-- Rings" << std::endl;
  filter_point_cloud_height(cloud, cloud_filtered, -0.5, -0.3);
  std::cerr << "PointCloud for rings has: " << cloud_filtered->points.size() << " data points." << std::endl;
  log_pointcloud(cloud_filtered);

  if (cloud_filtered->points.size() < 4)
    return;

  for (int iter = 0; iter < torus_ransac_max_iterations; iter++)
  {
    // Sample four random points
    pcl::RandomSample<PointT> sample;
    pcl::PointCloud<PointT> four_points_cloud2;
    sample.setInputCloud(cloud_filtered);
    sample.setSample(4);
    sample.setSeed(rand());
    sample.filter(four_points_cloud2);

    // Convert them into vectors
    for (int i = 0; i < four_points_cloud2.points.size(); i++)
      points.push_back(Vector3f(four_points_cloud2.points[i].x, four_points_cloud2[i].y, four_points_cloud2.points[i].z));

    // If points are not coplanar, continue with next loop iteration and next four points
    if (!are_points_coplanar(points))
      continue;

    PointT center;
    if (!calculate_circle_center(points, center))
      continue;

    // Calculate radius
    VectorXf radiuses(cloud_filtered->points.size());
    float radius = 0;
    float mse = 0;
    float range = 0;
    for (int i = 0; i < cloud_filtered->points.size(); i++)
    {
      radiuses(i, 1) = sqrtf(powf(cloud_filtered->points[i].x, 2.0) + powf(cloud_filtered->points[i].y, 2.0) + powf(cloud_filtered->points[i].z, 2.0));
    }
    radius = radiuses.sum() / radiuses.size();
    range = radiuses.maxCoeff() - radiuses.minCoeff();

    // Calculate MSE
    for (int i = 0; i < radiuses.size(); i++)
    {
      radiuses[i] -= radius;
    }
    radiuses = radiuses * radiuses;
    mse = radiuses.sum() / radiuses.size();

    if (range < min_range)
    {
      min_mse = mse;
      best_center = center;
      radius = radius;
      four_points_cloud = four_points_cloud2;
      min_range = range;
    }
    points.clear();
  }

  // TEST: publish four points
  for (int i = 0; i < four_points_cloud.points.size(); i++)
  {
    publish_marker(four_points_cloud.points[i], 1.0, 0.0);
  }
  std::cerr << "Four points size: " << four_points_cloud.points.size() << std::endl;

  publish_marker(best_center, 0.0, 1.0);
  std::cin.get();
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

cv::Point pointToPixel(Eigen::Vector4f point) {
  // This data is available from the projection matrix in camera info topic
  // rostopic echo /camera/rgb/camera_info.
  float fx = 554.254691191187;
  float fy = 554.254691191187;
  float cx = 320.5;
  float cy = 240.5;

  // Use pinhole camera model to compute (x, y) coordinates of the centroid on image
  float y = (point[1] / point[2]) * fy + cy;
  float x = (point[0] / point[2]) * fx + cx;

  return cv::Point((int) x, (int) y);
}

std_msgs::ColorRGBA averageColorAround(cv::Mat image, cv::Point center, int regionSize) {
  // Compute average color of a window of size regionSize around the
  // cylinder centroid on image
  int numberOfSamples = (regionSize * 2 + 1) * (regionSize * 2 + 1);

  int red = 0;
  int green = 0;
  int blue = 0;
  for (int i = -regionSize; i <= regionSize; i++) {
    for (int j = -regionSize; j <= regionSize; j++) {
      cv::Point point = cv::Point(center.x + j, center.y + i);
      cv::Vec3b color = image.at<cv::Vec3b>(point);
      red += color[0];
      green += color[1];
      blue += color[2];
    }
  }

  std_msgs::ColorRGBA color;
  color.r = red / numberOfSamples;
  color.g = green / numberOfSamples;
  color.b = blue / numberOfSamples;
  color.a = 255;
  return color;
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

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    std::cerr << "Centroids: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << std::endl;

    // Get robot coordinates in map frame
    geometry_msgs::PointStamped pointRobot, pointRobotMap;
    pointRobot.header.frame_id = "camera_rgb_optical_frame";
    pointRobot.header.stamp = point_camera.header.stamp;
    pointRobotMap.header.frame_id = "map";
    pointRobotMap.header.stamp = pointRobot.header.stamp;

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

    tf2::doTransform(point_camera, point_map, tss);
    tf2::doTransform(pointRobot, pointRobotMap, tss);

    // Now that we have computed cylinder position and robot position in map
    // coordinate frame, we can compute the approaching point for cylinder
    // To compute approaching point, compute vector v pointing from cylinder to
    // robot. Then compute point that is 0.5m far from the cylinder center. This
    // is the approaching point. To get the robot orientation, flip the vector.
    double dx = pointRobotMap.point.x - point_map.point.x;
    double dy = pointRobotMap.point.y - point_map.point.y;
    double dz = pointRobotMap.point.z - point_map.point.z;
    double norm = sqrt(dx * dx + dy * dy + dz * dz);
    dx /= norm; dy /= norm; dz /= norm;

    // Now that vector components have been initialized, we can multiply them by
    // 0.5 to get a point 0.5m away from the cylinder centroid. The approaching
    // point is then at point_map + v * 0.5
    double approachingPointX = point_map.point.x + dx * 0.5;
    double approachingPointY = point_map.point.y + dy * 0.5;
    double approachingPointZ = point_map.point.z + dz * 0.5;

    // Get the orientation of the approaching point
    tf2::Quaternion approachingPointOrientation;
    double angle = atan2(-dy, -dx);
    approachingPointOrientation.setRPY(0, 0, angle);

    geometry_msgs::Quaternion approachingPointOrientationMessage;
    approachingPointOrientationMessage = tf2::toMsg(approachingPointOrientation);

    std::cerr << "VECTOR: " << dx << ", " << dy << ", " << dz << std::endl;

    std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;
    std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
    puby.publish(outcloud_cylinder);

    // If the last image message is available, get color of the centroid from the received image
    color_classification::ColorClassification colorClassificator;
    std_msgs::ColorRGBA color;
    std_msgs::String classifiedColor;

    if (lastImageMessage) {
      cv::Point centroidPoint = pointToPixel(centroid);
      color = averageColorAround(lastImageMessage->image, centroidPoint, 5);

      // After average color has been calculated, classify it
      colorClassificator.request.color = color;

      // Classify the selected color using color_classification service
      if (color_classifier.call(colorClassificator)) {
        std::cerr << "CLASSIFIED COLOR: " << colorClassificator.response.classified_color << std::endl;
        classifiedColor.data = colorClassificator.response.classified_color;
        // Color has beem successfully classified
      } else {
        ROS_ERROR("Failed to call color_classifier service");
        return;
      }

    }

    // Publish the message only if point is not nan
    if (point_map.point.x == point_map.point.x && point_map.point.z == point_map.point.z && point_map.point.y == point_map.point.y)
    {
      // Correctly configure the header
      cylinder_detection_message.header.stamp = ros::Time::now();
      cylinder_detection_message.header.frame_id = "map";
      // Set detection and approaching point
      cylinder_detection_message.approaching_point_pose.position.x = approachingPointX;
      cylinder_detection_message.approaching_point_pose.position.y = approachingPointY;
      cylinder_detection_message.approaching_point_pose.position.z = approachingPointZ;
      cylinder_detection_message.approaching_point_pose.orientation = approachingPointOrientationMessage;
      cylinder_detection_message.object_pose.position.x = point_map.point.x;
      cylinder_detection_message.object_pose.position.y = point_map.point.y;
      cylinder_detection_message.object_pose.position.z = point_map.point.z;
      // Set color classification to message
      cylinder_detection_message.color = color;
      cylinder_detection_message.classified_color = classifiedColor.data;
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
  std::cerr << "\nEntered cloud callback" << std::endl;
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
  if (cloud->points.size() == 0) return;

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
  // find_rings2(cloud_filtered, cloud_normals3); // Unusable

  // Remove all planes
  remove_all_planes(cloud_filtered, cloud_normals, cloud_filtered2, cloud_normals2);

  // Find cylinders and remove them from the point cloud
  find_cylinders(cloud_filtered2, cloud_normals2, cloud_filtered3, cloud_normals3);

  // log_pointcloud(cloud_filtered3);

  // find_rings2(cloud_filtered3, cloud_normals3);
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

  nh.getParam("/cylinder_and_ring_detector/torus_ransac_max_iterations", torus_ransac_max_iterations);
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

  // Also create a ROS subscriber for camera image
  image_transport::ImageTransport imageTransport(nh);
  image_transport::Subscriber cameraSubscriber = imageTransport.subscribe("/camera/rgb/image_raw", 1, cameraCallback);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2>("point_cloud/planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2>("point_cloud/cylinder", 1);
  pub_torus_cloud = nh.advertise<pcl::PCLPointCloud2>("point_cloud/torus", 1);
  pub_testing = nh.advertise<pcl::PCLPointCloud2>("point_cloud/log", 1);

  pub_cylinder = nh.advertise<object_detection_msgs::ObjectDetection>("cylinder_detections_raw", 1);
  pub_torus = nh.advertise<object_detection_msgs::ObjectDetection>("torus_detections_raw", 1);
  pub_markers = nh.advertise<visualization_msgs::MarkerArray>("torus_detections_marker_raw", 100);

  color_classifier = nh.serviceClient<color_classification::ColorClassification>("color_classifier");

  // Spin
  ros::spin();
}