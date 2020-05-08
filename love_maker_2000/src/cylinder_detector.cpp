#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
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

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

using namespace Eigen;

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pub_cylinder;
ros::Publisher pub_testing;
ros::Publisher pub_markers;
visualization_msgs::MarkerArray markers;

ros::ServiceClient color_classifier;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZ PointT;

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

bool isEnabled = false;

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

    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // Write the planar inliers to disk
    extract.filter(*cloud_plane);

    if (!cloud_plane->points.empty()) {
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
void find_cylinders(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr cloud_filtered_normals, cv_bridge::CvImagePtr image, ros::Time timestamp)
{
  // Objects we need
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  Eigen::Vector4f centroid;

  // Datasets
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  // Create the segmentation object for cylinder segmentation and set all the parameters
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

  // Write the cylinder inliers to disk
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_cylinder);
  if (cloud_cylinder->points.size() >= cylinder_points_threshold) {
    pcl::compute3DCentroid(*cloud_cylinder, centroid);

    //Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;
    visualization_msgs::Marker marker;
    object_detection_msgs::ObjectDetection cylinder_detection_message;
    geometry_msgs::TransformStamped tss;

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = timestamp;

    point_map.header.frame_id = "map";
    point_map.header.stamp = timestamp;

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    // std::cerr << "Centroids: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << std::endl;

    // Get robot coordinates in map frame
    geometry_msgs::PointStamped pointRobot, pointRobotMap;
    pointRobot.header.frame_id = "camera_rgb_optical_frame";
    pointRobot.header.stamp = point_camera.header.stamp;
    pointRobotMap.header.frame_id = "map";
    pointRobotMap.header.stamp = pointRobot.header.stamp;

    try {
      tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", timestamp);
    } catch (tf2::TransformException &ex) {
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

    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
    puby.publish(outcloud_cylinder);

    // If the last image message is available, get color of the centroid from the received image
    color_classification::ColorClassification colorClassificator;
    std_msgs::ColorRGBA color;
    std_msgs::String classifiedColor;

    cv::Point centroidPoint = pointToPixel(centroid);
    color = averageColorAround(image->image, centroidPoint, 5);

    // After average color has been calculated, classify it
    colorClassificator.request.color = color;

    // Classify the selected color using color_classification service
    if (color_classifier.call(colorClassificator)) {
      // std::cerr << "CLASSIFIED COLOR: " << colorClassificator.response.classified_color << std::endl;
      classifiedColor.data = colorClassificator.response.classified_color;
      // Color has beem successfully classified
    } else {
      ROS_ERROR("Failed to call color_classifier service");
      return;
    }

    // Publish the message only if point is not nan
    if (point_map.point.x == point_map.point.x && point_map.point.z == point_map.point.z && point_map.point.y == point_map.point.y) {
      // Correctly configure the header
      cylinder_detection_message.header.stamp = timestamp;
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
  } else {
    *cloud_filtered = *cloud;
    *cloud_filtered_normals = *cloud_normals;
  }
}

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  if (!isEnabled)
    return;

  // GET THE ACTUAL IMAGE FROM RECEIVED IMAGE MESSAGE
  cv_bridge::CvImagePtr rgbImage;
  try {
    rgbImage = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image->encoding.c_str());
  }

  // CONVERT THE POINT CLOUD FROM sensor_msgs::PointCloud2ConstPtr TO pcl::PCLPointCloud2
  pcl::PCLPointCloud2* cloud_blob = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_blob);
  pcl_conversions::toPCL(*cloud_msg, *cloud_blob);

  // If there are no points in point cloud, exit
  if (cloud_blob->width == 0 || cloud_blob->height == 0)
    return;

  // Get the received time of point cloud from image / pointcloud header
  ros::Time timestamp = image->header.stamp;

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
  // std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;
  if (cloud->points.empty()) return;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 2);
  pass.filter(*cloud_filtered);
  // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Remove all planes
  remove_all_planes(cloud_filtered, cloud_normals, cloud_filtered2, cloud_normals2);

  // Find cylinders and remove them from the point cloud
  find_cylinders(cloud_filtered2, cloud_normals2, cloud_filtered3, cloud_normals3, rgbImage, timestamp);
}

void toggleCallback(const std_msgs::Bool::ConstPtr& enabled) {
  isEnabled = enabled->data;
  std::cerr << "Cylinder detector enabled: " << isEnabled << std::endl;
}

void get_parameters(ros::NodeHandle nh)
{
  // Get parameters for cylinder segmentation from the launch file
  nh.getParam("/cylinder_detector/normal_distance_weight", cylinder_normal_distance_weight);
  nh.getParam("/cylinder_detector/max_iterations", cylinder_max_iterations);
  nh.getParam("/cylinder_detector/distance_threshold", cylinder_distance_threshold);
  nh.getParam("/cylinder_detector/radius_max", cylinder_radius_max);
  nh.getParam("/cylinder_detector/radius_min", cylinder_radius_min);
  nh.getParam("/cylinder_detector/cylinder_points_threshold", cylinder_points_threshold);
  // Get parameters for plane segmentation from the launch file
  nh.getParam("/cylinder_detector/plane_points_threshold", plane_points_threshold);

  nh.getParam("/cylinder_detector/torus_ransac_max_iterations", torus_ransac_max_iterations);
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

  message_filters::Subscriber<sensor_msgs::Image> imageSubscriber(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloudSubscriber(nh, "input", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> synchronizer(imageSubscriber, pointcloudSubscriber, 10);
  synchronizer.registerCallback(boost::bind(&callback, _1, _2));

  // Subscriber to enable and disable cylinder detections
  ros::Subscriber toggleSubscriber = nh.subscribe("/cylinder_detector_toggle", 10, toggleCallback);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2>("point_cloud/planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2>("point_cloud/cylinder", 1);
  pub_testing = nh.advertise<pcl::PCLPointCloud2>("point_cloud/log", 1);

  pub_cylinder = nh.advertise<object_detection_msgs::ObjectDetection>("cylinder_detections_raw", 1);

  color_classifier = nh.serviceClient<color_classification::ColorClassification>("color_classifier");

  // Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Spin
  ros::spin();
}
