cmake_minimum_required(VERSION 2.8.3)
project(exercise6)

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  object_detection_msgs
)

## Add include directories
include_directories(
	${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Declare the catkin package
catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    Eigen
    PCL
)

add_executable (example src/example.cpp)
add_executable (example_voxelgrid src/example_voxelgrid.cpp)
add_executable (planes src/planes.cpp)
add_executable (find_plane src/find_plane.cpp)
add_executable (dominant_plane_with_obstacle_clustering src/dominant_plane_with_obstacle_clustering.cpp)
#rosbuild_add_executable (cloud_viewer src/cloud_viewer.cpp)
#rosbuild_add_executable (pose_estimation_cluster src/pose_estimation_cluster.cpp)
add_executable (don_segmentation src/don_segmentation.cpp)
add_executable (minicut_segmentation src/minicut_segmentation.cpp)
add_executable (region_growing src/region_growing.cpp)
add_executable (centroid_estimation src/centroid_estimation.cpp)
add_executable (voxelgrid src/voxelgrid.cpp)

