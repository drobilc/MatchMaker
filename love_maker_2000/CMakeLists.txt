cmake_minimum_required(VERSION 2.8.3)
project(love_maker_2000)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  rospy
	cv_bridge
	sensor_msgs
	visualization_msgs
	tf2_ros
	nav_msgs
	std_msgs
	geometry_msgs
	dynamic_reconfigure
	genmsg
	nodelet
	nodelet_topic_tools
	pcl_conversions
	pcl_msgs
	pluginlib
	rosbag
	rosconsole
	roslib
  tf2_geometry_msgs
	object_detection_msgs
  color_classification
  image_transport
)

find_package( OpenCV REQUIRED )

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
## find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Add include directories
include_directories(
  include ${catkin_INCLUDE_DIRS}
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
    rospy
    message_runtime
    sensor_msgs
    std_msgs
    object_detection_msgs
    color_classification
    image_transport
    cv_bridge
  DEPENDS
    Boost
    Eigen
    PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable (cylinder_and_ring_detector src/cylinder_and_ring_detector.cpp)
target_link_libraries(cylinder_and_ring_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(cylinder_and_ring_detector ${catkin_EXPORTED_TARGETS})

add_executable (voxelgrid_new src/voxelgrid.cpp)