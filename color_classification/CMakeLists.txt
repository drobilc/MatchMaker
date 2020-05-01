cmake_minimum_required(VERSION 2.8.3)
project(color_classification)

find_package(catkin REQUIRED COMPONENTS
	rospy roscpp cv_bridge sensor_msgs visualization_msgs tf2_ros nav_msgs std_msgs geometry_msgs message_generation
)

add_service_files(FILES ColorClassification.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
