cmake_minimum_required(VERSION 2.8.3)
project(face_classification)

find_package(catkin REQUIRED COMPONENTS
	rospy roscpp cv_bridge sensor_msgs visualization_msgs tf2_ros nav_msgs std_msgs geometry_msgs message_generation sensor_msgs
)

add_service_files(FILES FaceClassification.srv)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
