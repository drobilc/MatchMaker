cmake_minimum_required(VERSION 2.8.3)
project(object_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
	rospy roscpp cv_bridge sensor_msgs visualization_msgs tf2_ros nav_msgs std_msgs geometry_msgs message_generation
)
find_package( OpenCV REQUIRED )

add_message_files(FILES ObjectDetection.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs)
