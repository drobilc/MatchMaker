cmake_minimum_required(VERSION 2.8.3)
project(speech_transcription)

find_package(catkin REQUIRED COMPONENTS
	rospy roscpp std_msgs message_generation
)

add_service_files(FILES InquireAffirmation.srv InquireColor.srv InquirePreferences.srv)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
