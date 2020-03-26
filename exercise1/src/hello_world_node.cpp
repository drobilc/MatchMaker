//This is the simplest possible program in ROS

//This is a standard ROS class, you will want to include it in every ROS program
#include <ros/ros.h>

int main(int argc, char *argv[])
{
	//Initialize the ROS system (the client lirary). We call this once at the begining of our programs. 
	ros::init(argc,argv,"hello_ros");	//The last parameter is the name of our node

	//Establish this program as a ROS node. This is the mechanism we use to comunicate with the ROS system.
	ros::NodeHandle nh; //Creatng this object registers the node with the ROS master

	//Send some output as a log message.
	ROS_INFO_STREAM("Hello, ROS!"); //Just an info message
	return 0;
}