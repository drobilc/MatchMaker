#include <ros/ros.h> // As previously said the standard ROS library
//#include <geometry_msgs/Twist.h> // For publishing geometry::Twist messages
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"our_string_publisher"); //Initialize the ROS system, give the node the name "publish_velocity"
	ros::NodeHandle nh;	//Register the node with the ROS system

	//create a publisher object.
	ros::Publisher pub = nh.advertise<std_msgs::String>("our_pub1/chat1", 1000);	//the message tyoe is inside the angled brackets
																						//topic name is the first argument
																						//queue size is the second argument


	//Loop at 2Hz until the node is shutdown.
	ros::Rate rate(2);
	
	int count = 0;
	while(ros::ok()){
		//Create the message.
		std_msgs::String msg;

		std::stringstream ss;

		ss << "hello publishing world " << count;
		msg.data=ss.str();

		//Publish the message
		pub.publish(msg);

		//Send a message to rosout
		ROS_INFO("%s", msg.data.c_str());

		//Wait until it's time for another iteration.
		rate.sleep();

		//Increase the counter
		count++;
	}

	return 0;
}