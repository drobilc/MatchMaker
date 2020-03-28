//Publishing randomly generated velocity messages for turtle-simulator

#include <ros/ros.h> // As previously said the standard ROS library
#include "exercise1/Greeting.h"

#include <sstream>

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"our_custom_publisher"); //Initialize the ROS system, give the node the name "publish_velocity"
	ros::NodeHandle nh;	//Register the node with the ROS system

	//create a publisher object.
	ros::Publisher pub = nh.advertise<exercise1::Greeting>("our_pub2/chat2", 1000);	//the message tyoe is inside the angled brackets
																						//topic name is the first argument
																						//queue size is the second argument
	//Loop at 1Hz until the node is shutdown.
	ros::Rate rate(1);

	while(ros::ok()){
		//Create the message.
		exercise1::Greeting msg;

		std::stringstream ss;

		ss << "hello custom messaging world";
		msg.content=ss.str();
		//msg.count=seq;

		//Publish the message
		pub.publish(msg);

		//Send a message to rosout
		ROS_INFO("%s", msg.content.c_str());

		//Wait untilit's time for another iteration.
		rate.sleep();
	}

	return 0;
}
