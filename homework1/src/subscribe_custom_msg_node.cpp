//The program subscribes to turtle1/pose and prints the messages
#include <ros/ros.h>
#include "exercise1/Greeting.h" //The message type

//This is a callback function. It executes every time a new pose message arrives
void messageReceived(const exercise1::Greeting::ConstPtr& msg){	//The parameter is the message type

	ROS_INFO("I recieved the string: %s", msg->content.c_str());
}

int main(int argc, char **argv){

	//Initialize ROS system, become a registered node
	ros::init(argc, argv,"our_custom_subscriber");
	ros::NodeHandle nh;

	//Create a subscriber objet
	ros::Subscriber sub=nh.subscribe("our_pub2/chat2",1000,&messageReceived);	//First parameter is the topic
																				//Second parameter is the queue size
																				//Third parameter is a ponter to a
																					//callback function that will execute
																					//each time a new message is recieved

	//Ros will take over after this
	ros::spin();
	//the same thing can be done with a loop like:
		//while(ros::ok()){  ros::spinOnce(); }
}
