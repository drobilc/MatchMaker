#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
	double scale_linear, scale_angular;

	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle nh;
	//ros::NodeHandle pnh("~");

	//create a publisher object.
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    // For the turtle simulation map the topic to /turtle1/cmd_vel
    // For the turtlebot simulation and Turtlebot map the topic to /cmd_vel_mux/input/navi

	//Seed the random number generator.
	srand(time(0));

	//Loop at 2Hz until the node is shutdown.
	ros::Rate rate(2);

	//pnh.getParam("scale_linear", scale_linear);
	//pnh.getParam("scale_angular", scale_angular);

	ros::param::get("~scale_linear", scale_linear);
	ros::param::get("~scale_angular", scale_angular);

	while(ros::ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = scale_linear * (double(rand())/double(RAND_MAX));
		msg.angular.z = scale_angular * 2 * (double(rand())/double(RAND_MAX)-0.5);

		pub.publish(msg);

		ROS_INFO_STREAM("Sending random velocity command:"
			<< "linear=" << msg.linear.x << scale_linear
			<<"angular=" << msg.angular.z);

		//Wait untilit's time for another iteration.
		rate.sleep();
	}
	return 0;
}
