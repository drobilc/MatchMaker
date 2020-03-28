#include "ros/ros.h"
#include "exercise1/Reverse.h"

#include <string>
#include <iostream>
#include <algorithm>

bool manipulate(exercise1::Reverse::Request  &req,
         exercise1::Reverse::Response &res)
{
  res.comment = req.content;
  std::reverse(res.comment.begin(), res.comment.end());

  ROS_INFO("request: %s, response: %s", req.content.c_str(), res.comment.c_str());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "our_service_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("our_service_node/string", manipulate);
  ROS_INFO("I am ready to mess up your string!");
  ros::spin();

  return 0;
}
