#include "ros/ros.h"
#include "exercise1/Reverse.h"

//#include <cstdlib>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "our_client_node");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<exercise1::Reverse>("our_service_node/string");

  exercise1::Reverse srv;
  std::stringstream ss;

  ss << "RInS is the best course in Mordor";

  srv.request.content = ss.str();

  ros::service::waitForService("our_service_node/string", 1000);

  ROS_INFO("Sending: %s", srv.request.content.c_str());

  if (client.call(srv))
  {
    ROS_INFO("The service returned: %s", srv.response.comment.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}
