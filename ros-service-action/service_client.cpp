#include "ros/ros.h"
#include "services_pkg/CustomServiceMessage.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client");
  ros::NodeHandle nh;
  ros::ServiceClient service_client = nh.serviceClient<services_pkg::CustomServiceMessage>("/move_custom");
  services_pkg::CustomServiceMessage srv;
  srv.request.side = 4;
  srv.request.repetitions = 2;
  if (service_client.call(srv))
  {
    ROS_INFO("Service successfully called. Moving robot in a square.");
  }
  else
  {
    ROS_ERROR("Failed to call service /move_custom");
    return 1;
  }
  srv.request.side = 8;
  srv.request.repetitions = 1;
  if (service_client.call(srv))
  {
    ROS_INFO("Service successfully called. Moving robot in a square.");
  }
  else
  {
    ROS_ERROR("Failed to call service /move_custom");
    return 1;
  }
  return 0;
}
