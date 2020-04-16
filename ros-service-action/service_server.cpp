#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "services_pkg/CustomServiceMessage.h"
#include <unistd.h>

class Move
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer my_service;
    ros::Publisher vel_pub;
    ros::Rate *rate;
    geometry_msgs::Twist vel_msg;
public:
    Move();
    bool callback(services_pkg::CustomServiceMessage::Request  &req,
                 services_pkg::CustomServiceMessage::Response &res);
    void move_forward(int n_seconds);
    void turn_90(int n_seconds);
};

Move::Move(){
    my_service = nh.advertiseService("/move_custom", &Move::callback, this);
    ROS_INFO("Service /move_custom Ready");
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    rate = new ros::Rate(1);
}

bool Move::callback(services_pkg::CustomServiceMessage::Request  &req,
                 services_pkg::CustomServiceMessage::Response &res){
  ROS_INFO("The Service /move_custom has been called");
  int i = 0;
  int side_seconds = req.side;
  while (i < req.repetitions)
    {
        for(int j = 0; j < 4; j++){
            move_forward(side_seconds);
            turn_90(2);
        }
        i++;
        rate->sleep();
    }
  res.success = true;
  ROS_INFO("Finished /move_custom");
  return true;
}

void Move::move_forward(int n_seconds){
    ROS_INFO("Moving forward Drone...");
    int i = 0;
    while (i < n_seconds)
    {
      vel_msg.linear.x = 0.1;
      vel_msg.angular.z = 0;
      vel_pub.publish(vel_msg);
      i++;
      rate->sleep();
    }
    vel_msg.linear.x = 0;
    vel_pub.publish(vel_msg);
}

void Move::turn_90(int n_seconds){
    ROS_INFO("Turning Drone...");
    int i = 0;
    while (i < n_seconds)
    {
      vel_msg.linear.x = 0;
      vel_msg.angular.z = 0.4;
      vel_pub.publish(vel_msg);
      i++;
      rate->sleep();
    }
    vel_msg.linear.z = 0;
    vel_pub.publish(vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "service_server");
  Move move;
  ros::spin();
  return 0;
}
