#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actions_pkg/CustomActionMsgAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

class CustomAction
{
protected:
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<actions_pkg::CustomActionMsgAction> as;
  std::string action_name;
  actions_pkg::CustomActionMsgFeedback feedback;
  actions_pkg::CustomActionMsgResult result;
  int rate_hz;
  bool success;
  ros::Rate *rate;
  ros::Publisher move_pub;
  geometry_msgs::Twist move_msg;
  ros::Publisher takeoff_pub;
  std_msgs::Empty takeoff_msg;
  ros::Publisher land_pub;
  std_msgs::Empty land_msg;
public:
  CustomAction(std::string name) :
    as(nh, name, boost::bind(&CustomAction::executeCB, this, _1), false),
    action_name(name)
  {
    as.start();
    rate_hz = 1;
    success = true;
    rate = new ros::Rate(rate_hz);
    move_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    takeoff_pub = nh.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
    land_pub = nh.advertise<std_msgs::Empty>("/drone/land", 1000);
  }

  void executeCB(const actions_pkg::CustomActionMsgGoalConstPtr &goal)
  {
    std::string req = goal->goal;
    if(req == "takeoff"){
        feedback.feedback = "Taking Off";
        this->takeoff_drone();
    }
    else if(req == "land"){
        feedback.feedback = "Landing";
        this->stop_drone();
        this->land_drone();
    }
    as.publishFeedback(feedback);
    rate->sleep();
    if(success)
    {
        ROS_INFO("%s: Succeeded", req.c_str());
        as.setSucceeded(result);
    }
  }

  void takeoff_drone(void)
  {
    ROS_INFO("Taking Off Drone...");
    int i = 0;
    while (i < 4)
    {
      takeoff_pub.publish(takeoff_msg);
      i++;
      rate->sleep();
    }
  }

  void land_drone(void)
  {
    ROS_INFO("Landing Drone...");
    int i = 0;
    while (i < 4)
    {
      land_pub.publish(land_msg);
      i++;
      rate->sleep();
    }
  }

  void stop_drone(void)
  {
    ROS_INFO("Stopping Drone...");
    int i = 0;
    while (i < 3)
    {
      move_msg.linear.x = 0;
      move_msg.angular.z = 0;
      move_pub.publish(move_msg);
      i++;
      rate->sleep();
    }
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");
  CustomAction action("action_custom_msg_as");
  ros::spin();
  return 0;
}
