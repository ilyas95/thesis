#include <ros/ros.h>
#include <actions_pkg/CustomActionMsgAction.h>
#include <actionlib/client/simple_action_client.h>

void doneCb(const actionlib::SimpleClientGoalState& state,
            const actions_pkg::CustomActionMsgResultConstPtr& result)
{
  ROS_INFO("The Action has been completed");
  ros::shutdown();
}

void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const actions_pkg::CustomActionMsgFeedbackConstPtr& feedback)
{
  ROS_INFO("Action in progress");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_client");
  actionlib::SimpleActionClient<actions_pkg::CustomActionMsgAction> client("action_custom_msg_as", true);
  client.waitForServer();
  actions_pkg::CustomActionMsgGoal goal;
  goal.goal = "takeoff";
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  goal.goal = "move_forward";
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  goal.goal = "land";
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  client.waitForResult();
  return 0;
}
