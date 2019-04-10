#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "ros_tutorials_action/FibonacciAction.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_client");

  actionlib::SimpleActionClient<ros_tutorials_action::FibonacciAction> action_client("ros_tutorial_action", true);

  ROS_INFO("waiting for action server to start.");
  action_client.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  ros_tutorials_action::FibonacciGoal goal;
  goal.order = 20;
  action_client.sendGoal(goal);

  bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = action_client.getState();
    ROS_INFO("Action Finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("Action did not finish before the time out.");
  }

  return 0;
}