#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "ros_tutorials_action/FibonacciAction.h"

class FibonacciAction {
 protected:
  ros::NodeHandle nh;

  actionlib::SimpleActionServer<ros_tutorials_action::FibonacciAction> action_server;
  std::string action_name;

  ros_tutorials_action::FibonacciFeedback feedback;
  ros_tutorials_action::FibonacciResult result;

 public:
  FibonacciAction(std::string name) :
      action_server(nh, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), action_name(name) {
    action_server.start();
  }

  ~FibonacciAction(void) {
  }

  void executeCB(const ros_tutorials_action::FibonacciGoalConstPtr &goal) {

    ros::Rate r(1);
    bool success = true;

    feedback.sequence.clear();
    feedback.sequence.push_back(0);
    feedback.sequence.push_back(1);

    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i",
             action_name.c_str(), goal->order, feedback.sequence[0], feedback.sequence[1]);

    for (int i = 1; i <= goal->order; i++) {
      if (action_server.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name.c_str());
        action_server.setPreempted();
        success = false;
        break;
      }
      feedback.sequence.push_back(feedback.sequence[i] + feedback.sequence[i - 1]);
      action_server.publishFeedback(feedback);
      r.sleep();
    }

    if (success) {
      result.sequence = feedback.sequence;
      ROS_INFO("%s: Succeeded", action_name.c_str());
      action_server.setSucceeded(result);
    }
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_server");
  FibonacciAction fibonacci("ros_tutorial_action");

  ros::spin();
  return 0;
}

