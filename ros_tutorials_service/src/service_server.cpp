#include "ros/ros.h"
#include "ros_tutorials_service/SrvTutorial.h"

// when a service request is received, it execute below.
bool calculation(ros_tutorials_service::SrvTutorial::Request &req,
                 ros_tutorials_service::SrvTutorial::Response &res) {
  res.result = req.a + req.b;

  ROS_INFO("request x=%ldm y=%ld", (long int) req.a, (long int) req.b);
  ROS_INFO("sending back response: %ld", (long int) res.result);

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "topic_Subscriber");
  ros::NodeHandle nh;

  ros::ServiceServer ros_tutorials_service_server = nh.advertiseService("ros_tutorial_srv", calculation);

  ROS_INFO("ready srv server!");

  // wait for service request.
  ros::spin();

  return 0;
}
