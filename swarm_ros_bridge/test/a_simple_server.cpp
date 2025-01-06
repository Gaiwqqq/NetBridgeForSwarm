//
// Created by gwq on 1/5/25.
//

#include <ros/ros.h>
#include <swarm_ros_bridge/AddTwoInts.h>

bool add_two_ints_callback(swarm_ros_bridge::AddTwoInts::Request &req, swarm_ros_bridge::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("Request: %d + %d = %d", (int) req.a, (int) req.b, (int) res.sum);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "a_simple_server");
  ros::NodeHandle nh("~");
  ros::ServiceServer server = nh.advertiseService("/add_two_ints", add_two_ints_callback);
  ROS_INFO("Ready to add two ints.");
  ros::spin();
  return 0;
}
