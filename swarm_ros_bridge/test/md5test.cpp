//
// Created by gwq on 1/4/25.
//
#include <iostream>
#include <ros/ros.h>
#include <ros/message.h>
#include <std_srvs/Empty.h>
#include <swarm_ros_bridge/AddTwoInts.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "md5test");
  ros::NodeHandle n;
  swarm_ros_bridge::AddTwoInts srv;
  std::cout << ros::service_traits::md5sum(srv) << std::endl;
  return 0;
}