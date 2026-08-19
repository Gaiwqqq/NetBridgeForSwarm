//
// Created by gwq on 12/13/24.
//
#include "ros/ros.h"
#include "bridge_factory.hpp"

#include <exception>

int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_bridge_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_public_;
    std::shared_ptr<ros::AsyncSpinner> spinner = std::make_shared<ros::AsyncSpinner>(4);
    try {
        BridgeFactory factory(nh, nh_public_);
        factory.startBridge();
        spinner->start();
        ros::waitForShutdown();
        factory.stopBridge();
        return 0;
    } catch (const std::exception& exception) {
        ROS_FATAL_STREAM("[Bridge] startup failed: " << exception.what());
        return 1;
    }
}
