//
// Created by gwq on 1/5/25.
//

#include <ros/ros.h>
#include <swarm_ros_bridge/AddTwoInts.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "a_simple_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<swarm_ros_bridge::AddTwoInts>("/drone1/add_two_ints");
    swarm_ros_bridge::AddTwoInts srv;
    srv.request.a = 12345;
    srv.request.b = 67890;
    if (client.call(srv)) {
        ROS_INFO("Sum of %d and %d is %d", srv.request.a, srv.request.b, srv.response.sum);
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    return 0;
}