/**
 * @file bridge_factory.cpp
 * @author      Weiqi Gai   (github: Gaiwqqq) 2024.12
 * @contributor KengHou Hoi  (github: James Hoi) 2024.8
 * @contributor Peixuan Shu (shupeixuan@qq.com) 2023.1
 * @date 2024.12.13
 * @brief This file contains the implementation of the BridgeFactory class.
 * @version 0.1 beta
 * @license BSD 3-Clause License
 * @copyright (c) 2024, Weiqi Gai
 * All rights reserved.
 */

#ifndef SRC_BRIDGE_FACTORY_H
#define SRC_BRIDGE_FACTORY_H

#include "ros/ros.h"
#include "Eigen/Eigen"
#include "msgs_macro.hpp"
#include "topic_factory.h"
#include "service_factory.h"
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <zmqpp/zmqpp.hpp>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "pic_socket.h"
#include <mutex>
#include <boost/tti/has_data.hpp>
/*
zmqpp is the c++ wrapper around ZeroMQ
Install zmqpp :
    sudo apt install libzmqpp-dev
zmqpp reference link:
    https://zeromq.github.io/zmqpp/namespacezmqpp.html

Install topic_tools:
    sudo apt install ros-noetic-topic-tools
topic_tools reference link:
    http://wiki.ros.org/topic_tools
*/

class BridgeFactory
{
public:
#define DRONE_ID_NULL -9999
    explicit BridgeFactory(ros::NodeHandle& node, ros::NodeHandle& node_public);
    ~BridgeFactory();
    void startBridge();
    void stopBridge();

private:
    std::shared_ptr<ros::NodeHandle> nh_, nh_public_;
    std::string           ns_; // namespace of this node
    std::string           my_hostname_;
    bool                  do_odom_convert_;
    bool                  is_debug_;
    int                   my_drone_id_;
    std::string           my_ip_;
    XmlRpc::XmlRpcValue   ip_xml_;
    XmlRpc::XmlRpcValue   topics_xml_, services_xml_;
    std::vector<TopicCfg>                      topic_cfgs_;
    std::vector<ServiceConfig>                 service_cfgs_;
    std::map<std::string, TopicFactory::Ptr>   send_topics_, recv_topics_;
    std::map<std::string, ServiceFactory::Ptr> service_servers_, service_clients_;
    std::map<std::string, bool>                dst_hostname_map_, src_hostname_map_;
    std::map<std::string, std::string>         ip_map_;                          // map host name and IP

    void getMyHostName();
    void getIpAndTopicConfig();
    void getServiceConfigAndInit();
    void topicOperatorInit();
    static std::string checkIfPointToPointPipeline(TopicCfg& topic_cfg);

    //tools
    static bool xmlContain(XmlRpc::XmlRpcValue array, std::string value);
};

#endif //SRC_BRIDGE_FACTORY_H
