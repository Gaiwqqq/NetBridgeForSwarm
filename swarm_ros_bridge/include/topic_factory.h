//
// Created by gwq on 1/4/25.
//

#ifndef SRC_TOPIC_FACTORY_H
#define SRC_TOPIC_FACTORY_H

#include "ros/ros.h"
#include "Eigen/Eigen"
#include "msgs_macro.hpp"
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
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/compression/octree_pointcloud_compression.h"
#include "pcl/filters/voxel_grid.h"
#include "swarm_ros_bridge/PtCloudCompress.h"
#include "sensor_msgs/PointCloud2.h"
#include <mutex>
#include <boost/tti/has_data.hpp>

#define SUB_QUEUE_SIZE 10
#define PUB_QUEUE_SIZE 20
BOOST_TTI_HAS_DATA(to_drone_ids);

struct TopicCfg{
#define NOT_ONLY_ONE_DST "not_only"
    std::string name_;
    std::string raw_name_;
    std::string type_;
    std::string src_hostname_;
    std::string src_ip_;
    std::string only1_dst_hostname_;
    std::string my_ip_, my_hostname_;
    std::vector<std::string> dst_hostname_;
    XmlRpc::XmlRpcValue src_hostnames_xml;
    XmlRpc::XmlRpcValue dst_hostnames_xml;
    std::map<std::string, bool> dst_hostname_map_, src_hostname_map_, src_ip_map_, dst_ip_map_;
    double max_freq_{10.0f};
    double img_resize_rate_{1.0f};
    int  port_;
    bool cloud_compress_{false};
    double cloud_downsample_{-1};
    bool has_prefix_{true};
    bool same_prefix_{false};
    bool dynamic_dst_{false};
};

class TopicFactory
{
public:
    enum SEND_OR_RECV{ SEND, RECV };
    typedef std::shared_ptr<TopicFactory> Ptr;
    TopicFactory(const TopicCfg& topic_cfg,
                 const std::map<std::string, std::string>& ip_map,
                 SEND_OR_RECV send_or_recv,
                 const std::shared_ptr<ros::NodeHandle>& nh_public);
    ~TopicFactory();
    void createThread();
    void stopThread();

private:
    TopicCfg topic_cfg_;
    SEND_OR_RECV send_or_recv_;
    std::unique_ptr<zmqpp::socket>  sender_, receiver_;
    std::unique_ptr<UDPImgSender>   udp_sender_;
    std::unique_ptr<UDPImgReceiver> udp_receiver_;
    std::map<std::string, std::unique_ptr<zmqpp::socket>> dynamic_senders_;
    zmqpp::context_t context_;
    std::map<std::string, std::string> ip_map_;

    ros::Time       sub_last_time_;
    int             send_num_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;
    bool            recv_thread_flag_;
    bool            recv_flag_last_;
    std::thread     recv_thread_;
    std::mutex      recv_mutex_;

    void recvFunction();
    bool sendFreqControl();

    template <typename T>
    void subCallback(const ros::MessageEvent<const T> &event);

    template <typename T>
    ros::Subscriber nh_sub(std::string topic_name, const std::shared_ptr<ros::NodeHandle> &nh);

    ros::Subscriber topicSubscriber(const std::string& topic_name, std::string msg_type,
                                    const std::shared_ptr<ros::NodeHandle>& nh);
    static ros::Publisher  topicPublisher(const std::string& topic_name, std::string msg_type,
                                          const std::shared_ptr<ros::NodeHandle>& nh);
    void deserializePublish(uint8_t *buffer_ptr, size_t msg_size, std::string msg_type);

    template <typename T>
    void deserializePub(uint8_t *buffer_ptr, size_t msg_size);

    template <typename T>
    void ptCloudProcess(const T& msg, size_t & data_len, std::unique_ptr<uint8_t[]>& data);
};

#endif //SRC_TOPIC_FACTORY_H
