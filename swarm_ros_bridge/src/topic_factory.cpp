//
// Created by gwq on 1/4/25.
//
#include "topic_factory.h"

// ------------ Begin of TopicFactory class ----------
TopicFactory::TopicFactory(const TopicCfg& topic_cfg,
                           const std::map<std::string, std::string>& ip_map,
                           SEND_OR_RECV send_or_recv,
                           const std::shared_ptr<ros::NodeHandle>& nh_public) {
  topic_cfg_    = topic_cfg;
  send_or_recv_ = send_or_recv;
  ip_map_       = ip_map;

  // process sockets!
  if (send_or_recv_ == SEND_OR_RECV::SEND){
    if (topic_cfg_.type_ == "sensor_msgs/Image"){
      udp_sender_ = std::make_unique<UDPImgSender>(ip_map_[topic_cfg_.only1_dst_hostname_].c_str(), topic_cfg_.port_);
    }else if (!topic_cfg_.dynamic_dst_){
      sender_ = std::make_unique<zmqpp::socket>(context_, zmqpp::socket_type::pub);
      const std::string url = "tcp://" + topic_cfg_.src_ip_ + ":" + std::to_string(topic_cfg_.port_);
      sender_->bind(url);
    }else{
      for (const auto& dst : topic_cfg_.dst_hostname_map_){
        if (dst.second && dst.first != topic_cfg_.my_hostname_){
          std::unique_ptr<zmqpp::socket> sender_tmp(new zmqpp::socket(context_, zmqpp::socket_type::pub));
          dynamic_senders_[dst.first] = std::move(sender_tmp);
          const std::string url = "tcp://" + ip_map_[dst.first] + ":" + std::to_string(topic_cfg_.port_);
          dynamic_senders_[dst.first]->connect(url);
        }
      }
    }
    if (topic_cfg_.dynamic_dst_)
      INFO_MSG(" SEND DYNAMIC | " << "src -> "<< topic_cfg_.src_hostname_ << (topic_cfg_.src_hostname_map_[topic_cfg_.my_hostname_] ? "(ME) | " : " | ")
                        << topic_cfg_.name_ << " | " << topic_cfg_.max_freq_ << "Hz");
    else
      INFO_MSG(" SEND | " << "src -> "<< topic_cfg_.src_hostname_ << (topic_cfg_.src_hostname_map_[topic_cfg_.my_hostname_] ? "(ME) | " : " | ")
                        << topic_cfg_.name_ << " | " << topic_cfg_.max_freq_ << "Hz");

  }else if (send_or_recv == SEND_OR_RECV::RECV){
    if (topic_cfg_.type_ == "sensor_msgs/Image"){
      udp_receiver_ = std::make_unique<UDPImgReceiver>(topic_cfg_.port_);
    }else{
      std::string const zmq_topic = "";
      std::unique_ptr<zmqpp::socket> receiver_tmp(new zmqpp::socket(context_, zmqpp::socket_type::sub));
      receiver_tmp->subscribe(zmq_topic);
      if (!topic_cfg_.dynamic_dst_){
        const std::string url = "tcp://" + topic_cfg_.src_ip_ + ":" + std::to_string(topic_cfg_.port_);
        receiver_tmp->connect(url);
      }else{
        const std::string url = "tcp://" + topic_cfg_.my_ip_ + ":" + std::to_string(topic_cfg_.port_);
        receiver_tmp->bind(url);
      }
      receiver_ = std::move(receiver_tmp);
    }
    INFO_MSG(" RECV | " << topic_cfg_.name_ << " | " << topic_cfg_.max_freq_ << "Hz");
  }

  // process ros publisher and subscriber
  if (send_or_recv_ == SEND_OR_RECV::SEND){
    sub_last_time_ = ros::Time::now();
    send_num_      = 0;
    sub_           = topicSubscriber(topic_cfg_.name_, topic_cfg_.type_, nh_public);
  }else if (send_or_recv_ == SEND_OR_RECV::RECV){
    pub_ = topicPublisher(topic_cfg.name_, topic_cfg_.type_, nh_public);
  }
}

TopicFactory::~TopicFactory() = default;

ros::Subscriber TopicFactory::topicSubscriber(const std::string& topic_name, std::string msg_type,
                                              const std::shared_ptr<ros::NodeHandle> &nh) {
#define X(type, classname)                    \
      if (msg_type == type)                       \
        return nh_sub<classname>(topic_name, nh);
  MSGS_MACRO
#undef X

          ROS_FATAL("[bridge_node][topic_subsriber] Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
  exit(1);
}

template <typename T>
void TopicFactory::subCallback(const ros::MessageEvent<const T> &event) {
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  topic = topic_cfg_.name_;
  const std::shared_ptr<T> ptr = std::make_shared<T>(*event.getConstMessage());
  T& msg = *ptr.get();
  // frequency ctrl
  if (sendFreqControl()) return;

  namespace ser = ros::serialization;
  size_t data_len;              // bytes length of msg
  std::unique_ptr<uint8_t[]> send_buffer;
  zmqpp::message send_array;

  // odom convert to posestamped
  if constexpr (std::is_same<T, nav_msgs::Odometry>::value){
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.header.frame_id = msg.child_frame_id;
    // pose.header.frame_id = std::string("drone_") + std::to_string(my_drone_id);
    pose.pose = msg.pose.pose;

    /* serialize the sending messages into send_buffer */
    data_len = ser::serializationLength(pose);
    send_buffer = std::make_unique<uint8_t[]>(data_len); // create a dynamic length array
    ser::OStream stream(send_buffer.get(), data_len);
    ser::serialize(stream, pose);
  }
  else if constexpr (std::is_same<T, sensor_msgs::Image>::value){
    try{
      sensor_msgs::ImageConstPtr image_msg = event.getConstMessage();
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat frame = cv_ptr->image;
      cv::Mat frame_compressed;
      cv::Size target_size(floor(frame.cols * topic_cfg_.img_resize_rate_),
                           floor(frame.rows * topic_cfg_.img_resize_rate_));
      cv::resize(frame, frame_compressed, target_size);
      udp_sender_->send(frame_compressed);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("[SubCallback]: cv_bridge exception: %s", e.what());
    }
    return;
  }
  else if constexpr (std::is_same<T, sensor_msgs::PointCloud2>::value) {
    ptCloudProcess(msg, data_len, send_buffer);
  }
  else{
    /* serialize the sending messages into send_buffer */
    data_len = ser::serializationLength(msg);
    send_buffer = std::make_unique<uint8_t[]>(data_len); // create a dynamic length array
    ser::OStream stream(send_buffer.get(), data_len);
    ser::serialize(stream, msg);
  }
  /* zmq send message */
  bool dont_block = false; // Actually for PUB mode zmq socket, send() will never block
  if constexpr (has_data_to_drone_ids<T, std::vector<uint8_t>>::value){
    send_array << data_len;
    send_array.add_raw(reinterpret_cast<void const *>(send_buffer.get()), data_len);
    for (size_t j = 0; j < msg.to_drone_ids.size(); ++j){
      int id = msg.to_drone_ids[j];
      std::string target = "drone" + std::to_string(id);
      if (dynamic_senders_.find(target) != dynamic_senders_.end()) {
        dynamic_senders_[target]->send(send_array, dont_block);
      }else{
        INFO_MSG_RED("[TopicFactory]: to_drone_ids not matched in config file: " << target);
      }
    }
  }
  else{
    send_array << data_len;
    send_array.add_raw(reinterpret_cast<void const *>(send_buffer.get()), data_len);
    sender_->send(send_array, dont_block);
  }
}

template<typename T>
void TopicFactory::ptCloudProcess(const T &msg, size_t &data_len, std::unique_ptr<uint8_t[]> &data) {
  namespace ser = ros::serialization;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(msg, *cloud_in);
  if (topic_cfg_.cloud_downsample_ > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(static_cast<float>(topic_cfg_.cloud_downsample_),
                    static_cast<float>(topic_cfg_.cloud_downsample_),
                    static_cast<float>(topic_cfg_.cloud_downsample_));
    sor.filter(*cloud_downsampled);
    cloud_in = cloud_downsampled;
  }
  if (topic_cfg_.cloud_compress_) {
    std::stringstream compressed_data;
    std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> pt_cloud_compressor;
    pt_cloud_compressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
      pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3, false);
    pt_cloud_compressor->encodePointCloud(cloud_in, compressed_data);

    swarm_ros_bridge::PtCloudCompress msg_send_compressed;
    msg_send_compressed.compressed_data.data = compressed_data.str();
    msg_send_compressed.original_width       = cloud_in->width;
    msg_send_compressed.original_height      = cloud_in->height;
    msg_send_compressed.original_frame_id    = cloud_in->header.frame_id;

    /* serialize the sending messages into send_buffer */
    data_len = ser::serializationLength(msg_send_compressed);
    data     = std::make_unique<uint8_t[]>(data_len);
    ser::OStream stream(data.get(), data_len);
    ser::serialize(stream, msg_send_compressed);
  }else {
    data_len = ser::serializationLength(msg);
    data     = std::make_unique<uint8_t[]>(data_len); // create a dynamic length array
    ser::OStream stream(data.get(), data_len);
    ser::serialize(stream, msg);
  }
}

template <typename T>
ros::Subscriber TopicFactory::nh_sub(std::string topic_name, const std::shared_ptr<ros::NodeHandle> &nh){
  return nh->subscribe<const ros::MessageEvent<T const>&>
  (topic_name, SUB_QUEUE_SIZE, &TopicFactory::subCallback<T>, this, ros::TransportHints().tcpNoDelay());
}

bool TopicFactory::sendFreqControl() {
  bool discard_flag = false;
  ros::Time t_now = ros::Time::now();

  // if unlimited
  if (std::fabs(topic_cfg_.max_freq_-(-1)) < std::numeric_limits<double>::epsilon()){
    return false;
  }
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num_ + 1) / (t_now - sub_last_time_).toSec() >topic_cfg_.max_freq_)
    discard_flag = true;
  else{
    discard_flag = false;
    send_num_++;
  }
  // freq control period (1s)
  if ((t_now - sub_last_time_).toSec() > 30.0){
    sub_last_time_ = t_now;
    send_num_ = 0;
  }
  return discard_flag; // flag of discarding this message
}

ros::Publisher TopicFactory::topicPublisher(const std::string& topic_name, std::string msg_type,
                                            const std::shared_ptr<ros::NodeHandle> &nh) {
#define X(type, classname) \
        if (msg_type == type)    \
          return nh->advertise<classname>(topic_name, PUB_QUEUE_SIZE);
  MSGS_MACRO
#undef X

          ROS_FATAL("[bridge_node][topicPublisher] Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
  exit(1);
}

void TopicFactory::deserializePublish(uint8_t *buffer_ptr, size_t msg_size, std::string msg_type) {
#define X(type, classname) \
        if (msg_type == type)    \
          return deserializePub<classname>(buffer_ptr, msg_size);
  MSGS_MACRO
#undef X

          ROS_FATAL("[bridge_node][deserializePublish] Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
  exit(1);
}

template <typename T>
void TopicFactory::deserializePub(uint8_t *buffer_ptr, size_t msg_size) {
  namespace ser = ros::serialization;
  ser::IStream stream(buffer_ptr, msg_size);
  // deserialize the receiving messages into ROS msg
  T msg;

  if constexpr (std::is_same<T, nav_msgs::Odometry>::value){
    // posestamped convert to odom
    geometry_msgs::PoseStamped pose;
    ser::deserialize(stream, pose);
    msg.header = pose.header;
    msg.header.frame_id = "world";
    msg.child_frame_id = pose.header.frame_id;
    msg.pose.pose = pose.pose;
  }
  else if constexpr (std::is_same<T, sensor_msgs::PointCloud2>::value) {
    if (topic_cfg_.cloud_compress_) {
      swarm_ros_bridge::PtCloudCompress msg_compress;
      ser::deserialize(stream, msg_compress);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      std::stringstream compressed_data;
      compressed_data << msg_compress.compressed_data.data;

      std::unique_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>> pt_cloud_compressor;
      pt_cloud_compressor = std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
        pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3, false);
      pt_cloud_compressor->decodePointCloud(compressed_data, cloud);
      cloud->width           = msg_compress.original_width;
      cloud->height          = msg_compress.original_height;
      cloud->header.frame_id = msg_compress.original_frame_id;
      pcl::toROSMsg(*cloud, msg);
    }else
      ser::deserialize(stream, msg);
  }
  else{
    ser::deserialize(stream, msg);
  }
  // publish ROS msg
  pub_.publish(msg);
}

void TopicFactory::recvFunction() {
  recv_mutex_.lock();
  double recv_freq = 1000.0 / topic_cfg_.max_freq_;
  recv_mutex_.unlock();
  ros::Duration loop_timer(1.0/recv_freq);
  if (topic_cfg_.type_ == "sensor_msgs/Image"){
    while (recv_thread_flag_){
      cv::Mat frame = udp_receiver_->read();
      if (!frame.empty()) {
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub_.publish(image_msg);
      }
      loop_timer.sleep();
    }
    return;
  }

  zmqpp::poller poller;
  poller.add(*receiver_);
  // receive and process message (except image)
  while (recv_thread_flag_){
    if (poller.poll(1000)) { // 1000ms timeout
      /* receive and process message */
      zmqpp::message recv_message;
      // receive(&,true) for non-blocking, receive(&,false) for blocking
      bool dont_block = false; // 'true' leads to high cpu load
      bool recv_flag = receiver_->receive(recv_message, dont_block); // receive success flag
      if (recv_flag) {
        size_t data_len;
        recv_message >> data_len; // unpack meta data
        std::unique_ptr<uint8_t[]> recv_buffer = std::make_unique<uint8_t[]>(data_len);
        memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_message.raw_data(recv_message.read_cursor())),
               data_len);
        try {
          deserializePublish(recv_buffer.get(), data_len, topic_cfg_.type_);
        } catch (std::exception &e) {
          recv_mutex_.lock();
          ROS_ERROR("[recvFunction][%s]: %s", topic_cfg_.name_.c_str(), e.what());
          recv_mutex_.unlock();
        }
      }
      if (recv_flag != recv_flag_last_) {
        std::string topicName = topic_cfg_.name_;
        ROS_INFO("[bridge node] \"%s\" received!", topicName.c_str());
        recv_flag_last_ = recv_flag;
      }
    }
  }
}

void TopicFactory::createThread() {
  recv_thread_flag_ = true;
  recv_flag_last_   = false;
  recv_thread_      = std::thread(&TopicFactory::recvFunction, this);
  recv_thread_.detach();
}

void TopicFactory::stopThread() {
  if (send_or_recv_ == SEND_OR_RECV::SEND){
    sender_->close();
    for (auto & sender : dynamic_senders_) sender.second->close();
    sub_.shutdown();
  }else if (send_or_recv_ == SEND_OR_RECV::RECV){
    recv_mutex_.lock();
    recv_thread_flag_ = false;
    recv_mutex_.unlock();
    receiver_->close();
    pub_.shutdown();
  }
}
