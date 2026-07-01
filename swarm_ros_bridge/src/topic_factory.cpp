//
// Created by gwq on 1/4/25.
//
#include "topic_factory.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

// ------------ Begin of TopicFactory class ----------
TopicFactory::TopicFactory(const TopicCfg& topic_cfg,
                           const std::map<std::string, std::string>& ip_map,
                           SEND_OR_RECV send_or_recv,
                           const std::shared_ptr<ros::NodeHandle>& nh_public) {
  topic_cfg_    = topic_cfg;
  send_or_recv_ = send_or_recv;
  ip_map_       = ip_map;
  metrics_state_.topic_name = topic_cfg_.name_;
  metrics_state_.msg_type = topic_cfg_.type_;
  metrics_state_.direction = send_or_recv_ == SEND_OR_RECV::SEND ? "send" : "recv";
  metrics_state_.codec = topic_cfg_.type_ == "sensor_msgs/PointCloud2"
                             ? (topic_cfg_.cloud_compress_ ? topic_cfg_.cloud_codec_ : "raw")
                             : "raw";
  metrics_state_.configured_rate_hz = topic_cfg_.max_freq_;
  metrics_state_.window_start = ros::Time::now();
  metrics_state_.adaptive_quality_enabled = topic_cfg_.img_adaptive_quality_;
  metrics_state_.configured_jpeg_quality =
      static_cast<std::uint32_t>(topic_cfg_.img_jpeg_quality_);
  metrics_state_.current_jpeg_quality =
      static_cast<std::uint32_t>(topic_cfg_.img_jpeg_quality_);
  metrics_state_.target_bandwidth_kbps = topic_cfg_.img_target_bandwidth_kbps_;
  metrics_state_.quality_step = static_cast<std::uint32_t>(topic_cfg_.img_quality_step_);
  metrics_state_.adapt_cooldown_frames =
      static_cast<std::uint32_t>(topic_cfg_.img_adapt_cooldown_frames_);

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
    sub_last_time_ = ros::Time(0);
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
      cv_bridge::CvImageConstPtr cv_ptr;
      if (image_msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
      } else {
        cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      }
      const cv::Mat& frame = cv_ptr->image;
      cv::Mat frame_compressed;
      cv::Size target_size(floor(frame.cols * topic_cfg_.img_resize_rate_),
                           floor(frame.rows * topic_cfg_.img_resize_rate_));
      cv::resize(frame, frame_compressed, target_size);
      const int jpeg_quality = currentImageJpegQuality();
      const std::size_t encoded_size =
          udp_sender_->send(frame_compressed, jpeg_quality);
      recordSend(encoded_size);
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
        recordSend(data_len);
      }else{
        INFO_MSG_RED("[TopicFactory]: to_drone_ids not matched in config file: " << target);
        recordDrop();
      }
    }
  }
  else{
    send_array << data_len;
    send_array.add_raw(reinterpret_cast<void const *>(send_buffer.get()), data_len);
    sender_->send(send_array, dont_block);
    recordSend(data_len);
  }
}

template<typename T>
void TopicFactory::ptCloudProcess(const T &msg, size_t &data_len, std::unique_ptr<uint8_t[]> &data) {
  namespace ser = ros::serialization;
  sensor_msgs::PointCloud2 cloud_msg = msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_msg, *cloud_in);
  if (topic_cfg_.cloud_downsample_ > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(static_cast<float>(topic_cfg_.cloud_downsample_),
                    static_cast<float>(topic_cfg_.cloud_downsample_),
                    static_cast<float>(topic_cfg_.cloud_downsample_));
    sor.filter(*cloud_downsampled);
    cloud_in = cloud_downsampled;
    pcl::toROSMsg(*cloud_in, cloud_msg);
    cloud_msg.header = msg.header;
  }
  if (topic_cfg_.cloud_compress_) {
    swarm_ros_bridge::PtCloudCompress msg_send_compressed;
    msg_send_compressed.original_header = msg.header;
    msg_send_compressed.sender_stamp = msg.header.stamp;
    msg_send_compressed.original_width = cloud_msg.width;
    msg_send_compressed.original_height = cloud_msg.height;
    msg_send_compressed.original_frame_id = msg.header.frame_id;
    msg_send_compressed.voxel_leaf_size = static_cast<float>(topic_cfg_.cloud_downsample_);
    msg_send_compressed.receiver_stamp = true;
    msg_send_compressed.codec = topic_cfg_.cloud_codec_;

    if (topic_cfg_.cloud_codec_ == "draco") {
      swarm_ros_bridge::transport::EncodedPointCloud encoded;
      if (!draco_codec_.Encode(cloud_msg, &encoded)) {
        throw std::runtime_error("Draco encode failed");
      }
      msg_send_compressed.compressed_bytes = encoded.payload;
    } else {
      std::stringstream compressed_data;
      auto pt_cloud_compressor =
          std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
              pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3, false);
      pt_cloud_compressor->encodePointCloud(cloud_in, compressed_data);
      const std::string bytes = compressed_data.str();
      msg_send_compressed.compressed_bytes.assign(bytes.begin(), bytes.end());
      msg_send_compressed.codec = "pcl_octree";
    }

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
  ros::Time t_now = ros::Time::now();

  // if unlimited
  if (std::fabs(topic_cfg_.max_freq_-(-1)) < std::numeric_limits<double>::epsilon()){
    return false;
  }

  const double min_interval_sec = 1.0 / std::max(0.001, topic_cfg_.max_freq_);
  if (!sub_last_time_.isZero() &&
      (t_now - sub_last_time_).toSec() < min_interval_sec) {
    recordDrop();
    return true;
  }

  sub_last_time_ = t_now;
  send_num_++;
  return false;
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
  double latency_ms = -1.0;

  if constexpr (std::is_same<T, nav_msgs::Odometry>::value){
    // posestamped convert to odom
    geometry_msgs::PoseStamped pose;
    ser::deserialize(stream, pose);
    msg.header = pose.header;
    latency_ms = inferLatencyMs(msg.header);
    msg.header.frame_id = "world";
    msg.child_frame_id = pose.header.frame_id;
    msg.pose.pose = pose.pose;
  }
  else if constexpr (std::is_same<T, sensor_msgs::PointCloud2>::value) {
    if (topic_cfg_.cloud_compress_) {
      swarm_ros_bridge::PtCloudCompress msg_compress;
      ser::deserialize(stream, msg_compress);
      latency_ms = msg_compress.sender_stamp.isZero()
                       ? inferLatencyMs(msg_compress.original_header)
                       : (ros::Time::now() - msg_compress.sender_stamp).toSec() * 1000.0;

      if (msg_compress.codec == "draco") {
        swarm_ros_bridge::transport::EncodedPointCloud encoded;
        encoded.codec = msg_compress.codec;
        encoded.source_stamp = msg_compress.sender_stamp;
        encoded.receive_stamp = ros::Time::now();
        encoded.frame_id = msg_compress.original_frame_id;
        encoded.payload = msg_compress.compressed_bytes;
        if (!draco_codec_.Decode(encoded, &msg)) {
          throw std::runtime_error("Draco decode failed");
        }
      } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        std::stringstream compressed_data;
        compressed_data.write(
            reinterpret_cast<const char*>(msg_compress.compressed_bytes.data()),
            static_cast<std::streamsize>(msg_compress.compressed_bytes.size()));

        auto pt_cloud_compressor =
            std::make_unique<pcl::io::OctreePointCloudCompression<pcl::PointXYZ>>(
                pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, false, 1e-3, 1e-3, false);
        pt_cloud_compressor->decodePointCloud(compressed_data, cloud);
        cloud->width = msg_compress.original_width;
        cloud->height = msg_compress.original_height;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = msg_compress.original_frame_id;
      }
      msg.header.stamp = ros::Time::now();
    } else {
      ser::deserialize(stream, msg);
      latency_ms = inferLatencyMs(msg.header);
      msg.header.stamp = ros::Time::now();
    }
  }
  else{
    ser::deserialize(stream, msg);
    if constexpr (has_data_header<T, std_msgs::Header>::value) {
      latency_ms = inferLatencyMs(msg.header);
    }
  }
  // publish ROS msg
  pub_.publish(msg);
  recordReceive(msg_size, latency_ms);
}

void TopicFactory::recvFunction() {
  recv_mutex_.lock();
  double recv_freq = topic_cfg_.max_freq_ > 0.0 ? topic_cfg_.max_freq_ : 60.0;
  recv_mutex_.unlock();
  ros::Duration loop_timer(1.0 / std::max(1.0, recv_freq));
  if (topic_cfg_.type_ == "sensor_msgs/Image"){
    while (recv_thread_flag_){
      cv::Mat frame = udp_receiver_->read();
      if (!frame.empty()) {
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub_.publish(image_msg);
        recordReceive(static_cast<std::size_t>(frame.total() * frame.elemSize()), -1.0);
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

swarm_ros_bridge::diagnostics::TopicMetrics TopicFactory::GetMetricsSnapshot() const {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  const_cast<TopicFactory*>(this)->pruneMetricsWindowLocked(now);
  return swarm_ros_bridge::diagnostics::MakeTopicMetrics(metrics_state_, now);
}

int TopicFactory::currentImageJpegQuality() const {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  return static_cast<int>(metrics_state_.current_jpeg_quality);
}

void TopicFactory::recordSend(std::size_t bytes) {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  pruneMetricsWindowLocked(now);
  metrics_state_.total_sent++;
  metrics_state_.total_send_bytes += bytes;
  metrics_state_.packet_size = static_cast<std::uint32_t>(bytes);
  metrics_state_.last_send_time = now;
  metrics_state_.recent_send_times.push_back(now);
  metrics_state_.recent_send_bytes.emplace_back(now, bytes);
  maybeAdaptImageQualityLocked(now);
}

void TopicFactory::recordDrop() {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  metrics_state_.dropped_messages++;
}

void TopicFactory::recordReceive(std::size_t bytes, double latency_ms) {
  std::lock_guard<std::mutex> lock(metrics_mutex_);
  const ros::Time now = ros::Time::now();
  pruneMetricsWindowLocked(now);
  metrics_state_.total_received++;
  metrics_state_.total_recv_bytes += bytes;
  metrics_state_.packet_size = static_cast<std::uint32_t>(bytes);
  metrics_state_.last_recv_time = now;
  metrics_state_.recent_recv_times.push_back(now);
  metrics_state_.recent_recv_bytes.emplace_back(now, bytes);
  computeLatencyStats(latency_ms, &metrics_state_);
}

void TopicFactory::pruneMetricsWindowLocked(const ros::Time& now) {
  const ros::Duration max_age(metrics_state_.rate_window_sec);
  while (!metrics_state_.recent_send_times.empty() &&
         now - metrics_state_.recent_send_times.front() > max_age) {
    metrics_state_.recent_send_times.pop_front();
  }
  while (!metrics_state_.recent_recv_times.empty() &&
         now - metrics_state_.recent_recv_times.front() > max_age) {
    metrics_state_.recent_recv_times.pop_front();
  }
  while (!metrics_state_.recent_send_bytes.empty() &&
         now - metrics_state_.recent_send_bytes.front().first > max_age) {
    metrics_state_.recent_send_bytes.pop_front();
  }
  while (!metrics_state_.recent_recv_bytes.empty() &&
         now - metrics_state_.recent_recv_bytes.front().first > max_age) {
    metrics_state_.recent_recv_bytes.pop_front();
  }
}

void TopicFactory::maybeAdaptImageQualityLocked(const ros::Time& now) {
  if (topic_cfg_.type_ != "sensor_msgs/Image" || !metrics_state_.adaptive_quality_enabled) {
    return;
  }
  if (metrics_state_.target_bandwidth_kbps <= 0.0 ||
      metrics_state_.recent_send_bytes.size() < 2) {
    return;
  }
  if (metrics_state_.total_sent <
      metrics_state_.last_adapt_total_sent + metrics_state_.adapt_cooldown_frames) {
    return;
  }

  std::size_t recent_bytes = 0;
  for (const auto& item : metrics_state_.recent_send_bytes) {
    recent_bytes += item.second;
  }
  const ros::Time window_start = metrics_state_.recent_send_bytes.front().first;
  const double observed_window = std::max(0.001, (now - window_start).toSec());
  const double bandwidth_kbps =
      static_cast<double>(recent_bytes) * 8.0 / 1000.0 / observed_window;
  const double upper_bound = metrics_state_.target_bandwidth_kbps * 1.15;
  const double lower_bound = metrics_state_.target_bandwidth_kbps * 0.85;

  int current_quality = static_cast<int>(metrics_state_.current_jpeg_quality);
  const int min_quality = topic_cfg_.img_min_jpeg_quality_;
  const int max_quality = topic_cfg_.img_max_jpeg_quality_;
  const int step = topic_cfg_.img_quality_step_;

  if (bandwidth_kbps > upper_bound && current_quality > min_quality) {
    current_quality = std::max(min_quality, current_quality - step);
    metrics_state_.current_jpeg_quality = static_cast<std::uint32_t>(current_quality);
    metrics_state_.last_adapt_total_sent = metrics_state_.total_sent;
  } else if (bandwidth_kbps < lower_bound && current_quality < max_quality) {
    current_quality = std::min(max_quality, current_quality + step);
    metrics_state_.current_jpeg_quality = static_cast<std::uint32_t>(current_quality);
    metrics_state_.last_adapt_total_sent = metrics_state_.total_sent;
  }
}

double TopicFactory::inferLatencyMs(const std_msgs::Header& header) {
  if (header.stamp.isZero()) {
    return -1.0;
  }
  return (ros::Time::now() - header.stamp).toSec() * 1000.0;
}

void TopicFactory::computeLatencyStats(double latency_ms,
                                       swarm_ros_bridge::diagnostics::TopicRuntimeState* state) {
  if (state == nullptr || latency_ms < 0.0) {
    return;
  }

  if (state->total_received <= 1 || state->avg_latency_ms <= 0.0) {
    state->avg_latency_ms = latency_ms;
    state->jitter_ms = 0.0;
    state->last_latency_ms = latency_ms;
    return;
  }

  const double alpha = 0.2;
  state->avg_latency_ms = state->avg_latency_ms * (1.0 - alpha) + latency_ms * alpha;
  if (state->last_latency_ms >= 0.0) {
    const double delta = std::fabs(latency_ms - state->last_latency_ms);
    state->jitter_ms = state->jitter_ms * (1.0 - alpha) + delta * alpha;
  }
  state->last_latency_ms = latency_ms;
}

void TopicFactory::createThread() {
  recv_thread_flag_ = true;
  recv_flag_last_   = false;
  recv_thread_      = std::thread(&TopicFactory::recvFunction, this);
  recv_thread_.detach();
}

void TopicFactory::stopThread() {
  if (send_or_recv_ == SEND_OR_RECV::SEND){
    if (sender_) {
      sender_->close();
    }
    for (auto & sender : dynamic_senders_) sender.second->close();
    sub_.shutdown();
  }else if (send_or_recv_ == SEND_OR_RECV::RECV){
    recv_mutex_.lock();
    recv_thread_flag_ = false;
    recv_mutex_.unlock();
    if (receiver_) {
      receiver_->close();
    }
    pub_.shutdown();
  }
}
