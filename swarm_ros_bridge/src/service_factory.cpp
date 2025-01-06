//
// Created by gwq on 1/4/25.
//
#include "service_factory.h"


ServiceFactory::ServiceFactory(ServiceFactory::ClientOrServer client_or_server, std::shared_ptr<ros::NodeHandle> nh,
                               const ServiceConfig& config) {
  client_or_server_ = client_or_server;
  nh_ = nh;
  config_ = config;
  if (!typeExists(config_.service_type)) {
    INFO_MSG_RED("[SrvFactory] Invalid service type: " << config_.service_type);
    return;
  }
  if (client_or_server_ == ServiceFactory::CLIENT) {
    ros::AdvertiseServiceOptions options;
    options.service        = config_.service_prefix_name;
    options.callback_queue = nullptr;
    options.datatype       = config_.service_type;
    options.md5sum         = getServiceMd5(config_.service_type);
    options.helper         = boost::make_shared<ServiceCallbackHelper>(config_.service_prefix_name,
                                                                       config_.service_name, this);
    options.req_datatype   = config_.service_type + "Request";
    options.res_datatype   = config_.service_type + "Response";

    INFO_MSG_YELLOW("[SrvFactory] Advertising service: " << config_.service_prefix_name << " type: " << config_.service_type);
    server_handle_client_msg_ = nh_->advertiseService(options);
  }else if (client_or_server_ == ServiceFactory::SERVER) {
    server_socket_.reset(new zmqpp::socket(context_, zmqpp::socket_type::router));
    try{
      server_socket_->bind("tcp://" + config_.server_ip + ":" + config_.port);
      INFO_MSG_GREEN("[SrvFactory] Service |"<< config_.service_name <<"| socket bound to: tcp://"
                                              << config_.server_ip << ":" << config_.port);
      server_thread_ = std::make_unique<boost::thread>(&ServiceFactory::step, this);
      server_thread_->detach();
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("[SrvFactory] Failed to bind server socket: " << e.what());
      return;
    }
  }
}

bool ServiceFactory::call(const std::string &service_name, ros::ServiceCallbackHelperCallParams &params) const {
  zmqpp::socket socket(context_, zmqpp::socket_type::dealer);
  socket.set(zmqpp::socket_option::identity, config_.my_hostname.c_str());
  try{
    socket.connect("tcp://" + config_.server_ip + ":" + config_.port);
    zmqpp::message request;
    request << service_name.length();
    request.add_raw(service_name.c_str(), service_name.length());
    request << params.request.num_bytes;
    request.add_raw(params.request.buf.get(), params.request.num_bytes);
    socket.send(request);

    // recv response
    zmqpp::message response;
    zmqpp::poller poller;
    poller.add(socket);
    if (poller.poll(1000)){
      bool recv_flag = socket.receive(response, false);
      if (recv_flag){
        size_t resp_length;
        response >> resp_length;
        boost::shared_array<uint8_t> data(new uint8_t[resp_length]);
        memcpy(data.get(), static_cast<const uint8_t *>(response.raw_data(response.read_cursor())), resp_length);
        params.response = ros::SerializedMessage(data, resp_length);
        INFO_MSG_GREEN("[SrvFactory] service |"<< service_name <<"| get response .");
        return true;
      }
    }else{
      INFO_MSG_RED("[SrvFactory] service |"<< service_name <<"| Failed to recv response .");
      return false;
    }
  }catch (const std::exception& e) {
    ROS_ERROR_STREAM("[SrvFactory] Failed to call service: " << service_name << " error: " << e.what());
    return false;
  }
}

std::string ServiceFactory::getServiceMd5(const std::string &type_in) {
#define X(type, classname) \
    if (type_in == type)                       \
      return getServiceMsgMd5<classname>(type_in);
  SRVS_MACRO
#undef X
  return "";
}

bool ServiceFactory::typeExists(const std::string &type_in) {
#define X(type, classname) \
    if (type_in == type)   \
      return true;
  SRVS_MACRO
#undef X
  return false;
}

void ServiceFactory::step() {
  zmqpp::poller poller;
  poller.add(*server_socket_);
  while (server_shutdown_flag_){
    if (poller.poll(1000)){
      shared_mutex_.lock();
      if (poller.has_input(*server_socket_)){
        zmqpp::message request_tmp;
        bool recv_flag = server_socket_->receive(request_tmp, false);
        shared_mutex_.unlock();
        if (recv_flag){
          shared_mutex_.lock();
          client_handlers_.push_back(std::make_shared<ClientHandler>(std::move(request_tmp), server_socket_));
          shared_mutex_.unlock();
        }
      }
    }
  }
}

void ServiceFactory::stopServerThread() {
  shared_mutex_.lock();
  server_shutdown_flag_ = false;
  shared_mutex_.unlock();
}

template<typename T>
std::string ServiceFactory::getServiceMsgMd5(const std::string &type_in) {
  std::string error_msg, res;
  if (!ros::names::validate(type_in, error_msg)){
    ROS_ERROR_STREAM("[SrvFactory] Invalid service type name: " << error_msg);
    return "";
  }
  T srv_msg;
  try {
    res = ros::service_traits::md5sum(srv_msg);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[SrvFactory] Failed to get md5sum for service type: " << type_in << " error: " << e.what());
    return "";
  }
  return res;
}

// ------------------------ ServiceCallbackHelper ------------------------//
bool ServiceCallbackHelper::call(ros::ServiceCallbackHelperCallParams &params) {
  return m_client_->call(m_name_raw_, params);
}

// --------------------------- Client Handler ---------------------------//
ClientHandler::ClientHandler(zmqpp::message request, std::shared_ptr<zmqpp::socket> & socket) {
  shared_mutex_.lock();
  socket_  = socket;
  request_ = std::make_unique<zmqpp::message>(std::move(request));
  thread_  = std::make_unique<boost::thread>(&ClientHandler::run, this);
  shared_mutex_.unlock();
  thread_->detach();
}

void ClientHandler::run(){
  std::string client_id, request_name, service_name;
  size_t      service_name_length, request_length;
  *request_ >> client_id;             // router 自动添加 client_id
  *request_ >> service_name_length;
  *request_ >> service_name;
  *request_ >> request_length;

  boost::shared_array<uint8_t> req_data(new uint8_t[request_length]);
  memcpy(req_data.get(), static_cast<const uint8_t *>(request_->raw_data(request_->read_cursor())), request_length);
  INFO_MSG_YELLOW("[ClientHandler] client |" << client_id << "| send a request (reached server!)");

  // 打印request消息
  std::string request_str;
  for (int i = 0; i < request_length; i++){
    request_str += std::to_string(req_data.get()[i]) + " ";
  }
  INFO_MSG_YELLOW("[ClientHandler] client |" << client_id << "| request:" << request_str);

  if (service_name_length != service_name.length() || service_name.empty()){
    ROS_ERROR_STREAM("[ClientHandler] Invalid service name length or name: "
                     << service_name_length << " " << service_name);
    return;
  }
  try{
    ros::ServiceClientOptions options(service_name, "*", false, ros::M_string());
    ros::ServiceClient client = ros::NodeHandle().serviceClient(options);
    if (!client.isValid()){
      ROS_ERROR_STREAM("[ClientHandler] Failed to create remote client for service: " << service_name);
      return;
    }
    ros::SerializedMessage request_msg(req_data, request_length);
    ros::SerializedMessage response_msg, response_msg_send;
    topic_tools::ShapeShifter response_shifter, request_shifter;
    zmqpp::message response_send_zmq;

    INFO_MSG_YELLOW("[SrvFactory] 1. try to call service |" << service_name << "|");
    ros::serialization::deserializeMessage(request_msg, request_shifter);
    bool call_flag = client.call(request_shifter, response_shifter, "*");
    INFO_MSG_YELLOW("[SrvFactory] 2. |"<< service_name <<"| call result: (0 failed | 1 success)" << call_flag);

    response_msg_send = ros::serialization::serializeServiceResponse(call_flag, response_shifter);
    response_send_zmq << client_id << response_msg_send.num_bytes;
    response_send_zmq.add_raw(response_msg_send.buf.get(), response_msg_send.num_bytes);

    shared_mutex_.lock();
    socket_->send(response_send_zmq);
    shared_mutex_.unlock();
    INFO_MSG_GREEN("**[ClientHandler] 3. server send a response_shifter to client |"<< client_id << "|");
  }catch (const std::exception& e) {
    ROS_ERROR_STREAM("[ClientHandler] Failed to call service: " << service_name << " error: " << e.what());
    return;
  }
}