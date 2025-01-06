//
// Created by gwq on 1/4/25.
//

#ifndef SRC_SERVICE_FASTORY_H
#define SRC_SERVICE_FASTORY_H

#include <ros/ros.h>
#include <ros/names.h>
#include <ros/package.h>
#include <ros/message.h>
#include <topic_tools/shape_shifter.h>
#include <msgs_macro.hpp>
#include <zmqpp/zmqpp.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <iostream>
#include <shared_mutex>


class ServiceFactory;
class ServiceCallbackHelper;
class ClientHandler;

struct ServiceConfig {
  std::string service_name;
  std::string service_prefix_name;
  std::string service_type;
  std::string server_ip;
  std::string my_hostname;   // only used for client
  std::string port;
  bool        if_prefix;
  ServiceConfig() : if_prefix(false) {};
};

// override ros::ServiceCallbackHelper call function to support remote call
class ServiceCallbackHelper : public ros::ServiceCallbackHelper {
public:
  ServiceCallbackHelper(std::string service_name, std::string service_name_raw,ServiceFactory* client)
      : m_name_(std::move(service_name)), m_name_raw_(std::move(service_name_raw)), m_client_(client) {};
  bool call(ros::ServiceCallbackHelperCallParams& params) override;
private:
  std::string m_name_;
  std::string m_name_raw_;
  ServiceFactory* m_client_;
};


class ClientHandler{
public:
  typedef std::shared_ptr<ClientHandler> Ptr;
  ClientHandler(zmqpp::message request, std::shared_ptr<zmqpp::socket> &socket);
  ~ClientHandler() = default;
private:
  bool close_thread_flag_{false};
  void run();
  std::unique_ptr<zmqpp::message> request_;
  std::shared_ptr<zmqpp::socket>  socket_;
  std::unique_ptr<boost::thread>  thread_;
  std::shared_mutex               shared_mutex_;
};

// handle service request and response
class ServiceFactory {
public:
#define TRIED_TIMES_MAX 10
  enum ClientOrServer {
    CLIENT,
    SERVER
  };
  typedef std::shared_ptr<ServiceFactory> Ptr;
  ServiceFactory(ClientOrServer client_or_server, std::shared_ptr<ros::NodeHandle> nh, const ServiceConfig& config);
  ~ServiceFactory() = default;

  bool call(const std::string& service_name, ros::ServiceCallbackHelperCallParams& params) const;
  void stopServerThread();
  void stopClientThread();
private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_mutex                shared_mutex_;
  ros::ServiceServer               server_handle_client_msg_; // server handle client request, send it to remote server
  ClientOrServer                   client_or_server_;
  ServiceConfig                    config_;
  zmqpp::context_t                 context_;
  std::shared_ptr<zmqpp::socket>   server_socket_;
  std::unique_ptr<boost::thread>   server_thread_;
  std::vector<ClientHandler::Ptr>  client_handlers_;

  bool                             server_shutdown_flag_{true};

  static bool typeExists(const std::string & type_in);
  static std::string getServiceMd5(const std::string& type_in);
  template<typename T>
  static std::string getServiceMsgMd5(const std::string &type_in);

  void step();
};

#endif //SRC_SERVICE_FASTORY_H
