/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#include <assert.h>
#include "pic_socket.h"

UDPImgSender::UDPImgSender()
{
   udp_sender = NULL;
   config = false;
   ip[0] = 0;
   port = 0;
   mode = 0;
}

UDPImgSender::UDPImgSender(const char* ip, uint16_t port, int mode)
{
   udp_sender = NULL;
   config = false;
   this->port = 0;
   this->mode = 0;
   set_target_ip(ip);
   set_target_port(port);
   set_mode(mode);
   assert(init());
}

UDPImgSender::~UDPImgSender()
{
   if (udp_sender) delete udp_sender;
}

void UDPImgSender::set_mode(int mode)
{
   if (config)
   {
      std::cout << "You have already init UDPImgSender." << std::endl;
      return;
   }
   this->mode = mode;
}

void UDPImgSender::set_target_ip(const char* ip)
{
   if (config)
   {
      std::cout << "You have already init UDPImgSender." << std::endl;
      return;
   }
   strcpy(this->ip, ip);
}

void UDPImgSender::set_target_port(uint16_t port)
{
   if (config)
   {
      std::cout << "You have already init UDPImgSender." << std::endl;
      return;
   }
   this->port = port;
}

bool UDPImgSender::init()
{
   if (config)
   {
      std::cout << "You have already init UDPImgSender." << std::endl;
      return false;
   }
   if (strlen(ip) <= 0) return false;
   if (port == 0) return false;
   udp_sender = new udp::UDPSender();
   udp_sender->set_target_ip(ip);
   udp_sender->set_target_port(port);
   udp_sender->set_pack_size(4096);
   if (!udp_sender->init())
   {
      delete udp_sender;
      udp_sender = NULL;
      return false;
   }
   config = true;
   return true;
}

void UDPImgSender::send(cv::Mat img)
{
   if (!config)
   {
      std::cout << "You have not init the UDPImgSender." << std::endl;
      return;
   }
   if (mode != MODE_CV)
   {
      std::cout << "You should send JPG-buffer." << std::endl;
      return;
   }
   std::vector<uint8_t> buffer;
   MatToJpgBuffer(img, buffer, 80);
   udp_sender->send((uint8_t*)&buffer[0], buffer.size());
}

void UDPImgSender::sendjpg(uint8_t* jpg_buffer, size_t size)
{
   if (!config)
   {
      std::cout << "You have not init the UDPImgSender." << std::endl;
      return;
   }
   if (mode != MODE_JPGBUF)
   {
      std::cout << "You should send cv::Mat." << std::endl;
      return;
   }
   udp_sender->send(jpg_buffer, size);
}

void UDPImgSender::sendrgb(uint8_t* rgb_buffer, size_t width, size_t height, int quality)
{
   if (!config)
   {
      std::cout << "You have not init the UDPImgSender." << std::endl;
      return;
   }
   if (mode != MODE_RGBBUF)
   {
      std::cout << "You should send cv::Mat." << std::endl;
      return;
   }
   std::vector<uint8_t> jpg_buffer = RgbBufferToJpgBuffer(rgb_buffer, width, height, quality);
   udp_sender->send(&jpg_buffer[0], jpg_buffer.size());
}

UDPImgReceiver::UDPImgReceiver()
{
   udp_receiver = NULL;
   cv_que = NULL;
   jpg_que = NULL;
   rgb_que = NULL;
   port = 0;
   recv_handle = false;
   config = false;
   mode = 0;
}

UDPImgReceiver::UDPImgReceiver(uint16_t port, int mode)
{
   udp_receiver = NULL;
   cv_que = NULL;
   jpg_que = NULL;
   rgb_que = NULL;
   this->port = 0;
   recv_handle = false;
   config = false;
   this->mode = 0;
   set_listen_port(port);
   set_mode(mode);
   assert(init());
}

UDPImgReceiver::~UDPImgReceiver()
{
   if (config)
   {
      recv_handle = false;
      recv_thread.join();
   }
   if (udp_receiver) delete udp_receiver;
   if (cv_que) delete cv_que;
   if (jpg_que) delete jpg_que;
   if (rgb_que) delete rgb_que;
}

void UDPImgReceiver::set_mode(int mode)
{
   if (config)
   {
      std::cout << "You have already init UDPImgReceiver." << std::endl;
      return;
   }
   this->mode = mode;
}

void UDPImgReceiver::set_listen_port(uint16_t port)
{
   if (config)
   {
      std::cout << "You have already init UDPImgReceiver." << std::endl;
      return;
   }
   this->port = port;
}

void UDPImgReceiver::recv_func()
{
   std::vector<uint8_t> buffer;
   udp::RecvInfo info;
   while (recv_handle)
   {
      buffer = udp_receiver->recv(info);
      if (info.flag == FLAG_SUCCESS)
      {
         if (mode == MODE_CV)
         {
            cv::Mat img;
            JpgBufferToMat((uint8_t*)&buffer[0], buffer.size(), img);
            if (cv_que->size() >= 3) cv_que->pop();
            cv_que->push(img);
         }
         else if (mode == MODE_JPGBUF)
         {
            if (jpg_que->size() >= 3) jpg_que->pop();
            jpg_que->push(buffer);
         }
         else if (mode == MODE_RGBBUF)
         {
            RgbImg rgb = JpgBufferToRgbBuffer(&buffer[0], buffer.size());
            if (rgb_que->size() >= 3) rgb_que->pop();
            rgb_que->push(rgb);
         }
      }
   }
}

bool UDPImgReceiver::init()
{
   if (config) 
   {
      std::cout << "You have already init UDPImgReceiver." << std::endl;
      return false;
   }
   if (port == 0) return false;
   udp_receiver = new udp::UDPReceiver();
   udp_receiver->set_listen_port(port);
   if (!udp_receiver->init())
   {
      delete udp_receiver;
      udp_receiver = NULL;
      return false;
   }
   if (mode == MODE_CV)
   {
      cv_que = new BlockingQueue<cv::Mat>(5);
   }
   else if (mode == MODE_JPGBUF)
   {
      jpg_que = new BlockingQueue<std::vector<uint8_t> >(5);
   }
   else if (mode == MODE_RGBBUF)
   {
      rgb_que = new BlockingQueue<RgbImg>(5);
   }
   recv_handle = true;
   recv_thread = std::thread(&UDPImgReceiver::recv_func, this);
   config = true;
   return true;
}

cv::Mat UDPImgReceiver::read()
{
   if (!config)
   {
      std::cout << "You have not init the UDPImgReceiver." << std::endl;
      cv::Mat empty_mat;
      return empty_mat;
   }
   if (mode != MODE_CV)
   {
      std::cout << "You should read JPG-buffer." << std::endl;
      cv::Mat empty_mat;
      return empty_mat;
   }
   if (cv_que->empty()) {
      return cv::Mat();
   }
   else {
      return cv_que->pop();
   }
}

std::vector<uint8_t> UDPImgReceiver::readjpg()
{
   if (!config)
   {
      std::cout << "You have not init the UDPImgReceiver." << std::endl;
      std::vector<uint8_t> empty_vector;
      return empty_vector;
   }
   if (mode != MODE_JPGBUF)
   {
      std::cout << "You should read cv::Mat." << std::endl;
      std::vector<uint8_t> empty_vector;
      return empty_vector;
   }
   return jpg_que->pop();
}

RgbImg UDPImgReceiver::readrgb()
{
   if (!config)
   {
      std::cout << "You have not init the UDPImgReceiver." << std::endl;
      RgbImg empty_img;
      return empty_img;
   }
   if (mode != MODE_RGBBUF)
   {
      std::cout << "You should read cv::Mat." << std::endl;
      RgbImg empty_img;
      return empty_img;
   }
   return rgb_que->pop();
}