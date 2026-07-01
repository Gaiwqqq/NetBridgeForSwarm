/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#include "udp.h"
using namespace udp;


#include <iostream>


UDPSender::UDPSender()
{
    // default parameters
    strcpy(target_ip, "0.0.0.0");
    target_port = 0;

    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_addr.s_addr = inet_addr(target_ip);
    target_addr.sin_port = htons(target_port);

    que_size = UDP_QUE_SIZE;
    pack_size = UDP_PACK_SIZE;
    broadcast = false;

    config = false;
    debug = false;

    work_handle = false;
}

UDPSender::~UDPSender()
{
    std::vector<uint8_t> vec;
    vec.reserve(0);

    work_handle = false;
    que->push(vec);
    work_thread.join();

    delete que;
#ifdef _WIN32
    closesocket(sock);
    WSACleanup();
#elif __linux__ || __APPLE__
    close(sock);
#endif
}

void UDPSender::set_target_port(uint16_t port)
{
    target_port = port;
    if (debug)
    {
        std::cout << "set target port:" << target_port << std::endl;
    }
    target_addr.sin_port = htons(target_port);
    return;
}

void UDPSender::set_target_ip(const char *ip)
{
    strcpy(target_ip, ip);
    if (debug)
    {
        std::cout << "set target ip:" << target_ip << std::endl;
    }
    target_addr.sin_addr.s_addr = inet_addr(target_ip);
    return;
}

void UDPSender::set_que_size(uint16_t size)
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the que_size parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    if (size < MIN_QUE_SIZE)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "The que_size you want to set is too small, "
                     "it will set to " << MIN_QUE_SIZE << " by default. " << std::endl;
        size = MIN_QUE_SIZE;
    }
    que_size = size;
    if (debug)
    {
        std::cout << "set que_size:" << que_size << std::endl;
    }
    return;
}

void UDPSender::set_pack_size(uint16_t size)
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the pack_size parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    if (size < MIN_PACK_SIZE)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "The pack_size you want to set is too small, "
                     "it will set to 64 by default. " << std::endl;
        size = MIN_PACK_SIZE;
    }
    if (size > MAX_PACK_SIZE)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "The pack_size you want to set is too large, "
                     "it will set to 4096 by default. " << std::endl;
        size = MAX_PACK_SIZE;
    }
    pack_size = size;
    if (debug)
    {
        std::cout << "set pack_size:" << que_size << std::endl;
    }
    return;
}

void UDPSender::enable_broadcast()
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the boardcast parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    broadcast = true;

    if (debug)
    {
        std::cout << "enable broadcast." << std::endl;
    }

    return;
}

void UDPSender::enable_debug()
{
    std::cout << "enable show debug info." << std::endl;
    debug = true;
    return;
}

bool UDPSender::init()
{
    // check
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot do that repeatedly anymore. " << std::endl;
        return false;
    }
    if (strcmp(target_ip, "0.0.0.0") == 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "Please set the target IP!" << std::endl;
        return false;
    }
    if (target_port == 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "Please set the correct port!" << std::endl;
        return false;
    }

    // config
#ifdef _WIN32
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "WSAStartup failed!" << std::endl;
        return false;
    }
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
#elif __linux__ || __APPLE__
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
#endif
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "socket creation failed!" << std::endl;
        return false;
    }

    if (broadcast)
    {
    #ifdef _WIN32
        int optval = 1;
        int ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char*)&optval, sizeof(int));
        if (ret < 0)
        {
            std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                         "Can not set sockeopt!" << std::endl;
            return false;
        }
    #elif __linux__ || __APPLE__
        int optval = 1;
        int ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST | SO_REUSEADDR, &optval, sizeof(int));
        if (ret < 0)
        {
            std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                         "Can not set sockeopt!" << std::endl;
            return false;
        }
    #endif
    }

    que = new BlockingQueue<std::vector<uint8_t> >(que_size);
    work_handle = true;
    work_thread = std::thread(&UDPSender::work, this);

    config = true;

    return true;
}

void UDPSender::send(uint8_t* buf, uint32_t size)
{
    if (!config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have not initialized the instance, "
                     "the package will be stored into the blocking queue."
                     "It won't be send out until you initialize the instance." << std::endl;
        return;
    }
    std::vector<uint8_t> vec(buf, buf + size);
    que->push(vec);
}

void UDPSender::work()
{
    int ret;

    uint8_t* vec_buf;

    uint8_t* send_buf;
    uint16_t buf_len = sizeof(UDPHead) + pack_size;
    send_buf = new uint8_t[buf_len];

    UDPHead* head = (UDPHead*)send_buf;
    uint8_t* buf = send_buf + sizeof(UDPHead);

    static uint16_t id = 0;

    while (work_handle)
    {
        id += 1;
        std::vector<uint8_t> vec = que->pop();
        if (!work_handle)
        {
            break;
        }

        vec_buf = new uint8_t[vec.size()];
        std::copy(vec.begin(), vec.end(), vec_buf);

        uint16_t pack_mod = vec.size() % pack_size;
        uint16_t pack_num = vec.size() / pack_size;

        head->piece_total = pack_mod ? pack_num + 1 : pack_num;
        head->pack_size = vec.size();
        if (debug)
        {
            std::cout << "size=" << vec.size() << " pack_mod=" << pack_mod << " pack_num=" << pack_num << std::endl;
        }

        uint8_t* dst_p;
        uint8_t* src_p;

        for (int i = 0; i < pack_num; i++)
        {
            dst_p = buf;
            src_p = vec_buf + i * pack_size;
            memcpy(dst_p, src_p, pack_size);

            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            head->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            head->pack_id = id;
            head->piece_num = i + 1;
            head->piece_size = pack_size;

            if (debug)
            {
                std::cout << "SEND[" << head->pack_id << "]" << head->piece_num << "/" << head->piece_total << std::endl;
            }
        #ifdef _WIN32
            ret = sendto(sock, (const char*)send_buf, buf_len, 0, (SOCKADDR*)&target_addr, sizeof(target_addr));
        #elif __linux__ || __APPLE__
            ret = sendto(sock, send_buf, buf_len, 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
        #endif
            if (ret < 0)
            {
                std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                             "Failed to send UDP packet!" << std::endl;
            }
        }

        if (pack_mod)
        {
            dst_p = buf;
            src_p = vec_buf + pack_num * pack_size;
            memcpy(dst_p, src_p, pack_mod);

            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            head->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            head->pack_id = id;
            head->piece_num = pack_num + 1;
            head->piece_size = pack_mod;

            if (debug)
            {
                std::cout << "SEND[" << head->pack_id << "]" << head->piece_num << "/" << head->piece_total << std::endl;
            }

        #ifdef _WIN32
            ret = sendto(sock, (const char*)send_buf, sizeof(UDPHead) + pack_mod, 0, (sockaddr*)&target_addr, sizeof(target_addr));
        #elif __linux__ || __APPLE__
            ret = sendto(sock, send_buf, sizeof(UDPHead) + pack_mod, 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
        #endif
            if (ret < 0)
            {
                std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                             "Failed to send UDP packet!" << std::endl;
            }
        }

        delete[] vec_buf;
    }
}
