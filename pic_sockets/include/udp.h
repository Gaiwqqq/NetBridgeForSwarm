/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#ifndef UDP_H
#define UDP_H

#include "blocking_queue.h"
#include "network.h"

#include <map>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <cstdint>
#include <cstring>

#ifdef _WIN32
    #include <WinSock2.h>
    #include <ws2tcpip.h>
#elif __linux__ || __APPLE__
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif


char* addr_to_ip_string(sockaddr_in addr);
char* ip_to_string(uint32_t ip_int);

namespace udp
{

#define UDP_PACK_SIZE   512
#define UDP_QUE_SIZE    1000
#define PACK_TIMEOUT    200  // ms
#define PIECE_TIMEOUT   50   // ms

#define MIN_PACK_SIZE   64
#define MAX_PACK_SIZE   4096
#define MIN_QUE_SIZE    50


#define FLAG_SUCCESS 0
#define FLAG_TIMEOUT 1
#define FLAG_LOST    2
#define FLAG_ERROR   3


struct UDPHead
{
    long long   timestamp;
    uint16_t    pack_id;
    uint16_t    piece_num;
    uint16_t    piece_total;
    uint16_t    piece_size;
    uint32_t    pack_size;
};


struct RecvHead
{
    uint32_t    src_ip;
    uint16_t    src_port;

    long long   timestamp;
    long long   delay;

    uint16_t    base;

    uint16_t    num;
    uint16_t    total;
    uint32_t    size;

    uint8_t     flag;
    uint8_t*    buffer;
};


struct RecvInfo
{
    uint8_t flag;

    uint32_t src_ip;
    uint16_t src_port;

    double work_delay_ms;
    double com_delay_ms;
};


class UDPSender
{
public:
    UDPSender();
    ~UDPSender();

    void set_target_port(uint16_t port);
    void set_target_ip(const char *ip);
    void set_que_size(uint16_t size);
    void set_pack_size(uint16_t size);
    void enable_broadcast();
    bool init();

    void enable_debug();

    void send(uint8_t* buf, uint32_t size);

private:
#ifdef _WIN32
    WSADATA         wsaData;
    SOCKET          sock;
#elif __linux__ || __APPLE__
    int             sock;
#endif
    char            target_ip[16];
    uint16_t        target_port;
    sockaddr_in     target_addr;
    uint16_t        que_size;
    uint16_t        pack_size;
    bool            broadcast;

    bool            config;
    bool            debug;

    BlockingQueue<std::vector<uint8_t> >*  que;

    bool            work_handle;
    std::thread     work_thread;
    void            work();
};


class UDPReceiver
{
public:
    UDPReceiver();
    ~UDPReceiver();

    void set_listen_port(uint16_t port);
    void set_que_size(uint16_t size);
    void set_piece_timeout(uint16_t ms);
    void set_pack_timeout(uint16_t ms);
    void enable_async(void (*callback)(uint32_t, uint8_t*, RecvInfo));
    bool init();

    void enable_debug();

    std::vector<uint8_t> recv(RecvInfo& info);

private:
#ifdef _WIN32
    WSADATA         wsaData;
    SOCKET          sock;
#elif __linux__ || __APPLE__
    int             sock;
#endif
    uint16_t        server_port;
    sockaddr_in     server_addr;
    uint16_t        que_size;
    uint16_t        piece_timeout;
    uint16_t        pack_timeout;

    bool            config;
    bool            async;
    bool            debug;

    std::map<uint32_t, RecvHead*> recv_map;

    BlockingQueue<std::vector<uint8_t> >* recv_que;
    bool            recv_handle;
    std::thread     recv_thread;
    void            recv_func();

    BlockingQueue<RecvHead>* work_que;
    bool            work_handle;
    std::thread     work_thread;
    void            work_func();

    long long last_success_time;
    void delivery_pack(RecvHead recv_head);
    void (*callback_func) (uint32_t, uint8_t*, RecvInfo);
};

}

#endif // UDP_H

