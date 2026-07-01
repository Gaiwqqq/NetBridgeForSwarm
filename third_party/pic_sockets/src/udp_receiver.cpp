/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#include "udp.h"
using namespace udp;


#include <iostream>


char* addr_to_ip_string(sockaddr_in addr)
{
    return ip_to_string(addr.sin_addr.s_addr);
}


char* ip_to_string(uint32_t ip_int)
{
    char* ip_str;
    ip_str = new char[16];
    uint32_t ip = ip_int;
    sprintf(ip_str, "%u.%u.%u.%u", ip & 0x000000FF, (ip & 0x0000FF00) >> 8,
            (ip & 0x00FF0000) >> 16, (ip & 0xFF000000) >> 24);
    return ip_str;
}


UDPReceiver::UDPReceiver()
{
    // default parameters
    server_port = 0;

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(server_port);

    que_size = UDP_QUE_SIZE;
    piece_timeout = PIECE_TIMEOUT;
    pack_timeout = PACK_TIMEOUT;

    config = false;
    async = false;
    debug = false;

    recv_handle = false;
    work_handle = false;

    last_success_time = 0;
}

UDPReceiver::~UDPReceiver()
{
    work_handle = false;
    work_thread.join();
    if (!async)
    {
        delete work_que;
    }

    if (debug)
    {
        std::cout << "free work_thread." << std::endl;
    }

    recv_handle = false;
    recv_thread.join();
    delete recv_que;

    if (debug)
    {
        std::cout << "free recv_thread." << std::endl;
    }

    for (auto it = recv_map.begin(); it != recv_map.end(); ++it)
    {
        RecvHead* head = it->second;
        if (head->buffer) delete[] head->buffer;
        head->buffer = NULL;
    }

    if (debug)
    {
        std::cout << "free recv_map." << std::endl;
    }

#ifdef _WIN32
    if (config)
    {
        closesocket(sock);
        WSACleanup();
    }
#elif __linux__ || __APPLE__
    if (config)
    {
        close(sock);
    }
#endif
}

void UDPReceiver::set_listen_port(uint16_t port)
{
    server_port = port;
    if (debug)
    {
        std::cout << "set server port:" << server_port << std::endl;
    }
    server_addr.sin_port = htons(server_port);
    return;
}


void UDPReceiver::set_que_size(uint16_t size)
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the que_size parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    if (async)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "It seems that you have already enabled the async mode, "
                     "so it won't use the que-size in face. "
                     "But the function call will be executed normally." << std::endl;
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


void UDPReceiver::set_piece_timeout(uint16_t ms)
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the piece-timeout parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    piece_timeout = ms;
    if (debug)
    {
        std::cout << "set piece-timeout(ms):" << piece_timeout << std::endl;
    }
    return;
}


void UDPReceiver::set_pack_timeout(uint16_t ms)
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the pack-timeout parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    pack_timeout = ms;
    if (debug)
    {
        std::cout << "set pack-timeout(ms):" << pack_timeout << std::endl;
    }
    return;
}


void UDPReceiver::enable_async(void (*callback)(uint32_t, uint8_t*, RecvInfo))
{
    if (config)
    {
        std::cerr << "warning: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have initialized the instance already, "
                     "and cannot modify the async parameter anymore. "
                     "This function call will be ignored." << std::endl;
        return;
    }
    async = true;

    if (debug)
    {
        std::cout << "enable async mode." << std::endl;
    }

    callback_func = callback;

    return;
}


void UDPReceiver::enable_debug()
{
    std::cout << "enable show debug info." << std::endl;
    debug = true;
    return;
}

bool UDPReceiver::init()
{
    // check
    if (server_port == 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "Please set the correct port!" << std::endl;
        return false;
    }

    // config
#ifdef _WIN32
    int timeout = piece_timeout;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "WSAStartup failed!" << std::endl;
        return false;

    }
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "socket creation failed!" << std::endl;
        WSACleanup();
        return false;
    }
    if (bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "bind failed!" << std::endl;
        goto close_sock;
    }
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) == SOCKET_ERROR)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "Can not set timeout!" << std::endl;
        goto close_sock;
    }
#elif __linux__ || __APPLE__
    struct timeval timeout = {0, piece_timeout * 1000};
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "socket creation failed!" << std::endl;
        return false;
    }
    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)))
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "bind failed!" << std::endl;
        goto close_sock;
    }
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "Can not set timeout!" << std::endl;
        goto close_sock;
    }
#endif

    recv_que = new BlockingQueue<std::vector<uint8_t> >(que_size);
    if (recv_que == NULL)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "Can not create recv_que!" << std::endl;
        goto close_sock;
    }

    if (!async)
    {
        work_que = new BlockingQueue<RecvHead>(que_size);
        if (work_que == NULL)
        {
            std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                         "Can not create work_que!" << std::endl;
            goto close_sock;
        }
    }

    recv_handle = true;
    recv_thread = std::thread(&UDPReceiver::recv_func, this);

    work_handle = true;
    work_thread = std::thread(&UDPReceiver::work_func, this);

    config = true;

    return true;

close_sock:
#ifdef _WIN32
    closesocket(sock);
    WSACleanup();
    return false;
#elif __linux__ || __APPLE__
    close(sock);
    return false;
#endif
}

std::vector<uint8_t> UDPReceiver::recv(RecvInfo& info)
{
    RecvHead recv_head;
    std::vector<uint8_t> res;

    info.src_ip = 0;
    info.src_port = 0;
    info.work_delay_ms = 0;
    info.com_delay_ms = 0;

    if (async)
    {
        std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                     "You have set the instance to async mode, "
                     "and cannot call recv function. "
                     "This function call will be ignored." << std::endl;
        info.flag = FLAG_ERROR;
        res.reserve(0);
        return res;
    }

    recv_head = work_que->pop();
    if (recv_head.flag != FLAG_SUCCESS)
    {
        info.flag = recv_head.flag;
        res.reserve(0);
        return res;
    }
    info.flag = recv_head.flag;
    res = std::vector<uint8_t>(recv_head.buffer, recv_head.buffer + recv_head.size);
    delete recv_head.buffer;
    return res;
}


void UDPReceiver::recv_func()
{
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);

    uint8_t* buffer;
    uint16_t buf_len = len + sizeof(UDPHead) + MAX_PACK_SIZE;
    buffer = new uint8_t[buf_len];

    std::vector<uint8_t> vec_buf;

    while (recv_handle)
    {
        int ret = recvfrom(sock, (char*)buffer + len, buf_len, 0, (struct sockaddr*)&client_addr, &len);
        if (ret < 0)
        {
            vec_buf.clear();
            recv_que->push(vec_buf);
            if (debug)
            {
                std::cout << "[recv_func]recv timeout!" << std::endl;
            }
        }
        else
        {
            memcpy(buffer, &client_addr, len);
            vec_buf = std::vector<uint8_t>(buffer, buffer + len + ret);
            recv_que->push(vec_buf);
            if (debug)
            {
                std::cout << "[recv_func]recv:" << ret << "byte." << std::endl;
            }
        }
    }

    delete[] buffer;
    return;
}

void UDPReceiver::work_func()
{
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);

    UDPHead udp_head;
    uint8_t* buffer;
    std::vector<uint8_t> buffer_vec;
    RecvHead recv_head;

    std::vector<uint8_t> vec_buf;
    long long last_timestamp, timestamp;

    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    last_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    last_success_time = last_timestamp;

    while (work_handle)
    {
        now = std::chrono::system_clock::now();
        duration = now.time_since_epoch();
        last_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

        vec_buf = recv_que->pop();

        now = std::chrono::system_clock::now();
        duration = now.time_since_epoch();
        timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        if (vec_buf.size() <= 0)
        {   // 超过piece_timeout没有收到分片
            if (debug)
            {
                std::cout << "[work_func]pop void vector." << std::endl;
            }
            if (timestamp - last_timestamp >= pack_timeout * 1000
            || timestamp - last_success_time >= pack_timeout * 1000)
            {   // 超过pack_timeout，交付超时包裹
                memset(&recv_head, 0, sizeof(recv_head));
                recv_head.flag = FLAG_TIMEOUT;
                delivery_pack(recv_head);
                last_success_time = timestamp;
            }
            else
            {   // 没有超过pack_timeout，忽略，去处理下map
                goto process;
            }
        }
        else
        {   // 收到新的分片
            bool buffer_erased = false;
//            buffer_vec = std::vector<uint8_t>(vec_buf.begin(), vec_buf.end());
            buffer = new uint8_t[vec_buf.size() - len - sizeof(UDPHead)];


            std::copy(vec_buf.begin(), vec_buf.begin() + len, (char*)&client_addr);
            std::copy(vec_buf.begin() + len, vec_buf.begin() + len + sizeof(UDPHead), (char*)&udp_head);
//            std::copy(vec_buf.begin() + len + sizeof(UDPHead), vec_buf.end(), buffer_vec.begin());
            std::copy(vec_buf.begin() + len + sizeof(UDPHead), vec_buf.end(), (char*)buffer);

            if (debug)
            {
                char *ip = addr_to_ip_string(client_addr);
                printf("[work_func]recv_ip:%s pack_id:%u piece_num:%u "
                       "piece_total:%u piece_size:%u pack_size:%u\n",
                       ip, udp_head.pack_id, udp_head.piece_num, udp_head.piece_total,
                       udp_head.piece_size, udp_head.pack_size);
                delete[] ip;
            }

            // 计算分片隶属包裹的id，由ip地址后16位和id组成
            uint32_t key = 0;
            key |= (udp_head.pack_id) & 0x0000FFFF;
            key |= (client_addr.sin_addr.s_addr & 0x0000FFFF) << 16;

            RecvHead *head;

            auto it = recv_map.find(key);
            if (it != recv_map.end())
            {   // 分片隶属于已知包裹
                head = it->second;
                if (head->size != udp_head.pack_size || head->total != udp_head.piece_total)
                {   // 接收出错
                    std::cerr << "Error: [" << __FILE__ << "," << __LINE__ << "]"
                                 "Recv Error!" << std::endl;
                    delete[] head->buffer;
                    head->buffer = NULL;
                    head->flag = FLAG_ERROR;
                    head->num = 0;
                    head->total = 0;
                    head->size = 0;
                    recv_head = *(head);
                    // 交付错误包裹，由map中清除id
                    delivery_pack(recv_head);
                    auto it = recv_map.find(key);
                    recv_map.erase(it);
                }
                else if (head->timestamp < last_success_time)
                {   // 该包裹初始接收时间，晚于上次已经交付的包裹的初始接收时间（影响实时性）
                    delete[] buffer;
                    buffer_erased = true;
//                    buffer_vec.clear();
                    // 忽略该包裹
                    auto it = recv_map.find(key);
                    recv_map.erase(it);
                }
                else
                {   // 向包裹添加新分片
//                    memcpy(head->buffer + (udp_head.piece_num - 1) * head->base, buffer_vec.data(), udp_head.piece_size);
                    memcpy(head->buffer + (udp_head.piece_num - 1) * head->base, buffer, udp_head.piece_size);
                    head->num += 1;
                    if (head->num == head->total)
                    {   // 包裹所有分片均接收到，交付
                        recv_head = *(head);
                        delivery_pack(recv_head);
                        auto it = recv_map.find(key);
                        recv_map.erase(it);
                        last_success_time = timestamp;
                    }
                }
            }
            else
            {   // 分片隶属于新包裹
                head = new RecvHead;
                head->src_ip = client_addr.sin_addr.s_addr;
                head->src_port = client_addr.sin_port;

                head->timestamp = timestamp;
                head->delay = timestamp - udp_head.timestamp;

                head->size = udp_head.pack_size;
                head->num = 1;
                head->total = udp_head.piece_total;

                head->flag = FLAG_SUCCESS;
                head->buffer = new uint8_t[head->size];

                if (udp_head.piece_total == 1)
                {   // 可以直接交付了
                    head->base = head->size;
//                    memcpy(head->buffer, buffer_vec.data(), udp_head.pack_size);
                    memcpy(head->buffer, buffer, udp_head.pack_size);
                    recv_head = *(head);
                    delivery_pack(recv_head);
                }
                else
                {   // 建立新包裹
                    if (udp_head.piece_num == udp_head.piece_total)
                    {   // 先收到了最后一个分片
                        head->base = (udp_head.pack_size - udp_head.piece_size) / (udp_head.piece_total - 1);
                    }
                    else
                    {   // 先收到了前面的分片
                        head->base = udp_head.piece_size;
                    }
//                    memcpy(head->buffer, buffer_vec.data(), head->base);
                    memcpy(head->buffer + (udp_head.piece_num - 1) * head->base, buffer, udp_head.piece_size);
                    recv_map.insert(std::make_pair(key, head));
                }
            }
            if (!buffer_erased) delete[] buffer;
        }

    process:
        // 遍历处理map中的包裹
        std::vector<std::map<uint32_t, RecvHead*>::iterator> toErase;
        toErase.clear();
        for (auto it = recv_map.begin(); it != recv_map.end(); ++it)
        {
            RecvHead* head = it->second;

            if (timestamp - head->timestamp > PACK_TIMEOUT * 1000)
            {   // 超时，认为丢包，交付。
                head->flag = FLAG_LOST;
                delete[] head->buffer;
                head->buffer = NULL;
                recv_head = *(head);
                delivery_pack(recv_head);
                toErase.push_back(it);
            }
        }
        for(auto it : toErase)
        {
            recv_map.erase(it);
        }
    }
}

void UDPReceiver::delivery_pack(RecvHead recv_head)
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    long long now_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

    RecvInfo info;
    info.flag = recv_head.flag;
    info.src_ip = recv_head.src_ip;
    info.src_port = recv_head.src_port;
    info.work_delay_ms = (now_timestamp - recv_head.timestamp) / 1000.0;
    info.com_delay_ms = recv_head.delay / 1000.0;

    if (debug)
    {
        std::cout << "[delivery_pack]Got pack. Flag:" << (int)recv_head.flag << " Size:" << recv_head.size
                  << " work_delay:" << info.work_delay_ms << "ms"
                  << " com_delay:" << info.com_delay_ms << "ms" << std::endl;
    }

    if (async)
    {
        callback_func(recv_head.size, recv_head.buffer, info);
        if (recv_head.buffer) delete[] recv_head.buffer;
    }
    else
    {
        work_que->push(recv_head);
    }
}

