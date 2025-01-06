/*
 * Copyright (C) 2024 Coder.AN
 * Email: an.hongjun@foxmail.com
 * Page: www.anhongjun.top
 */
#ifndef __NETWORK_H__
#define __NETWORK_H__


#include <sys/socket.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h> 
#include <unistd.h> 
#include <netdb.h>  
#include <net/if.h>  
#include <arpa/inet.h> 
#include <sys/ioctl.h>  
#include <sys/types.h>  
#include <sys/time.h> 

#define ip_to_str(str, ip) \
    sprintf(str, "%u.%u.%u.%u", ip & 0x000000FF, (ip & 0x0000FF00) >> 8, (ip & 0x00FF0000) >> 16, (ip & 0xFF000000) >> 24)

uint32_t get_local_ip(const char *eth_inf);

#endif