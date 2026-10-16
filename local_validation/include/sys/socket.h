#pragma once
#include <sys/types.h>
#include <cstddef>
typedef unsigned socklen_t;
struct sockaddr { unsigned short sa_family; char data[14]; };
#define AF_CAN 29
#define SOCK_RAW 3
#define SOL_SOCKET 1
#define SO_SNDBUF 7
#define MSG_DONTWAIT 0x40
#define MSG_NOSIGNAL 0x4000
extern "C" {
int socket(int,int,int);
int bind(int,const sockaddr*,socklen_t);
int setsockopt(int,int,int,const void*,socklen_t);
ssize_t send(int,const void*,size_t,int);
ssize_t recv(int,void*,size_t,int);
}
