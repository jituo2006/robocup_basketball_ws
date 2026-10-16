#pragma once
#define POLLIN 1
#define POLLERR 8
#define POLLHUP 16
#define POLLNVAL 32
struct pollfd { int fd; short events, revents; };
typedef unsigned long nfds_t;
extern "C" int poll(pollfd*,nfds_t,int);
