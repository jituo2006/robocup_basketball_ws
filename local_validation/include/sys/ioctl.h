#pragma once
#define SIOCGIFINDEX 0x8933UL
extern "C" int ioctl(int,unsigned long,...);
