#pragma once
#include <cstdint>
typedef uint32_t canid_t;
typedef uint32_t can_err_mask_t;
#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
#define CAN_SFF_MASK 0x7FFU
#define CAN_EFF_MASK 0x1FFFFFFFU
#define CAN_ERR_MASK 0x1FFFFFFFU
struct can_frame { canid_t can_id; uint8_t can_dlc, pad[3]; alignas(8) uint8_t data[8]; };
struct can_filter { canid_t can_id, can_mask; };
struct sockaddr_can { unsigned short can_family; int can_ifindex; unsigned char unused[8]; };
