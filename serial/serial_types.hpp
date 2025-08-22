/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-08-22 11:17:29
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2025-08-22 12:53:03
 * @FilePath: /mas_vision_new/serial/serial_types.hpp
 * @Description: 
 */
#ifndef _SERIAL_TYPES_H_
#define _SERIAL_TYPES_H_

#include <cstdint>
#include <chrono>

struct __attribute__((packed)) ReceivePacket {
    uint8_t header=0xAA;
    uint8_t mode;
    float yaw;
    float pitch;
    float roll;
    uint8_t tail=0X5A;
};

struct __attribute__((packed)) SendPacket {
    uint8_t header=0XBB;
    float target_yaw;
    float target_pitch;
    float distance;
    uint8_t fire_advice;
    uint8_t tail=0X5B;
};

struct ReceivedDataMsg {
    double yaw;
    double pitch;
    double roll;
    uint8_t mode;
    std::chrono::steady_clock::time_point timestamp;
};


enum VisionMode {
  AUTO_AIM_RED = 0,
  AUTO_AIM_BLUE = 1,
  SMALL_RUNE_RED = 2,
  SMALL_RUNE_BLUE = 3,
  BIG_RUNE_RED = 4,
  BIG_RUNE_BLUE = 5,
};

#endif // _SERIAL_TYPES_H_