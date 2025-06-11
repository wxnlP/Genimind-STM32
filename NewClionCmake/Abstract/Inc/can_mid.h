#ifndef __CAN_MID_H
#define __CAN_MID_H

#include "can_fd.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "pose.h"

typedef enum {
    // 线速度 8字节
    CMD_LINEAR = 0x511,
    // 角速度 4字节
    CMD_ANGULAR,
    // 蜂鸣器和LED
    CMD_BUZZER,
} Ros2Cmd;

/* 里程计 */
typedef struct {
    float v_tx;
    float v_ty;
    float omega;
} Odom;

typedef struct {
    uint8_t led;
    uint8_t buzzer;
} BuzzerLed;

void CanMid_SendQuat(Quaternion* q);
void CanMid_SendImu(ImuReal* accelData, ImuReal* gyroData);
void CanMid_SendVoltage(float voltage);
void CanMid_SendOdom(float v_tx, float v_ty, float omega);
void CanMid_BytesToFloat(const uint8_t* bytes, float* data);
void CanMid_Init(void);
void CanMid_WriteOdom(Odom* data);
void CanMid_WriteBuzzerLed(BuzzerLed* data);
void CanMid_ReadOdom(Odom* data);
BaseType_t CanMid_ReadBuzzerLed(BuzzerLed* data);

#endif
