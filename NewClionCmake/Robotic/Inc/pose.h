#ifndef __POSE_H
#define __POSE_H

#include "icm20948.h"
// 四元数
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

// 欧拉角
typedef struct {
    float roll;     // 横滚角
    float pitch;    // 俯仰角
    float yaw;      // 航向角
} EulerAngles;

// Mahony滤波器状态
typedef struct {
    float Kp;         // 加速度计比例系数
    float Ki;         // 加速度计积分系数
    float Kp_mag;     // 磁力计比例系数
    float Ki_mag;     // 磁力计积分系数
    float integralFBx;
    float integralFBy;
    float integralFBz;
} MahonyFilter;

void Pose_MahonyUpdate(MahonyFilter *filter, float dt, Quaternion* q,
                       ImuReal* accData, ImuReal* gyroData);
void Pose_MahonyUpdate2(MahonyFilter *filter, float dt, Quaternion* q,
                        ImuReal* accData, ImuReal* gyroData, ImuReal* magData);
void Pose_QuatToEuler(const Quaternion* q, EulerAngles* euler);

#endif
