#ifndef __KINEMATICS_H
#define __KINEMATICS_H


#define ROBOT_R_X               0.11f       // 底盘X轴上两轮中心的间距的一半,单位：米
#define ROBOT_R_Y               0.123f      // 底盘Y轴上两轮中心的间距的一半,单位：米
#define WHEEL_RADIUS            0.0375f     // 轮子半径37.5mm,单位：米
#define WHEEL_GEAR_RATIO        28.0f       // 电机减速比
#define SAMPLING_TIME           0.01f       // 编码器采样时间10ms (单位：秒)
#define MAX_PWM_DUTY            100.0f      // PWM最大占空比值
#define ENCODER_PPR             52          // 输出轴每转脉冲数 (脉冲/转)，13ppr*4倍频=13*4
#define ENCODER_MAX             1300.0f

typedef enum {
    BM_FORWARD,        // 前进
    BM_RETREAT,        // 后退
    BM_SHIFT_RIGHT,    // 右移
    BM_SHIFT_LEFT,     // 左移
    BM_ROTATE_RIGHT,   // 右旋
    BM_ROTATE_LEFT,    // 左旋
    BM_RIGHT_FRONT,    // 右前
    BM_LEFT_BACK,      // 左后
    BM_LEFT_FRONT,     // 左前
    BM_RIGHT_BACK,     // 右后
} BasicMotion;

// 测试案例枚举
typedef enum {
    TEST_CASE_FORWARD,     // 纯平移测试
    TEST_CASE_ROTATION,    // 纯旋转测试
    TEST_CASE_DIAGONAL,    // 斜向运动测试
    TEST_CASE_COMBINED     // 平移+旋转复合测试
} TestCaseType;



void Kinematics_BasicMotion(float speed, BasicMotion motion);
void Kinematics_Inverse(float v_tx, float v_ty, float omega, float* target_rpm);
void Kinematics_RpmToDuty(const float* rpm, float* duty);
void Kinematics_CntToRpm(const float *cnt, float *rpm);
void Kinematics_Position(const float *actual_rpm, float *v_tx, float *v_ty, float *omega);
#endif
