#include "kinematics.h"
#include "pid.h"
#include <math.h>

// 麦克纳姆轮基础运动方向
static float g_basicMotions[10][4] = {
        {1.0f,  1.0f,  1.0f,  1.0f},
        {-1.0f, -1.0f, -1.0f, -1.0f},
        {1.0f,  -1.0f, -1.0f, 1.0f},
        {-1.0f, 1.0f,  1.0f,  -1.0f},
        {1.0f,  1.0f,  -1.0f, -1.0f},
        {-1.0f, -1.0f, 1.0f,  1.0f},
        {1.0f,  0.0f,  0.0f,  1.0f},
        {-1.0f, 0.0f,  0.0f,  -1.0f},
        {0.0f,  1.0f,  1.0f,  0.0f},
        {0.0f,  -1.0f, -1.0f, 0.0f},
};

/**
 * @brief 麦克纳姆轮基础运动方式，含10种基础的运动方式
 *
 * @param speed 单个电机单位的速度
 * @param motion 运动方式 @BasicMotion
 * @retval 无
 * */
void Kinematics_BasicMotion(float speed, BasicMotion motion) {
    float target[4] = {0};
    for (int i = 0; i < 4; i++) {
        target[i] = speed * g_basicMotions[motion][i];
    }
    PID_SetTarget(target);
}

/**
 * @brief 逆运动学：底盘速度 → 轮子目标转速
 *
 * @param v_tx   X轴平移速度 (m/s)
 * @param v_ty   Y轴平移速度 (m/s)
 * @param omega  旋转角速度 (rad/s)
 * @param target_rpm  输出：四个轮子的目标转速 (RPM，转/min)
 */
void Kinematics_Inverse(float v_tx, float v_ty, float omega, float *target_rpm) {
    // 旋转项计算：omega*(rx + ry)
    float rotation_term = omega * (ROBOT_R_X + ROBOT_R_Y);
    target_rpm[0] = v_tx - v_ty - rotation_term; // 轮1(左前)
    target_rpm[1] = v_tx + v_ty - rotation_term; // 轮2(左后)
    target_rpm[2] = v_tx + v_ty + rotation_term; // 轮3(右前)
    target_rpm[3] = v_tx - v_ty + rotation_term; // 轮4(右后)
    // 转换线速度(m/s) → 转速(RPM)
    // 公式：RPM = (线速度/(2πr)) * 60 * 减速比
    float scale = 60.0f * WHEEL_GEAR_RATIO / (2 * M_PI * WHEEL_RADIUS);
    for (int i = 0; i < 4; i++) {
        target_rpm[i] *= scale;
    }
}

/**
 * @brief 将转速转为占空比
 *
 * @param rpm 逆运动学解得出的电机转速
 * @param duty 返回计算的占空比结果
 * */
void Kinematics_RpmToDuty(const float *rpm, float *duty) {
    float cnt[4];
    for (int i = 0; i < 4; i++) {
        // 10ms计数值
        cnt[i] = ENCODER_PPR * WHEEL_GEAR_RATIO * rpm[i] * SAMPLING_TIME / 60;
        // 计算占空比
        duty[i] = fmaxf(fminf((cnt[i] / ENCODER_MAX) * 100.0f, 100.0f), -100.0f);
    }
}

void Kinematics_CntToRpm(const float *cnt, float *rpm) {
    for (int i = 0; i < 4; i++) {
        // 10ms计数值
        rpm[i] =  cnt[i] * 60 / (ENCODER_PPR * WHEEL_GEAR_RATIO * SAMPLING_TIME);
    }
}

/**
 * @brief 正运动学：轮子实际转速 → 底盘运动状态
 * @param actual_rpm  四个轮子的实际转速 (RPM)
 * @param v_tx  输出X轴速度 (m/s)
 * @param v_ty  输出Y轴速度 (m/s)
 * @param omega 输出角速度 (rad/s)
 */
void Kinematics_Position(const float *actual_rpm, float *v_tx, float *v_ty, float *omega)
{
    // 转速(RPM) → 线速度(m/s)
    float v_wheels[4];
    float scale = (2 * M_PI * WHEEL_RADIUS) / (60.0f * WHEEL_GEAR_RATIO);
    for (int i = 0; i < 4; i++) {
        v_wheels[i] = actual_rpm[i] * scale;
    }

    /* 构造最小二乘矩阵 Ax = b
     A矩阵结构（每行对应一个轮子）：
     [1, -1, -(rx+ry)]  // 轮1
     [1, 1,  (rx+ry)]   // 轮2
     [1, 1, -(rx+ry)]   // 轮3
     [1, -1, (rx+ry)]   // 轮4
     x = [v_tx, v_ty, omega]^T */

    // 计算ATA和ATb
    float ATA[3][3] = {0}, ATb[3] = {0};
    const float r_sum = ROBOT_R_X + ROBOT_R_Y;
    const float A[4][3] = {
            {1, -1, -r_sum},
            {1, 1,  r_sum},
            {1, 1,  -r_sum},
            {1, -1, r_sum}
    };

    // 矩阵乘法累加
    for (int i = 0; i < 4; i++) {
        // ATA = A^T * A
        ATA[0][0] += A[i][0] * A[i][0]; // ∑1*1
        ATA[0][1] += A[i][0] * A[i][1]; // ∑1*(-1)
        ATA[0][2] += A[i][0] * A[i][2]; // ∑1*±r_sum
        ATA[1][1] += A[i][1] * A[i][1]; // ∑(-1)*(-1)
        ATA[1][2] += A[i][1] * A[i][2]; // ∑(-1)*±r_sum
        ATA[2][2] += A[i][2] * A[i][2]; // ∑(±r_sum)^2

        // ATb = A^T * b
        ATb[0] += A[i][0] * v_wheels[i]; // ∑1*v_wi
        ATb[1] += A[i][1] * v_wheels[i]; // ∑(-1)*v_wi
        ATb[2] += A[i][2] * v_wheels[i]; // ∑±r_sum*v_wi
    }
    // 对称元素填充
    ATA[1][0] = ATA[0][1];
    ATA[2][0] = ATA[0][2];
    ATA[2][1] = ATA[1][2];

    // 克莱姆法则求解
    float det = ATA[0][0] * (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1])
                - ATA[0][1] * (ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0])
                + ATA[0][2] * (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]);

    if (fabs(det) < 1e-6) { // 奇异矩阵处理
        *v_tx = *v_ty = *omega = 0.0f;
        return;
    }

    // 计算逆矩阵的辅助行列式
    float inv_det = 1.0f / det;
    float invATA[3][3] = {
            {(ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) * inv_det,
                    (ATA[0][2] * ATA[2][1] - ATA[0][1] * ATA[2][2]) * inv_det,
                    (ATA[0][1] * ATA[1][2] - ATA[0][2] * ATA[1][1]) * inv_det},
            // 第二行（实际计算中可省略，因x=invATA·ATb只需第一行）
    };

    // 计算v_tx（仅需第一行与ATb的点积）
    *v_tx = invATA[0][0] * ATb[0] + invATA[0][1] * ATb[1] + invATA[0][2] * ATb[2];
    *v_ty = (ATb[1] - ATA[1][0] * (*v_tx)) / ATA[1][1]; // 简化计算
    *omega = (ATb[2] - ATA[2][0] * (*v_tx) - ATA[2][1] * (*v_ty)) / ATA[2][2];
}
