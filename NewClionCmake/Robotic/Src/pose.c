#include "pose.h"
#include <math.h>


/* ------------------------------Mahony解算四元数----------------------------- */

/**
 * @brief 四元数归一化处理，q_normalized = q / ||q||
 *
 * 只有单位四元数（模长为1）才能正确表示旋转
 *
 * @param q 四元数结构体 @Quaternion
 * @retval None
 * */
static void Pose_QuaternionNormalize(Quaternion* q)
{
    // 开平方
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    // 异常处理
    if (norm == 0) return;
    // 四元数归一化
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

/**
 * @brief 四元数转欧拉角
 *
 * 很多地方公式常用q0、q1、q2、q3代替四元数表示,四元数结构体的符号等价关系如下：
 * q->w = q0, q->x = q1, q->y = q2, q->z = q3.
 * 其中q0是唯一实数部，其余为虚数部 i、j、k
 *
 * @param q 输入四元数结构体
 * @param euler 输出欧拉角结构体
 * */
void Pose_QuatToEuler(const Quaternion* q, EulerAngles* euler)
{
    euler->roll = atan2f(2.0f * (q->w * q->x + q->y * q->z),
                         q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z);
    euler->pitch = asinf(2.0f * (q->w * q->y - q->z * q->x));
    euler->yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y),
                        q->w * q->w + q->x * q->x - q->y * q->y - q->z * q->z);
}


/**
 * @brief Mahony传感器融合算法解算四元数
 *
 * 以陀螺仪所测的角度为主，把由加速度得到角度误差补偿到由陀螺仪所得的角度值当中。
 * 之所以要这样做，是因为陀螺仪短期测量很准，但在长期测量时容易积累误差，而加速度则相反。
 *
 * @param filter Mahony滤波算法参数
 * @param q 输入当前四元数状态，返回最新的四元数状态
 * @param dt 时间间隔
 * @param accData 加速度计真实值
 * @param gyroData 角速度计真实值
 * @retval None
 * */
void Pose_MahonyUpdate(MahonyFilter *filter, float dt, Quaternion* q, ImuReal* accData, ImuReal* gyroData)
{
    /* 接收参数指针 */
    float ax = accData->X, ay = accData->Y, az = accData->Z;
    float gx = gyroData->X, gy = gyroData->Y, gz = gyroData->Z;
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;

    /* 将陀螺仪的单位°/s转化为rad/s */
    gx *= (float)(M_PI / 180.0f);
    gy *= (float)(M_PI / 180.0f);
    gz *= (float)(M_PI / 180.0f);

    /* 加速度归一化 */
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;

    /* 利用上次计算的四元数估算地面坐标系的重力加速度(g = 9.8m/s^2) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* 向量积计算角度差  axb = |a||b|sinθ
     * a向量在代码中是陀螺仪所得的各个轴的加速度值
     * b向量在代码中就是重力加速度的估算值
     * | i  j  k |
     * | ax ay az| =>
     * | vx vy vz|
     * */
    float ex = ay*vz - az*vy;  // X轴误差
    float ey = az*vx - ax*vz;  // Y轴误差
    float ez = ax*vy - ay*vx;  // Z轴误差

    /* 补偿到陀螺仪所测的值 */
    filter->integralFBx += ex * filter->Ki;
    filter->integralFBy += ey * filter->Ki;
    filter->integralFBz += ez * filter->Ki;

    gx += filter->Kp * ex + filter->integralFBx;
    gy += filter->Kp * ey + filter->integralFBy;
    gz += filter->Kp * ez + filter->integralFBz;

    q->w += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q->x += (q0 * gx + q2 * gz - q3 * gy)  * 0.5f * dt;
    q->y += (q0 * gy - q1 * gz + q3 * gx)  * 0.5f * dt;
    q->z += (q0 * gz + q1 * gy - q2 * gx)  * 0.5f * dt;

    Pose_QuaternionNormalize(q);
}


/**
 * @brief Mahony传感器融合算法解算四元数(含磁力计)
 *
 * 以陀螺仪所测的角度为主，把由加速度得到角度误差补偿到由陀螺仪所得的角度值当中。
 * 之所以要这样做，是因为陀螺仪短期测量很准，但在长期测量时容易积累误差，而加速度则相反。
 *
 * @param filter Mahony滤波算法参数
 * @param q 输入当前四元数状态，返回最新的四元数状态
 * @param dt 时间间隔
 * @param accData 加速度计真实值
 * @param gyroData 陀螺仪真实值
 * @param magData 磁力计真实值
 * @retval None
 * */
void Pose_MahonyUpdate2(MahonyFilter *filter, float dt, Quaternion* q,
                           ImuReal* accData, ImuReal* gyroData, ImuReal* magData)
{
    // 读取加速度、陀螺仪数据
    float ax = accData->X, ay = accData->Y, az = accData->Z;
    float gx = gyroData->X, gy = gyroData->Y, gz = gyroData->Z;
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;

    // 将陀螺仪单位转换为 rad/s
    gx *= (float)(M_PI / 180.0f);
    gy *= (float)(M_PI / 180.0f);
    gz *= (float)(M_PI / 180.0f);

    // 加速度归一化
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0) return;
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 估计重力方向 (单位矢量)
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算加速度和重力估计之间的误差（用于 roll 和 pitch 校正）
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    // ------------------磁力计校正部分------------------
    // 读取磁力计数据
    float mx = magData->X, my = magData->Y, mz = magData->Z;
    // 磁力计归一化
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0) return;
    mx /= norm;
    my /= norm;
    mz /= norm;

    // 根据当前姿态 (四元数) 提取 roll 和 pitch
    // （也可以直接用加速度数据估计，但此处使用四元数计算会更一致）
    float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1 - 2.0f*(q1*q1 + q2*q2));
    float pitch = asinf(2.0f*(q0*q2 - q3*q1));

    // 倾斜补偿磁力计数据
    // 修正为基于旋转矩阵的补偿（先绕X轴旋转-roll，再绕Y轴旋转-pitch）
    float cos_roll = cosf(-roll);
    float sin_roll = sinf(-roll);
    float cos_pitch = cosf(-pitch);
    float sin_pitch = sinf(-pitch);

// 应用旋转矩阵
    float mx_horiz = mx * cos_pitch + mz * sin_pitch;
    float my_horiz = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch;
    float mz_horiz = -mx * cos_roll * sin_pitch + my * sin_roll + mz * cos_roll * cos_pitch;

// 计算水平投影后的磁北方向
    float mx_comp = mx_horiz;
    float my_comp = my_horiz;
    // 计算磁力计获得的航向角（yaw），注意 atan2 参数顺序及符号可能需要根据实际校准调整
    float yaw_mag = atan2f(my_comp, mx_comp);

    // 从当前四元数计算估计的 yaw
    float yaw_est = atan2f(2.0f*(q0*q3 + q1*q2), 1 - 2.0f*(q2*q2 + q3*q3));

    // 计算 yaw 误差
    float yaw_err = yaw_mag - yaw_est;
    // 注意：yaw_err 需要归一化到 [-pi, pi]
    while (yaw_err > M_PI) yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    // 将 yaw 误差加入到误差反馈中
    // 这里可以只校正 z 轴（yaw），也可以分开不同的比例因子
    // 根据加速度模值判断动态状态
    float accel_norm = sqrtf(ax*ax + ay*ay + az*az);
    float dynamic_factor = fabs(accel_norm - 9.81f) > 0.5f ? 0.2f : 1.0f; // 动态时降低磁力计权重

    ez += yaw_err * dynamic_factor; // 动态环境下减弱磁力计校正

    // -----------------------------------------------------

    // 积分反馈（用于减缓长期漂移）
    filter->integralFBx += ex * filter->Ki;
    filter->integralFBy += ey * filter->Ki;
    filter->integralFBz += ez * filter->Ki;

    // 用比例反馈加上积分项修正陀螺仪数据
    gx += filter->Kp * ex + filter->integralFBx;
    gy += filter->Kp * ey + filter->integralFBy;
    gz += filter->Kp * ez + filter->integralFBz;

    // 四元数积分更新
    q->w += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q->x += (q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    q->y += (q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    q->z += (q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;

    // 归一化四元数，保持单位四元数
    Pose_QuaternionNormalize(q);
}

