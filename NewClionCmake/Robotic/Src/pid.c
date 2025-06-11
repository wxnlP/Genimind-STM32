#include "pid.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "oled.h"
#include "kinematics.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define new

// 信号量
static SemaphoreHandle_t g_pidSemaphore = NULL;

#ifdef old
static PID_Params g_PID = {1, 0, 0};
static PID_Error g_Error;
static float g_Target[4] = {0};
static float EncoderMax = 300;
#endif

#ifdef new
static Motor_PID g_MotorPID[4];  // 四电机独立实例
#endif


#ifdef old
/* 设置目标值 */
void PID_SetTarget(const float* pTarget) {
    for (int i = 0; i < 4; i++) {
        g_Target[i] = pTarget[i];
        Motor_SetSpeed(i+1, pTarget[i]);
    }
}

/* 将计算结果运用于控制器 */
static void PID_Func(const float* pDuty) {
    for (int i = 0; i < 4; i++) {
        Motor_SetSpeed(i+1, pDuty[i]);
    }
}

static float PID_EncoderToDuty(int16_t cnt) {
    return (float)((cnt*1.0/EncoderMax)*100);
}

/* PID信号量 */
void PID_Init(void) {
    g_pidSemaphore = xSemaphoreCreateBinary();
}

/* 释放互斥锁 */
BaseType_t  PID_GiveMutex(void) {
    return xSemaphoreGive(g_pidSemaphore);
}

/* 中断中释放信号量 */
BaseType_t  PID_GiveMutexFromISR(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    return xSemaphoreGiveFromISR(g_pidSemaphore, &xHigherPriorityTaskWoken);
}

/* 获取信号量 */
BaseType_t PID_TakeMutex(void) {
    return xSemaphoreTake(g_pidSemaphore, portMAX_DELAY);
}

/* 获取信号量状态 */
SemaphoreHandle_t PID_MutexStatus(void) {
    return g_pidSemaphore;
}

/* 获取PID计算结果 */
void PID_Controller(void) {
    float Out[4];
    char message[100] = "";
    int16_t cnt;
    for (int i = 0; i < 4; i++) {
        cnt = Encoder_Read_CNT(i+1);
        /* 获取真实值 */
        g_Error.Actual = PID_EncoderToDuty(cnt);
//        g_Error.Actual = cnt;
        /* 处理误差 */
        g_Error.Pre_Error = g_Error.Cur_Error;
        g_Error.Cur_Error = g_Target[i] - g_Error.Actual;
        g_Error.Cnt_Error += g_Error.Cur_Error;
        /* PID算法 */
        Out[i] = g_PID.P * g_Error.Cur_Error + g_PID.I * g_Error.Cnt_Error + g_PID.D * (g_Error.Cur_Error - g_Error.Pre_Error);
        /* 控制上下限 */
        if ( Out[i] > 100 ) { Out[i] = 100; }
        if ( Out[i] < -100 ) { Out[i] = -100; }
    }
    PID_Func(Out);
    sprintf(message, "%f,%f,%d\r\n", g_Error.Actual, Out[3], 50);
    HAL_UART_Transmit(&huart1, (uint8_t *) message, strlen(message), 100);
}

/* 设置PID参数P */
void PID_SetP(float P) {
    g_PID.P = P;
}

/* 设置PID参数I */
void PID_SetI(float I) {
    g_PID.I = I;
}

/* 设置PID参数D */
void PID_SetD(float D) {
    g_PID.D = D;
}

/* SerialPlot绘图交互 */
void PID_SerialPlot(const float* pDuty, float target, char* message) {
    sprintf(message, "%f,%f,%f,%f,%f\r\n", pDuty[0], pDuty[1], pDuty[2], pDuty[3],target);
    HAL_UART_Transmit(&huart1, (uint8_t *) message, strlen(message), 100);
}
#endif

#ifdef new
//------------------------------ 初始化部分 ------------------------------
void PID_Init(void) {
    g_pidSemaphore = xSemaphoreCreateMutex();
    for(int i=0; i<4; i++){
        g_MotorPID[i] = (Motor_PID){
            // 默认PID参数
            .params = {0.6f, 0.05f, 0.0f},
            .encoder_max = 1300,
            .target = 0.0f
        };
    }
}

//------------------------------ 核心控制逻辑 ------------------------------
static void Motor_PID_Calculate(uint8_t motor_id, int16_t cnt) {
    if(motor_id >=4) return;
    /* 数值转换 */
    float actual = (cnt * 1.0f / g_MotorPID[motor_id].encoder_max) * 100.0f;
    /* 误差计算 */
    PID_Error* err = &g_MotorPID[motor_id].error;
    err->Pre_Error = err->Cur_Error;
    err->Cur_Error = g_MotorPID[motor_id].target - actual;
    err->Cnt_Error += err->Cur_Error;
    /* 抗积分饱和处理 */
    if(fabsf(err->Cnt_Error) > 100.0f){  // 根据实际调整阈值
        err->Cnt_Error *= 0.9f;
    }
    /* PID计算 */
    float out = g_MotorPID[motor_id].params.P * err->Cur_Error
                + g_MotorPID[motor_id].params.I * err->Cnt_Error
                + g_MotorPID[motor_id].params.D * (err->Cur_Error - err->Pre_Error);
    /* 输出限幅 */
    g_MotorPID[motor_id].output = fmaxf(fminf(out, 100.0f), -100.0f);
}

void PID_Controller(float *v_tx, float *v_ty, float *omega) {
    if(xSemaphoreTake(g_pidSemaphore, pdMS_TO_TICKS(10)) == pdTRUE){
        float cntArray[4] = {0};
        float rpmArray[4] = {0};
        for(int i=0; i<4; i++){
            /* 获取编码器原始值 */
            cntArray[i] = Encoder_Read_CNT(i+1);
            Motor_PID_Calculate(i, (int16_t)cntArray[i]);
            Motor_SetSpeed(i+1, g_MotorPID[i].output);
        }
        Kinematics_CntToRpm(cntArray, rpmArray);
        Kinematics_Position(rpmArray, v_tx, v_ty, omega);
        xSemaphoreGive(g_pidSemaphore);
    }
}

void PID_SetTarget(const float* pTarget) {
    if(xSemaphoreTake(g_pidSemaphore, portMAX_DELAY) == pdTRUE){
        for(int i=0; i<4; i++){
            g_MotorPID[i].target = pTarget[i];
        }
        xSemaphoreGive(g_pidSemaphore);
    }
}

void PID_SetP(float P) {
    for(int i=0; i<4; i++){
        g_MotorPID[i].params.P = P;
    }
}

void PID_SetI(float I) {
    for(int i=0; i<4; i++){
        g_MotorPID[i].params.I = I;
    }
}

void PID_SetD(float D) {
    for(int i=0; i<4; i++){
        g_MotorPID[i].params.D = D;
    }
}
#endif


