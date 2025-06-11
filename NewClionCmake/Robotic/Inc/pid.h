#ifndef __PID_H
#define __PID_H

#include "motor.h"
#include "encoder.h"


/* PID可调的参数 */
typedef struct {
    float P;
    float I;
    float D;
} PID_Params;

/* PID需从外部获取的参数 */
typedef struct {
    float Cur_Error;
    float Pre_Error;
    float Cnt_Error;
    float Actual;
} PID_Error;

/* 新增电机独立参数结构体 */
typedef struct {
    PID_Params params;
    PID_Error error;
    float target;
    float output;
    int16_t encoder_max;
} Motor_PID;

void PID_Init(void);
//BaseType_t  PID_GiveMutex(void);
//BaseType_t  PID_GiveMutexFromISR(void);
//BaseType_t PID_TakeMutex(void);
//SemaphoreHandle_t PID_MutexStatus(void);
void PID_SetTarget(const float* pTarget);
void PID_Controller(float *v_tx, float *v_ty, float *omega);
void PID_SetP(float P);
void PID_SetI(float I);
void PID_SetD(float D);
//void PID_SerialPlot(const float* pDuty, float target, char* message);
void System_Init(void);
#endif
