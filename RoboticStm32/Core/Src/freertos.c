/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "beef.h"
#include "oled.h"
#include "voltage.h"
#include "pid.h"
#include "kinematics.h"
#include "pose.h"
#include "can_mid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
        .name = "defaultTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void IMU_Task(void *argument);

void RTOS_Init(void);

void CAN_Task(void *argument);

void PIDTask(void *arg);

void KinematicsTask(void *arg);

/* CAN中断 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {

    }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */
    RTOS_Init();
    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    xTaskCreate(IMU_Task, "IMU", 1024, NULL, osPriorityNormal, NULL);
    xTaskCreate(CAN_Task, "CAN", 128 * 4, NULL, osPriorityNormal, NULL);
    xTaskCreate(PIDTask, "PID", 128 * 4, NULL, osPriorityNormal, NULL);
    xTaskCreate(KinematicsTask, "Kinematic", 128 * 2, NULL, osPriorityNormal, NULL);

    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    float voltage;
    while (1) {
        voltage = ADC_Voltage_Actual();
        if (voltage < 9.50f) {
            Buzzer_ON_Time();
        }
        CanMid_SendVoltage(voltage);
        OLED_ShowVoltage(88, 2, voltage, 16);
        vTaskDelay(200);
    }

    /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 外设初始
void RTOS_Init(void) {
    OLED_Init();
    ADC_Voltage_Init();
    Motor_Init();
    Encoder_Init();
    PID_Init();
    CAN_Init();
    CanMid_Init();
    // 等待RDK的CAN初始化完�?
//    while(HAL_GPIO_ReadPin(RDK_Signal_GPIO_Port, RDK_Signal_Pin) != GPIO_PIN_SET);
//    LED_ON();
//    HAL_Delay(200);
//    LED_OFF();
}

// IMU任务
void IMU_Task(void *argument) {
    /* 传感器初始化 */
    ICM20948_Init();
    AK09916_Init();
    /* 数据缓冲�?????? */
    ImuReal accelReal, gyroReal, magReal;
    /* Mahony滤波器初始化 */
    MahonyFilter filter = {
            .Kp = 0.3f,
            .Ki = 0.005f,
            .integralFBx = 0,
            .integralFBy = 0,
            .integralFBz = 0
    };
    Quaternion quat = {1.0f, 0.0f, 0.0f, 0.0f};
    EulerAngles euler = {0};
    /* 初始姿�?�初始化标志 */
    uint32_t send_counter = 0;
    /* 控制发�?�周�?????? */
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 100Hz任务频率
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        /* 真实传感器数�?????? */
        ICM20948_RealData(&accelReal, &gyroReal);
        AK09916_RealData(&magReal);
        CanMid_SendImu(&accelReal, &gyroReal);
        vTaskDelay(5);
        /* Mahony滤波更新姿�?? */
        Pose_MahonyUpdate(&filter, 0.01f, &quat,
                          &accelReal, &gyroReal);
        CanMid_SendQuat(&quat);

        /* 精确周期延迟 */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// CAN任务
void CAN_Task(void *argument) {
    /* can缓存 */
    CAN_RxHeaderTypeDef RxMessage = {0};
    uint8_t RxData[8] = {0};
    float RealData[2] = {0};
    /* 标志�????? */
    uint8_t linear_flag = 0, angular_flag = 0;
    /* 指令存储 */
    Odom odom;
    while (1) {
        /* 接收并解�????? */
        if (CAN_ReceiveFlag()) {
            // 接收数据
            CAN_ReceiveMessage(&RxMessage, RxData);
            // 转换字节为真�????
            CanMid_BytesToFloat(RxData, RealData);
            switch (RxMessage.StdId) {
                case CMD_ANGULAR:
                    angular_flag = 1;
                    odom.omega = RealData[0];
                    break;
                case CMD_LINEAR:
                    linear_flag = 1;
                    odom.v_tx = RealData[0];
                    odom.v_ty = RealData[1];
                    break;
                case CMD_BUZZER:
                    Buzzer_LED_OnTime(RxData[0], RxData[1]);
                    break;
                default:
                    break;
            }
            /* 拼接线�?�度、角速度指令 */
            if (angular_flag == 1 && linear_flag == 1) {
                LED_ON();
                // 清零标志
                angular_flag = 0;
                linear_flag = 0;
                UART_SendFloat(&huart1, odom.v_tx);
                UART_SendFloat(&huart1, odom.v_ty);
                // 写队
                CanMid_WriteOdom(&odom);
            }
        }
        vTaskDelay(10);
    }
}

// PID任务
void PIDTask(void *arg) {
    float v_tx, v_ty, omega;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10ms控制周期
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        PID_Controller(&v_tx, &v_ty, &omega);
        CanMid_SendOdom(v_tx, v_ty, omega);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 运动�????
void KinematicsTask(void *arg) {
    float target_rpm[4] = {0};
    float duty[4] = {0};
    Odom odom;
    while (1) {
        CanMid_ReadOdom(&odom);
        // 逆运动学解算
        Kinematics_Inverse(odom.v_tx, odom.v_ty, odom.omega, target_rpm);
        // 转换为占空比并限
        Kinematics_RpmToDuty(target_rpm, duty);
        // 设置PID目标
        PID_SetTarget(duty);
//        LED_OFF();
    }
}

/* USER CODE END Application */

