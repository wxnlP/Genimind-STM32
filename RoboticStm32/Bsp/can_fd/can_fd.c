#include "can_fd.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

static SemaphoreHandle_t g_canMutex = NULL;


void CAN_Init(void) {
    g_canMutex = xSemaphoreCreateMutex();
}

/**
 * @brief 发送消息
 *
 * @param ID 报文ID
 * @param TxMessage Tx消息头结构体 CAN_TxHeaderTypeDef
 * @param pData 数据指针，可以传入一个数组
 * @retval 1:发送成功; 0:发送失败
 * */
uint8_t CAN_SendMessage(CAN_TxHeaderTypeDef* TxMessage, uint8_t* pData) {
    uint32_t TxMailbox;
    uint8_t result = 0;
    // 阻塞等待直到获取互斥锁
    if (xSemaphoreTake(g_canMutex, portMAX_DELAY) == pdTRUE) {
        // 发送数据并记录结果
        result = (HAL_CAN_AddTxMessage(&hcan, TxMessage, pData, &TxMailbox) == HAL_OK) ? 1 : 0;
        // 释放互斥锁
        xSemaphoreGive(g_canMutex);
    }
    // 返回结果
    return result;
}

/**
 * @brief 接收FIFO0邮箱非空标志
 *
 * @retval 1:邮箱非全空; 0:邮箱全空
 * */
uint8_t CAN_ReceiveFlag(void) {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        return 1;
    }
    return 0;
}

/**
 * @brief 接收消息
 *
 * @param RxMessage Rx消息头结构体 CAN_RxHeaderTypeDef
 * @param pData 将收到的消息的数据存于此变量
 * @retval 1:发送成功; 0:发送失败
 * */
uint8_t CAN_ReceiveMessage(CAN_RxHeaderTypeDef* RxMessage, uint8_t* pData) {
    uint8_t result = 0;
    // 阻塞等待直到获取互斥锁
    if (xSemaphoreTake(g_canMutex, portMAX_DELAY) == pdTRUE) {
        // 接收数据并记录结果
        result = (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, RxMessage, pData) == HAL_OK) ? 1 : 0;
        // 释放互斥锁
        xSemaphoreGive(g_canMutex);
    }
    // 返回结果
    return result;
}

