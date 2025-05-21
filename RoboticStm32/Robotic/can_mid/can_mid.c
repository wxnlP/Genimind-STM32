#include "can_mid.h"
#include <string.h>


#define INIT_CAN_TX_HEADER(id) { \
    .StdId = (id),               \
    .ExtId = 0,                  \
    .IDE = CAN_ID_STD,           \
    .RTR = CAN_RTR_DATA,         \
    .DLC = 8,                    \
    .TransmitGlobalTime = DISABLE \
}

static QueueHandle_t g_odomQueue;
static QueueHandle_t g_buzzerQueue;
static SemaphoreHandle_t g_canRxSemaphore = NULL;

static CAN_TxHeaderTypeDef g_quatTxMessage1 = INIT_CAN_TX_HEADER(0x411);
static CAN_TxHeaderTypeDef g_quatTxMessage2 = INIT_CAN_TX_HEADER(0x412);
static CAN_TxHeaderTypeDef g_linearTxMessage  = INIT_CAN_TX_HEADER(0x421);
static CAN_TxHeaderTypeDef g_angularTxMessage  = INIT_CAN_TX_HEADER(0x422);
static CAN_TxHeaderTypeDef g_imuTxMessage1  = INIT_CAN_TX_HEADER(0x431);
static CAN_TxHeaderTypeDef g_imuTxMessage2  = INIT_CAN_TX_HEADER(0x432);
static CAN_TxHeaderTypeDef g_imuTxMessage3  = INIT_CAN_TX_HEADER(0x433);
static CAN_TxHeaderTypeDef g_voltageTxMessage  = INIT_CAN_TX_HEADER(0x441);


static void CanMid_QuatToBytes(Quaternion* q, uint8_t* quat) {
    memcpy(quat, &q->w, sizeof(float));
    memcpy(quat+4, &q->x, sizeof(float));
    memcpy(quat+8, &q->y, sizeof(float));
    memcpy(quat+12, &q->z, sizeof(float));
}

static void CanMid_ImuToBytes(ImuReal* accelData, ImuReal* gyroData, uint8_t* imu) {
    memcpy(imu, &accelData->X, sizeof(float));
    memcpy(imu+4, &accelData->Y, sizeof(float));
    memcpy(imu+8, &accelData->Z, sizeof(float));
    memcpy(imu+12, &gyroData->X, sizeof(float));
    memcpy(imu+16, &gyroData->Y, sizeof(float));
    memcpy(imu+20, &gyroData->Z, sizeof(float));
}

/**
 * @brief 发送四元数
 *
 * @param q 四元数
 * */
void CanMid_SendQuat(Quaternion* q)
{
    // 初始化四元数数组
    static uint8_t quat[16] = {0};
    CanMid_QuatToBytes(q, quat);
    CAN_SendMessage(&g_quatTxMessage1, quat);
    CAN_SendMessage(&g_quatTxMessage2, quat+8);
}

/**
 * @brief 发送IMU六轴数据
 *
 * @param accelData 加速度计数据
 * @param gyroData 陀螺仪数据
 * */
void CanMid_SendImu(ImuReal* accelData, ImuReal* gyroData)
{
    static uint8_t imu[24] = {0};
    CanMid_ImuToBytes(accelData, gyroData, imu);
    CAN_SendMessage(&g_imuTxMessage1, imu);
    CAN_SendMessage(&g_imuTxMessage2, imu+8);
    CAN_SendMessage(&g_imuTxMessage3, imu+16);
}

/**
 * @brief 发送电池电压数据
 *
 * @param voltage 电池电压
 * */
void CanMid_SendVoltage(float voltage)
{
    uint8_t v[4] = {0};
    memcpy(v, &voltage, sizeof(float));
    CAN_SendMessage(&g_voltageTxMessage, v);
}

/**
 * @brief 发布里程计数据
 *
 * @param v_tx X轴线速度
 * @param v_ty Y轴线速度
 * @param omega Z轴角速度
 * */
void CanMid_SendOdom(float v_tx, float v_ty, float omega)
{
    static uint8_t odom[12] = {0};
    memcpy(odom, &v_tx, sizeof(float));
    memcpy(odom+4, &v_ty, sizeof(float));
    memcpy(odom+8, &omega, sizeof(float));
    CAN_SendMessage(&g_linearTxMessage, odom);
    CAN_SendMessage(&g_angularTxMessage, odom+8);
}

/**
 * @brief 将字节数组恢复为浮点数
 *
 * @param bytes 要恢复的字节数组，固定8字节长度
 * @param data 接收恢复后的浮点数组，8字节
 * */
void CanMid_BytesToFloat(const uint8_t* bytes, float* data)
{
    for (int i = 0; i < 2; i++) {
        memcpy(&data[i], bytes + i * sizeof(float), sizeof(float));
    }
}

void CanMid_Init(void) {
    g_odomQueue = xQueueCreate(10, sizeof(Odom));
//    g_odomQueue = xQueueCreate(10, sizeof(BuzzerLed));
}

void CanMid_WriteOdom(Odom* data) {
    xQueueSend(g_odomQueue, data, 0);
}

void CanMid_WriteBuzzerLed(BuzzerLed* data) {
    xQueueSend(g_odomQueue, data, 0);
}

void CanMid_ReadOdom(Odom* data) {
    xQueueReceive(g_odomQueue, data, portMAX_DELAY);
}

BaseType_t CanMid_ReadBuzzerLed(BuzzerLed* data) {
    return xQueueReceive(g_odomQueue, data, portMAX_DELAY);
}