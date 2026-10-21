/**
 *******************************************************************************
 * @file      :comm_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

#include "HW_can.hpp"
#include "can.h"
#include "ins_motor.hpp"    /* GetMotorWheel / GetMotorSteer */
#include "motor_GM6020.hpp" /* GM6020::GetTxId */
#include "motor_M3508.hpp"  /* M3508::GetTxId */

/* Private constants ---------------------------------------------------------*/
namespace
{
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;
} // namespace

/* Private function prototypes -----------------------------------------------*/
static void TransmitWheelFrame(void);
static void TransmitSteerFrame(void);

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void)
{
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CommTask(void)
{
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */
  TransmitWheelFrame();
  TransmitSteerFrame();
}

/* Private function definitions ----------------------------------------------*/
static void TransmitWheelFrame(void)
{
  uint8_t tx_data[8] = {0};

  for (int i = 0; i < kWheelNum; ++i)
  {
    (void)GetMotorWheel(i)->encode(tx_data);
  }

  // 用共享发送 ID 发出去。
  (void)CAN_Send_Msg(&hcan2, tx_data,
                     hello_world::motor::M3508::GetTxId(GetMotorWheel(0)->id()), 8);
}

static void TransmitSteerFrame(void)
{
  uint8_t tx_data[8] = {0}; /* 同 E：必须清零 */

  for (int i = 0; i < kSteerNum; ++i)
  {
    (void)GetMotorSteer(i)->encode(tx_data); /* 同 F */
  }

  hello_world::motor::InputType in_type = GetMotorSteer(0)->get_input_type();
  (void)CAN_Send_Msg(&hcan1, tx_data,
                     hello_world::motor::GM6020::GetTxId(GetMotorSteer(0)->id(), in_type), 8);
}
/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */