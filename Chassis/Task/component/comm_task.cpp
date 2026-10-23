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

#include "ins_comm.hpp" /* GetRemoteControl */
#include "usart.h"      /* huart3 */
#include "tick.hpp"     /* hello_world::tick::GetTickMs() */

/* Private constants ---------------------------------------------------------*/
namespace
{
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;
} // namespace

/* Private variables ---------------------------------------------------------*/
static uint8_t dbus_rx_buf[hello_world::remote_control::kRcRxDataLen];
/* Ozone 观察用 */
static volatile uint32_t rc_rx_cnt = 0;  /* 成功解包的帧数 */
static volatile uint32_t rc_err_cnt = 0; /* 长度不对、被丢弃的次数 */

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

  // 启动 DBUS 接收（空闲中断 + DMA）
  (void)HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3, dbus_rx_buf, hello_world::remote_control::kRcRxDataLen);
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

/* 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收； */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size)
{
  if (huart != &huart3)
  {
    return;
  }
  if (huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }
  /* ②  只处理完整的一帧：长度必须等于 18。
   *    线路中途空闲 Size<18 丢弃并计数。 */
  if (Size == hello_world::remote_control::kRcRxDataLen)
  {
    GetRemoteControl()->decode(dbus_rx_buf);
    rc_rx_cnt = rc_rx_cnt + 1U;
  }
  else
  {
    rc_err_cnt = rc_err_cnt + 1U;
  }

  /*    重启接收：DMA 为 NORMAL 时 HAL 已在 IDLE 里把状态复位成 READY，
   *    所以这里可以安全重启。每次重启都对齐到"下一帧的起点"。 */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3, dbus_rx_buf, hello_world::remote_control::kRcRxDataLen);
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

/**
 * @brief  遥控器是否在线
 * @retval true = 最近 100 ms 内收到过新帧
 * @note   rc_rx_cnt 是 static（本文件私有），所以这个判断只能放在本文件。
 *         不能只看"计数为 0"：上电初期本来就是 0，判据是"一段时间内有没有变化"。
 */
bool RcIsOnline(void)
{
  constexpr uint32_t kRcTimeoutMs = 100U;

  static uint32_t last_cnt = 0;
  static uint32_t last_tick = 0;

  uint32_t cnt = rc_rx_cnt; /* volatile 读一次，后面都用这个快照 */

  if (cnt != last_cnt)
  {
    last_cnt = cnt;
    last_tick = hello_world::tick::GetTickMs();
    return true;
  }

  /* 无符号减法：计到 2^32 回绕依然给出正确的时间差 */
  return (hello_world::tick::GetTickMs() - last_tick) < kRcTimeoutMs;
}