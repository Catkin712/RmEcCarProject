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

#include "DT7.hpp"      /* DT7::rc_xxx()，协议层不直接依赖它 */
#include "HW_can.hpp"   /* CanFilter_Init / CAN_Send_Msg */
#include "can.h"        /* hcan1 */
#include "ins_comm.hpp" /* GetRemoteControl */
#include "usart.h"      /* huart3 */
#include "iwdg.h"
#include "tick.hpp" /* hello_world::tick::GetTickMs() */

#include "gimbal_chassis_comm.hpp"     /* 完整类型：encode()/tx_id() */
#include "ins_chassis_gimbal_comm.hpp" /* GetGimbalChassisComm */

/* Private variables ---------------------------------------------------------*/
static uint8_t dbus_rx_buf[hello_world::remote_control::kRcRxDataLen];
/* 三个状态量：放在文件作用域、用 static（不用匿名 namespace），
 * 这样 Ozone 的表达式里直接写 g_comm_armed 就能找到，不会显示成
 * "(anonymous namespace)::g_comm_armed"。 */
static volatile bool g_comm_armed = false; /* 是否已收到过有效帧 */
static uint32_t g_boot_tick = 0;           /* CommTaskInit 里记录的开机时刻 */
static constexpr uint32_t kStartupGraceMs = 2000U; /* 开机宽限期 */
/* Ozone 观察用 */
static volatile uint32_t rc_rx_cnt = 0;  /* 成功解包的帧数 */
static volatile uint32_t rc_err_cnt = 0; /* 长度不对、被丢弃的次数 */
static volatile uint32_t tick_now = 0;
static volatile uint32_t link_tx_cnt = 0; /* 已发出的 0x102 帧数 */

/* Private function prototypes -----------------------------------------------*/
bool RcIsOnline(void); /* 定义在本文件末尾，CommTask 要先用 */

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void)
{
  g_boot_tick = hello_world::tick::GetTickMs();

  /* 云台 CAN1：只接底盘发来的 0x101，滤波器把 FIFO0/bank0 全收。
   * 云台 CAN2（Pitch 达妙）等任务 8 再开，现在不启动。 */
  CanFilter_Init(&hcan1);
  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 启动 DBUS 接收（空闲中断 + DMA）
  (void)HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3, dbus_rx_buf, hello_world::remote_control::kRcRxDataLen);
}

void CommTask(void)
{
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  hello_world::remote_control::DT7 *rc = GetRemoteControl();

  /* ① 把遥控器语义搬进协议结构：
   *    协议层（RobotComponents）不 include DT7，转换只在 Task 层做。 */
  comm->gimbal_data().rc_lv = rc->rc_lv();
  comm->gimbal_data().rc_lh = rc->rc_lh();
  comm->gimbal_data().rc_rv = rc->rc_rv();
  comm->gimbal_data().rc_rh = rc->rc_rh();
  comm->gimbal_data().rc_wheel = rc->rc_wheel();
  comm->gimbal_data().l_switch = static_cast<uint8_t>(rc->rc_l_switch());
  comm->gimbal_data().r_switch = static_cast<uint8_t>(rc->rc_r_switch());
  comm->gimbal_data().ready = true;
  comm->gimbal_data().rc_online = RcIsOnline();

  /* ② 打包并发出去（1 kHz，8 字节） */
  uint8_t frame[8] = {0};
  comm->encode(frame);
  (void)CAN_Send_Msg(&hcan1, frame, comm->tx_id(), 8);
  link_tx_cnt = link_tx_cnt + 1U;
}

/* Private function definitions ----------------------------------------------*/
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

    CommTaskNotifyCommOk(); // 有效帧
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
  static bool seen = false;

  uint32_t cnt = rc_rx_cnt; /* volatile 读一次，后面都用这个快照 */

  if (cnt != last_cnt)
  {
    last_cnt = cnt;
    last_tick = hello_world::tick::GetTickMs();
    seen = true; /* 收到过帧之后，时间判据才有意义 */
    return true;
  }
  if (!seen)
  {
    return false;
  }
  /* 无符号减法：计到 2^32 回绕依然给出正确的时间差 */
  return (hello_world::tick::GetTickMs() - last_tick) < kRcTimeoutMs;
}

extern "C" void CommTaskNotifyCommOk(void)
{
  g_comm_armed = true;
  (void)HAL_IWDG_Refresh(&hiwdg);
}

extern "C" void CommTaskWatchdog(void)
{
  /* Ozone 只能 watch 变量、不能 watch 函数返回值：
   * 每圈把当前时间写进 tick_now，调试时看它涨不涨即可 */
  tick_now = hello_world::tick::GetTickMs();

  if (g_comm_armed)
  {
    return; /* 已 armed：此后只由解包处喂狗 */
  }
  if ((tick_now - g_boot_tick) < kStartupGraceMs)
  {
    (void)HAL_IWDG_Refresh(&hiwdg); /* 宽限期：只喂狗，不 arm */
  }
}