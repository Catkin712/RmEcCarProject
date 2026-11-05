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
#include "iwdg.h"       /* hiwdg */

#include "gimbal_chassis_comm.hpp"     /* 完整类型：encode()/tx_id()/chassis_data() */
#include "ins_chassis_gimbal_comm.hpp" /* GetGimbalChassisComm */

/* Private constants ---------------------------------------------------------*/
namespace
{
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;
} // namespace

/* Private variables ---------------------------------------------------------*/
static uint8_t dbus_rx_buf[hello_world::remote_control::kRcRxDataLen];
/* 舵机发送侧的调试镜像：文件作用域（不用匿名 namespace），Ozone 里直接写名字就能找到。
 * g_steer_tx_cnt 涨而 g_steer_tx_ok 不涨 → CAN 发送失败；
 * 两者都不涨 → TransmitSteerFrame() 根本没被执行。 */
uint32_t g_steer_tx_cnt = 0;                       /* 调用次数 */
uint32_t g_steer_tx_ok = 0;                        /* HAL_OK 次数 */
uint32_t g_steer_tx_id = 0;                        /* 实际使用的发送 ID：0x1FE=电流帧，0x1FF=电压帧 */
uint8_t g_steer_tx_data[8] = {0};                  /* 最近一帧的 8 字节 */
static volatile bool g_comm_armed = false;         /* 是否已收到过有效帧 */
static uint32_t g_boot_tick = 0;                   /* CommTaskInit 里记录的开机时刻 */
static constexpr uint32_t kStartupGraceMs = 2000U; /* 开机宽限期 */
/* 看门狗判据：最近一次"有效通讯"的时刻（0x102 且 rc_online）。
 * 喂狗只看主循环 + 这个时刻够不够新，解包中断里绝不喂狗。 */
static volatile uint32_t g_last_comm_tick = 0;
static constexpr uint32_t kCommTimeoutMs = 50U; /* 0x102 是 100 Hz，50 ms 内必须有新帧 */
static volatile uint32_t g_reset_cause = 0;     /* 开机时的 RCC_CSR，bit29=IWDGRSTF */
static volatile uint32_t g_dbg_hang = 0;        /* 故障注入：置 1 模拟主循环卡死 */
/* Ozone 观察用 */
static volatile uint32_t rc_rx_cnt = 0;   /* 成功解包的帧数 */
static volatile uint32_t rc_err_cnt = 0;  /* 长度不对、被丢弃的次数 */
static volatile uint32_t tick_now = 0;    /* CommTask 每 1 ms 刷新的时间快照 */
static volatile uint32_t link_tx_cnt = 0; /* 已发出的 0x101 帧数 */

/* Private function prototypes -----------------------------------------------*/
static void TransmitWheelFrame(void);
static void TransmitSteerFrame(void);
static void TransmitLinkFrame(void);

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void)
{
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */
  g_boot_tick = hello_world::tick::GetTickMs();
  /* 读复位原因再清零：Ozone 里 g_reset_cause 的 bit29(IWDGRSTF)=1 就是看门狗复位 */
  g_reset_cause = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

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
  tick_now = hello_world::tick::GetTickMs();
  TransmitWheelFrame();
  TransmitSteerFrame();
  TransmitLinkFrame(); /* 双板帧：0x101，走 CAN2 */
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
  /* 先把这一帧抄一份给 Ozone，再发（否则看到的是下一帧的内容） */
  for (uint8_t i = 0U; i < 8U; i++)
  {
    g_steer_tx_data[i] = tx_data[i];
  }
  g_steer_tx_cnt = g_steer_tx_cnt + 1U;

  hello_world::motor::InputType in_type = GetMotorSteer(0)->get_input_type();
  uint32_t tx_id =
      hello_world::motor::GM6020::GetTxId(GetMotorSteer(0)->id(), in_type);
  g_steer_tx_id = tx_id;

  if (CAN_Send_Msg(&hcan1, tx_data, tx_id, 8) == HAL_OK)
  {
    g_steer_tx_ok = g_steer_tx_ok + 1U;
  }
}

/**
 * @brief  双板帧：底盘 → 云台（0x101）
 * @note   走 CAN2（和 4 个 M3508 同一条总线），1 kHz。
 *         现在只填 ready/mode 两个真值，yaw/pitch 目标等任务 6/8 再接。
 */
static void TransmitLinkFrame(void)
{
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();

  comm->chassis_data().ready = true;
  comm->chassis_data().mode = 0;       /* 任务 6 换成状态机的模式枚举 */
  comm->chassis_data().yaw_ref = 0.0f; /* 任务 8 再给目标角度 */
  comm->chassis_data().pitch_ref = 0.0f;

  uint8_t frame[8] = {0};
  comm->encode(frame);
  (void)CAN_Send_Msg(&hcan2, frame, comm->tx_id(), 8);
  link_tx_cnt = link_tx_cnt + 1U;
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

extern "C" void CommTaskNotifyCommOk(void)
{
  /* 只记"有效通讯发生的时刻"，绝不在这里喂狗：
   * 本函数跑在 CAN 接收中断里，中断活着不代表主循环活着。 */
  g_comm_armed = true;
  g_last_comm_tick = hello_world::tick::GetTickMs();
}

extern "C" void CommTaskWatchdog(void)
{
  /* 故障注入：Ozone 把 g_dbg_hang 写 1，本函数卡死不再喂狗 */
  if (g_dbg_hang != 0U)
  {
    for (;;)
    {
    }
  }

  /* Ozone 只能 watch 变量、不能 watch 函数返回值 */
  tick_now = hello_world::tick::GetTickMs();

  /* 两条判据同时成立才喂狗：主循环活着（本函数就是主循环调的）+
   * 云台 0x102 新鲜（含 rc_online）。开机 2 s 内放宽。 */
  const bool comm_fresh =
      g_comm_armed && ((tick_now - g_last_comm_tick) < kCommTimeoutMs);
  const bool in_grace = (tick_now - g_boot_tick) < kStartupGraceMs;

  if (comm_fresh || in_grace)
  {
    (void)HAL_IWDG_Refresh(&hiwdg);
  }
}