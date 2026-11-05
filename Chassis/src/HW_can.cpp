/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_can.hpp"

#include "can.h"
#include "ins_motor.hpp"    /* GetMotorWheel / GetMotorSteer */
#include "motor_GM6020.hpp" /* GM6020::GetRxId */
#include "motor_M3508.hpp"  /* M3508::GetRxId */
#include "tick.hpp"

#include "comm_task.hpp"               /* CommTaskNotifyCommOk */
#include "ins_chassis_gimbal_comm.hpp" /* GetGimbalChassisComm */
#include "gimbal_chassis_comm.hpp"     /* 完整类型：rx_id()/decode()/gimbal_data() */

extern uint32_t g_wheel_last_tick[4];
extern uint32_t g_steer_last_tick[4];

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];

// Ozone观察用
static volatile uint32_t can1_rx_cnt = 0;          /* CAN1 成功取走的帧数 */
static volatile uint32_t can2_rx_cnt = 0;          /* CAN2 成功取走的帧数 */
static volatile uint32_t can1_unknown_cnt = 0;     /* CAN1 未命中任何路由的帧数 */
static volatile uint32_t can2_unknown_cnt = 0;     /* CAN2 未命中任何路由的帧数 */
static volatile uint32_t can1_last_unknown_id = 0; /* 最近一个未命中的 ID */
static volatile uint32_t can2_last_unknown_id = 0;
static volatile uint32_t link_rx_cnt = 0; /* 0x102 双板帧的有效计数（Ozone 看） */
static volatile uint32_t link_joint_rx_cnt = 0; /* 0x103 关节帧的有效计数 */
constexpr uint32_t kGimbalJointRxId = 0x103U;

namespace
{
  /* 与 ins_motor.cpp 里的同名常量各自独立。
   * 这是有意的重复：路由层要知道"遍历几个"，但不想为此改动 Instance 层接口。
   * 数量变化时两处都要改——这是当前设计的已知代价。 */
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;
} // namespace

/* Private function prototypes -----------------------------------------------*/
static void RouteCan1Frame(const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8]);
static void RouteCan2Frame(const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8]);
/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  /* 必须 ={0}：CAN_FilterTypeDef 里有 SlaveStartFilterBank，
   * F4 上 HAL_CAN_ConfigFilter 会拿它写 CAN_FMR.CAN2SB（决定 bank 归 CAN1 还是 CAN2）。
   * 不初始化就是栈上的垃圾值：CAN2 可能一个 bank 都分不到 → 只发不收。 */
  CAN_FilterTypeDef can_filter = {0};

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14; /* bank 0~13 归 CAN1，14~27 归 CAN2 */

  if (hcan == &hcan1)
  {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  }
  else if (hcan == &hcan2)
  {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  }
  else
  {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef CAN_Send_Msg(CAN_HandleTypeDef *hcan, const uint8_t *msg,
                               uint32_t id, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;

  uint32_t mailbox = 0;

  return HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &mailbox);
}

/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK)
    {
      can1_rx_cnt = can1_rx_cnt + 1U;            /* 新增：成功取走一帧才计数 */
      RouteCan1Frame(&rx_header1, can1_rx_data); /* 新增：分发 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK)
    {
      can2_rx_cnt = can2_rx_cnt + 1U;            /* 新增：成功取走一帧才计数 */
      RouteCan2Frame(&rx_header2, can2_rx_data); /* 新增：分发 */
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* Private function definitions ----------------------------------------------*/

static void RouteCan1Frame(const CAN_RxHeaderTypeDef *header,
                           const uint8_t data[8])
{
  /* ① 只处理标准数据帧。
   *    IDE != CAN_ID_STD：扩展帧（29 位 ID），本项目不用。
   *    RTR != CAN_RTR_DATA：远程帧，没有数据段。 */
  if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA)
  {
    can1_unknown_cnt = can1_unknown_cnt + 1U;
    can1_last_unknown_id = header->StdId;
    return;
  }

  /* ② 电机反馈固定 8 字节；短帧不喂给 decode（decode 无条件读 data[0..7]）。 */
  if (header->DLC != 8U)
  {
    can1_unknown_cnt = can1_unknown_cnt + 1U;
    can1_last_unknown_id = header->StdId;
    return;
  }

  /* ③ 遍历舵电机，用电机自己记录的 id 反查接收 ID。*/
  for (int idx = 0; idx < kSteerNum; ++idx)
  {
    hello_world::motor::Motor *motor = GetMotorSteer(idx);
    if (hello_world::motor::GM6020::GetRxId(motor->id()) == header->StdId) // 命中
    {
      g_steer_last_tick[idx] = hello_world::tick::GetTickMs();
      (void)motor->decode(data);
      return;
    }
  }

  /* ④ 未命中：计数 + 记录 ID*/
  can1_unknown_cnt = can1_unknown_cnt + 1U;
  can1_last_unknown_id = header->StdId;
}

static void RouteCan2Frame(const CAN_RxHeaderTypeDef *header, //
                           const uint8_t data[8])
{
  if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA)
  {
    can2_unknown_cnt = can2_unknown_cnt + 1U;
    can2_last_unknown_id = header->StdId;
    return;
  }

  if (header->DLC != 8U)
  {
    can2_unknown_cnt = can2_unknown_cnt + 1U;
    can2_last_unknown_id = header->StdId;
    return;
  }

  /* 双板通讯帧：先于电机判断，避免每个电机都比一遍 */
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  if (header->StdId == comm->rx_id())
  {
    comm->decode(data);
    link_rx_cnt = link_rx_cnt + 1U;

    /* 收到帧 ≠ 遥控在线：云台复位后的宽限期里也会发帧（rc_online=false） */
    if (comm->gimbal_data().rc_online)
    {
      CommTaskNotifyCommOk();
    }
    return;
  }

  /* 云台→底盘的关节状态帧（0x103）：底盘用 yaw_rel 做“跟随云台” */
  if (header->StdId == kGimbalJointRxId)
  {
    if (comm->decodeJoint(data))
    {
      link_joint_rx_cnt = link_joint_rx_cnt + 1U;
    }
    return;
  }

  /* 轮电机*/
  for (int idx = 0; idx < kWheelNum; ++idx)
  {
    hello_world::motor::Motor *motor = GetMotorWheel(idx);
    if (hello_world::motor::M3508::GetRxId(motor->id()) == header->StdId)
    {
      g_wheel_last_tick[idx] = hello_world::tick::GetTickMs();
      (void)motor->decode(data);
      return;
    }
  }

  can2_unknown_cnt = can2_unknown_cnt + 1U;
  can2_last_unknown_id = header->StdId;
}