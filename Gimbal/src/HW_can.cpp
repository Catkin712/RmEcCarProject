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
#include "gimbal_chassis_comm.hpp"     /* 完整类型：rx_id()/decode() */
#include "ins_chassis_gimbal_comm.hpp" /* GetGimbalChassisComm */
#include "ins_motor.hpp"               /* GetMotorYaw/GetMotorPitch 与 ID */

/* Private variables ---------------------------------------------------------*/
static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];

/* Ozone 观察用 */
static volatile uint32_t can1_rx_cnt = 0;          /* CAN1 取走的帧数（不限 ID） */
static volatile uint32_t can1_unknown_cnt = 0;     /* CAN1 未命中路由的帧数 */
static volatile uint32_t can1_last_unknown_id = 0; /* 最近一个未命中的 ID */
static volatile uint32_t link_rx_cnt = 0;          /* 命中双板帧 ID 的帧数（含 CRC 错） */

/* CAN2（pitch 达妙）的计数：CAN2 用 FIFO1，所以单独一套 */
static volatile uint32_t can2_rx_cnt = 0;
static volatile uint32_t can2_unknown_cnt = 0;
static volatile uint32_t can2_last_unknown_id = 0;
/* 两路达妙反馈的总命中数；只有这个涨，说明 ID 猜对了 */
static volatile uint32_t dm_rx_cnt = 0;

/* Exported function definitions ---------------------------------------------*/

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  /* 必须 ={0}：CAN_FilterTypeDef 里有 SlaveStartFilterBank，
   * F4 上 HAL_CAN_ConfigFilter 会拿它写 CAN_FMR.CAN2SB（决定 bank 归 CAN1 还是 CAN2）。
   * 不初始化就是栈上的垃圾值，滤波器归属会随机错乱。 */
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

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
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

  /* mailbox 是输出参数：HAL 不保留它，调用方也不需要知道用了哪个邮箱，
   * 所以用局部变量；传 NULL 会出问题，这里必须给真实地址。 */
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
      can1_rx_cnt = can1_rx_cnt + 1U;

      robot::GimbalChassisComm *comm = GetGimbalChassisComm();

      /* 云台 CAN1 上挂两类帧：
       *   1) 底盘发来的双板帧 0x101；
       *   2) yaw 达妙的反馈帧（id + 0x10）。
       * 先按 4 个条件筛，避免把扩展帧/远程帧/短帧喂给 decode。 */
      if (rx_header1.IDE == CAN_ID_STD && rx_header1.RTR == CAN_RTR_DATA &&
          rx_header1.DLC == 8U && rx_header1.StdId == comm->rx_id())
      {
        comm->decode(can1_rx_data);
        link_rx_cnt = link_rx_cnt + 1U; /* 含 CRC 错的帧；成功解包数看 g_comm.rx_cnt_ */
      }
      else if (rx_header1.IDE == CAN_ID_STD &&
               rx_header1.StdId == (0x10U + GetYawMotorId()))
      {
        /* 达妙反馈帧：数据交给电机解码（角度/速度/力矩/温度/状态码） */
        GetMotorYaw()->decode(can1_rx_data);
        dm_rx_cnt = dm_rx_cnt + 1U;
      }
      else
      {
        can1_unknown_cnt = can1_unknown_cnt + 1U;
        can1_last_unknown_id = rx_header1.StdId;
      }
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
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      can2_rx_cnt = can2_rx_cnt + 1U;

      /* 云台 CAN2 只挂 pitch 达妙（反馈 ID = id + 0x10） */
      if (rx_header2.IDE == CAN_ID_STD && rx_header2.RTR == CAN_RTR_DATA &&
          rx_header2.DLC == 8U &&
          rx_header2.StdId == (0x10U + GetPitchMotorId()))
      {
        GetMotorPitch()->decode(can2_rx_data);
        dm_rx_cnt = dm_rx_cnt + 1U;
      }
      else
      {
        can2_unknown_cnt = can2_unknown_cnt + 1U;
        can2_last_unknown_id = rx_header2.StdId;
      }
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
