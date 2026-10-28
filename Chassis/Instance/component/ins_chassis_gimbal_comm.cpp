/**
 *******************************************************************************
 * @file      :ins_chassis_gimbal_comm.cpp
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
#include "ins_chassis_gimbal_comm.hpp"
#include "gimbal_chassis_comm.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 文件作用域而不是函数内 static：Ozone 只能 watch 变量，
 * 放全局后可以直接展开 g_comm.gimbal_data_ / rx_cnt_ / crc_err_cnt_。 */
static robot::GimbalChassisComm g_comm(robot::GimbalChassisComm::CodePart::Chassis, 0x101, 0x102);

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm() {
  /* 本板身份 / 本板发送 ID / 本板接收 ID
   *   底盘板：CodePart::Chassis, 0x101, 0x102
   *   云台板：CodePart::Gimbal,  0x102, 0x101 */
  return &g_comm;
}

/* Private function definitions ----------------------------------------------*/
