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
/* 故意放在文件作用域（原来的写法是函数内 static）：
 * Ozone 只能 watch 变量，函数内 static 对象在调试器里不好展开；
 * 放全局后可以直接看 g_comm.gimbal_data_ / rx_cnt_ / crc_err_cnt_。
 * 和当初把 Imu 从函数内 static 改成 g_imu 是同一个原因。 */
static robot::GimbalChassisComm g_comm(robot::GimbalChassisComm::CodePart::Gimbal, 0x102, 0x101);

/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm *GetGimbalChassisComm()
{
  /* 本板身份 / 本板发送 ID / 本板接收 ID
   *   底盘板：CodePart::Chassis, 0x101, 0x102
   *   云台板：CodePart::Gimbal,  0x102, 0x101 */
  return &g_comm;
}

/* Private function definitions ----------------------------------------------*/
