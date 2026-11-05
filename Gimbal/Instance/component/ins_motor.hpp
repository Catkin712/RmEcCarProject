/**
 *******************************************************************************
 * @file      :ins_motor.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_INSTANCE_INS_MOTOR_HPP_
#define GIMBAL_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取云台 yaw 关节电机单例（DM-J4310）。 */
hello_world::motor::Motor *GetMotorYaw();
/* 获取云台 pitch 关节电机单例（DM-J4310）。 */
hello_world::motor::Motor *GetMotorPitch();

/* 两个电机的 CAN ID（1~15）。HW_can 用它算反馈 ID（id + 0x10）做分发，
 * 避免把 ID 硬编码写死在两个文件里、改一个忘一个。 */
uint8_t GetYawMotorId(void);
uint8_t GetPitchMotorId(void);

#endif /* GIMBAL_INSTANCE_INS_MOTOR_HPP_ */
