/**
 *******************************************************************************
 * @file      :ins_imu.hpp
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
#ifndef CHASSIS_INSTANCE_INS_IMU_HPP_
#define CHASSIS_INSTANCE_INS_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取本板 IMU（姿态传感器）单例。 */
hello_world::imu::Imu *GetImu();

#endif /* CHASSIS_INSTANCE_INS_IMU_HPP_ */
