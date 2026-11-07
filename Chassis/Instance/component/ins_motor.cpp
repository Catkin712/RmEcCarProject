/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace
{
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;

  /* 轮电机（M3508）配置表
   *
   * 约定 ：本表下标 idx 与电机 CAN id 的关系为 id = idx + 1。
   * 位置命名：0=左前, 1=左后, 2=右后, 3=右前。
   */
  const hello_world::motor::OptionalParams kWheelParams[kWheelNum] = {
      /* idx0 左前 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf, .dir = hello_world::motor::Dir::kDirRev},
      /* idx1 左后 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf, .dir = hello_world::motor::Dir::kDirRev},
      /* idx2 右后 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
      /* idx3 右前 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
  };

  /* 舵电机（GM6020）配置表 */
  const hello_world::motor::OptionalParams kSteerParams[kSteerNum] = {
      /* idx0 左前 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = -0.516184628f},
      /* idx1 左后 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = 1.06546915f},
      /* idx2 右后 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = -0.0117657688f},
      /* idx3 右前 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = 1.59693146f},
  };
} // namespace
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static hello_world::motor::M3508 wheels[kWheelNum];
static hello_world::motor::GM6020 steers[kSteerNum];
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *
GetMotorWheel(int idx)
{
  HW_ASSERT(idx >= 0 && idx < kWheelNum, "Error: idx out of range");

  static bool initialized = false;
  if (!initialized)
  {
    for (int i = 0; i < kWheelNum; i++)
    {
      wheels[i].init(i + 1, kWheelParams[i]);
    }
    initialized = true;
  }
  return &wheels[idx];
}

hello_world::motor::Motor *GetMotorSteer(int idx)
{
  HW_ASSERT(idx >= 0 && idx < kSteerNum, "Error: idx out of range");
  static bool initialized = false;
  if (!initialized)
  {
    for (int i = 0; i < kSteerNum; i++)
    {
      steers[i].init(i + 1, kSteerParams[i]);
    }
    initialized = true;
  }
  return &steers[idx];
}

/* Private function definitions ----------------------------------------------*/
