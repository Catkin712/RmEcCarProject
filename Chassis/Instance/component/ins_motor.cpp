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
  constexpr uint8_t kYawId = 1;
  constexpr uint8_t kPitchId = 2;

  /* 轮电机（M3508）配置表
   *
   * 约定 ：本表下标 idx 与电机 CAN id 的关系为 id = idx + 1。
   * 位置命名：0=左前, 1=左后, 2=右后, 3=右前。
   *
   * 未填项：
   *   dir          —— 安装方向，上机标定。先用 kDirFwd 占位。
   *   angle_offset —— 零位，上机标定。先用 0 占位。
   */
  const hello_world::motor::OptionalParams kWheelParams[kWheelNum] = {
      /* idx0 左前 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf, .dir = hello_world::motor::Dir::kDirRev},
      /* idx1 左后 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
      /* idx2 右后 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
      /* idx3 右前 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
  };

  /* 舵电机（GM6020）配置表
   * input_type = kCurr：用电流帧 0x1FE 控制（±16384 ↔ ±3 A）。
   * 若电机在 RoboMaster Assist 里被设成"电流环"，它只认 0x1FE，对 0x1FF 静默忽略；
   * 这里显式走电流帧，两种情况都能动作。想回电压帧就把它改回 kRaw（默认值）。
   * 注意字段顺序必须与 OptionalParams 声明一致：input_type 在 angle_range 之前。 */
  const hello_world::motor::OptionalParams kSteerParams[kSteerNum] = {
      /* idx0 左前 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = -0.4994f},
      /* idx1 左后 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = -2.1026f},
      /* idx2 右后 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = 0.0767f},
      /* idx3 右前 */ {.input_type = hello_world::motor::InputType::kCurr, .angle_range = hello_world::motor::AngleRange::kNegPiToPosPi, .dir = hello_world::motor::Dir::kDirRev, .angle_offset = 1.4252f},
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
