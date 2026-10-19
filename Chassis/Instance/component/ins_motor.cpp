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
   *
   * 未填项：
   *   dir          —— 安装方向，上机标定。先用 kDirFwd 占位。
   *   angle_offset —— 零位，上机标定。先用 0 占位。
   */
  const hello_world::motor::OptionalParams kWheelParams[kWheelNum] = {
      /* idx0 左前 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
      /* idx1 左后 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
      /* idx2 右后 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
      /* idx3 右前 */ {.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf},
  };

  /* 舵电机（GM6020）配置表*/
  const hello_world::motor::OptionalParams kSteerParams[kSteerNum] = {
      /* idx0 */ {.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi},
      /* idx1 */ {.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi},
      /* idx2 */ {.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi},
      /* idx3 */ {.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi},
  };
} // namespace
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx)
{
  /* TODO【新人任务】:
   * 返回第 idx 号轮电机（M3508）单例。
   *
   * 你要做：
   *   1. 静态定义 4 个轮电机对象并存入数组（可用 hello_world::motor::CreateMotor
   *      (hello_world::motor::MotorType::kM3508, id, opt) 工厂，或直接构造 M3508）。
   *   2. 校验 idx ∈ [0,4)，越界时用 HW_ASSERT 或直接断言。
   *   3. 返回对应元素的地址。
   *   4. 每个电机的 CAN id、方向、减速比、角度偏移按实机接线配置（见考核说明文档）。
   *
   * 约束 / 提示：
   *   - 电机设备类已提供：inc/motor_M3508.hpp（M3508）、inc/motor.hpp（CreateMotor 工厂、
   *     OptionalParams 配置项）。
   *   - 电机按「左前 / 左后 / 右后 / 右前」命名，id 与方向见实机接线图。
   */
  HW_ASSERT(idx >= 0 && idx < kWheelNum, "Error: idx out of range");

  static hello_world::motor::M3508 wheels[kWheelNum];
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
  /* TODO【新人任务】:
   * 返回第 idx 号舵电机（GM6020）单例，做法同 GetMotorWheel()。
   * 舵电机控制的是轮子的转向角（舵向轮底盘），与轮电机（驱动轮速）配合完成运动；
   * 设备类见 inc/motor_GM6020.hpp，工厂 MotorType::kGM6020。
   */
  HW_ASSERT(idx >= 0 && idx < kSteerNum, "Error: idx out of range");
  static hello_world::motor::GM6020 steers[kSteerNum];
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
