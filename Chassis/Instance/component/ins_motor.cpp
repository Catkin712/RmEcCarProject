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
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorWheel(int idx) {
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
  (void)idx;
  return nullptr;
}

hello_world::motor::Motor *GetMotorSteer(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号舵电机（GM6020）单例，做法同 GetMotorWheel()。
   * 舵电机控制的是轮子的转向角（舵向轮底盘），与轮电机（驱动轮速）配合完成运动；
   * 设备类见 inc/motor_GM6020.hpp，工厂 MotorType::kGM6020。
   */
  (void)idx;
  return nullptr;
}

/* Private function definitions ----------------------------------------------*/
