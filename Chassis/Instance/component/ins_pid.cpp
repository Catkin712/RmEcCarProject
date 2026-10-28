/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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
#include "ins_pid.hpp"

#include "assert.hpp" /* HW_ASSER */
#include "pid.hpp"    /* Pid 的完整定义 */
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

namespace
{
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;

  /*
   * 单位：
   *   轮速环：ref/fb 单位 rad/s，输出按 M3508 raw 给（±16384）
   *   舵角环：ref/fb 单位 rad，  输出按 GM6020 raw 给（±25000）
   * 写法：指定初始化器，字段顺序要跟 Config 声明顺序一致 */
  const hello_world::pid::Pid::Config kWheelCfg[kWheelNum] = {
      /* idx0 左前 */ {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
      /* idx1 左后 */ {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
      /* idx2 右后 */ {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
      /* idx3 右前 */ {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
  };

  const hello_world::pid::Pid::Config kSteerCfg[kSteerNum] = {
      /* 四个舵 */
      {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
      {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
      {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
      {.ts = 0.001f, .out_lim = 0.0f, .i_lim = 0.0f},
  };
} // namespace
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::pid::Pid *GetPidMotorWheel(int idx)
{
  HW_ASSERT(idx >= 0 && idx < kWheelNum, "GetPidMotorWheel: idx out of range");

  static hello_world::pid::Pid wheelsPid[kWheelNum];
  static bool initialized = false;
  if (!initialized)
  {
    initialized = true;
    for (int i = 0; i < kWheelNum; i++)
    {
      wheelsPid[i].init(kWheelCfg[i]);
    }
  }

  return &wheelsPid[idx];
}

hello_world::pid::Pid *GetPidMotorSteer(int idx)
{
  HW_ASSERT(idx >= 0 && idx < kSteerNum, "GetPidMotorSteer: idx out of range");

  static hello_world::pid::Pid steersPid[kSteerNum];
  static bool initialized = false;
  if (!initialized)
  {
    for (int i = 0; i < kSteerNum; i++)
    {
      steersPid[i].init(kSteerCfg[i]);
    }
  }

  return &steersPid[idx];
}

/* Private function definitions ----------------------------------------------*/
