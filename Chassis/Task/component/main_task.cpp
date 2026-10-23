/**
 *******************************************************************************
 * @file      :main_task.cpp
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
#include "main_task.hpp"

#include "comm_task.hpp"
#include "ins_motor.hpp"
#include "tim.h"
#include "tick.hpp"

/* Private variables ---------------------------------------------------------*/
uint32_t g_wheel_last_tick[4] = {0};
uint32_t g_steer_last_tick[4] = {0};

/* Private constants ---------------------------------------------------------*/
namespace
{
  /* 遥控器超时：100 ms 未收到新帧即判失联。*/
  constexpr uint32_t kRcTimeoutMs = 100U;

  /* 电机超时：50 ms 未收到反馈即判失联。*/
  constexpr uint32_t kMotorTimeoutMs = 50U;
}

namespace
{
  /* 轮电机、舵电机各 4 个。 */
  constexpr int kWheelNum = 4;
  constexpr int kSteerNum = 4;
}

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void)
{
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */
  for (int i = 0; i < kWheelNum; ++i) // 电机单例化
  {
    (void)GetMotorWheel(i);
  }
  for (int i = 0; i < kSteerNum; ++i)
  {
    (void)GetMotorSteer(i);
  }
  CommTaskInit(); // 通讯层：滤波 → Start → 使能中断
}

void MainTask(void)
{
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
}

/* Private function definitions ----------------------------------------------*/
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    MainTask();
    CommTask();
  }
}

/**
 * @brief  第 idx 号轮电机是否在线
 * @note   判定对象是"该电机的反馈"，不是"CAN2 有没有帧"。
 *         三个电机在发、一个坏了时，全局计数照样增长，只有逐电机判定能发现。
 */
bool WheelIsOnline(int idx)
{
  HW_ASSERT(idx >= 0 && idx < 4, "WheelIsOnline: idx out of range: %d", idx);
  return (hello_world::tick::GetTickMs() - g_wheel_last_tick[idx]) <
         kMotorTimeoutMs;
}

bool SteerIsOnline(int idx)
{
  HW_ASSERT(idx >= 0 && idx < 4, "SteerIsOnline: idx out of range: %d", idx);
  return (hello_world::tick::GetTickMs() - g_steer_last_tick[idx]) <
         kMotorTimeoutMs;
}