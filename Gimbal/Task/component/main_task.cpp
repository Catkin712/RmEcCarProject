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
#include "tim.h" /* htim6 */

/* Exported function definitions ---------------------------------------------*/
void MainTaskInit(void) {
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */

  /* UART 接收已从底板迁移；CAN 滤波器等其余通讯初始化仍待任务 1。 */
  CommTaskInit();
}

void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief  1 ms 周期中断入口（TIM6）
 * @note   main.c 里同时开了 htim2 和 htim6 的更新中断：TIM2 是 32 位自由计数器
 *         （周期约 51 s），所以必须判 htim6，否则会被误当成 1 ms tick。
 *         HAL 回调是 C 链接，必须用 extern "C"。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    MainTask();  /* 控制链路（现在是空壳） */
    CommTask();  /* 通讯链路：把 RC 打包成 0x102 发出去 */
  }
}
