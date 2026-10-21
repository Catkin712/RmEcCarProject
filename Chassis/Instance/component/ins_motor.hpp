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
#ifndef CHASSIS_INSTANCE_INS_MOTOR_HPP_
#define CHASSIS_INSTANCE_INS_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* 获取第 idx 号轮电机单例（M3508，共 4 个，idx ∈ [0,4)）。
 *
 * 【越界契约】idx 越界由 HW_ASSERT 拦截（关中断死循环）。
 *   因此本工程禁止定义 USE_CUSTOM_DEFINE_ASSERT=0：
 *   一旦断言被关闭，越界将返回数组外地址（野指针），
 *   调用方解引用会静默破坏内存，且不会有任何报错。
 *
 * 【id 约定】idx 与电机 CAN id 的对应关系为 id = idx + 1（idx0↔id1 …）。
 *   idx 按安装位置命名：0=左前, 1=左后, 2=右后, 3=右前。
 *   若实机 id 不同，必须同时修改 ins_motor.cpp 的配置表与
 *   HW_can.cpp 的反馈路由换算。否则电机不响应，且无任何报错。
 *
 * 【调用时机】必须在任何中断使能之前完成首次调用（初始化会在此惰性执行）。
 */
hello_world::motor::Motor *GetMotorWheel(int idx);
/* 获取第 idx 号舵电机单例（GM6020，共 4 个，idx ∈ [0,4)）。
 * 【越界契约】idx 越界由 HW_ASSERT 拦截（关中断死循环）。
 * 【id 约定】id = idx + 1（idx0↔id1 …）。注意 GM6020 的 id 范围是 1~7
 * 【调用时机】必须在任何中断使能之前完成首次调用。
 */
hello_world::motor::Motor *GetMotorSteer(int idx);

#endif /* CHASSIS_INSTANCE_INS_MOTOR_HPP_ */
