/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-19 19:38:57
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-10-29 21:16:02
 * @FilePath: \Project_Steer\RmEcCarProject\Chassis\Instance\component\ins_robot.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      : ins_robot.cpp
 * @brief     : 底盘 Robot / Chassis 单例
 *******************************************************************************
 */
#include "ins_robot.hpp"

#include "chassis.hpp"
#include "robot.hpp"

/* Private variables ---------------------------------------------------------*/
static robot::Chassis g_chassis;
static robot::Robot g_robot(&g_chassis);

/* Exported function definitions ---------------------------------------------*/
robot::Robot *GetRobot()
{
  return &g_robot;
}

robot::Chassis *GetChassis()
{
  return &g_chassis;
}