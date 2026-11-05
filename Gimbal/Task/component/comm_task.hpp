/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-19 19:38:57
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-10-28 21:15:36
 * @FilePath: \Project_Steer\RmEcCarProject\Gimbal\Task\component\comm_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :comm_task.hpp
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
#ifndef GIMBAL_TASK_COMM_TASK_HPP_
#define GIMBAL_TASK_COMM_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif
    /* Exported function prototypes ----------------------------------------------*/
    void CommTaskInit(void);
    void CommTask(void);
    void GimbalTask(void);           /* 云台关节环：1 kHz，当前为只读自检 */
    void CommTaskWatchdog(void);     /* 主循环每圈调用：决定这一圈喂不喂 */
    void CommTaskNotifyCommOk(void); /* 解包成功处调用：置 armed 并喂狗 */
#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_TASK_COMM_TASK_HPP_ */
