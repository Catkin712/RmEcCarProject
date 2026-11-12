/**
 *******************************************************************************
 * @file      : chassis.cpp
 * @brief     : 底盘执行层：逆解 → 舵角环 / 轮速环 → setInput
 *******************************************************************************
 */
#include "chassis.hpp"

#include "base.hpp"
#include "ins_motor.hpp"
#include "ins_pid.hpp"
#include "pid.hpp"

/* 调试镜像：Ozone 直接 watch 这几个数组（static 成员在 Ozone 里不好展开） */
float g_steer_angle_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};  /* 舵电机反馈角 rad */
float g_steer_target_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f}; /* 展开后的目标角 rad */
float g_steer_out_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};    /* 舵角环输出 raw */
float g_wheel_out_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};    /* 轮速环输出 raw */
float g_wheel_cmd_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};    /* 轮速环目标 rad/s */
float g_wheel_vel_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};    /* 轮速环反馈 rad/s */
float g_wheel_i_dbg[4] = {0.0f, 0.0f, 0.0f, 0.0f};      /* 轮速积分项 raw */
float g_chassis_vel_dbg[3] = {0.0f, 0.0f, 0.0f};        /* vx, vy, w */
uint8_t g_chassis_enabled_dbg = 0;                      /* enabled_ 的镜像 */

namespace robot
{

    void Chassis::init(void)
    {
        vel_ = Velocity{};
        enabled_ = false;
        for (int i = 0; i < kWheelNum; ++i)
        {
            cmd_[i] = WheelCmd{};
            input_[i] = WheelInput{};
        }
        stop();
    }

    void Chassis::stop(void)
    {
        for (int i = 0; i < kWheelNum; ++i)
        {
            if (GetMotorSteer(i) != nullptr)
                GetMotorSteer(i)->setInput(0);
            if (GetMotorWheel(i) != nullptr)
                GetMotorWheel(i)->setInput(0);
            input_[i].steer = 0.0f;
            input_[i].wheel = 0.0f;
            GetPidMotorWheel(i)->reset(); /* 失能后不保留起步积分 */
            g_wheel_i_dbg[i] = 0.0f;
            g_wheel_out_dbg[i] = 0.0f;
        }
    }

    void Chassis::update(void)
    {
        /* 0. 状态镜像（Ozone 用） */
        g_chassis_vel_dbg[0] = vel_.vx;
        g_chassis_vel_dbg[1] = vel_.vy;
        g_chassis_vel_dbg[2] = vel_.w;
        g_chassis_enabled_dbg = enabled_ ? 1U : 0U;

        /* 1. 读四个舵的实际角度（已按 dir / angle_offset 处理） */
        float cur[kWheelNum] = {0.0f};
        for (int i = 0; i < kWheelNum; ++i)
        {
            cur[i] = GetMotorSteer(i)->angle();
            g_steer_angle_dbg[i] = cur[i];
        }

        /* 2. 逆解：得到每轮的目标舵角与目标轮速 */
        Solve(vel_, cur, cmd_, lock_);

        /* 目标角展开到当前角附近，避免 ±π 跳变（Ozone 里看到的就是真正送进 PID 的 ref） */
        for (int i = 0; i < kWheelNum; ++i)
        {
            g_steer_target_dbg[i] = hello_world::HandleAngleCross0Rad(cmd_[i].angle, cur[i]);
        }

        /* 3. 未使能：只写 0（急停/未就绪都走这里） */
        if (!enabled_)
        {
            stop();
            return;
        }

        /* 4. 两个闭环 */
        for (int i = 0; i < kWheelNum; ++i)
        {
            /* 舵角环：目标展开到当前角附近，避免 ±π 跳变 */
            input_[i].steer = GetPidMotorSteer(i)->calc(g_steer_target_dbg[i], cur[i]);
            GetMotorSteer(i)->setInput(input_[i].steer);

            /* 轮速 PI：零速目标（含自锁）每拍清历史，不跨拍积累积分。
             * 仍执行 calc，让比例项对实际残余轮速提供制动；失能走 stop。 */
            auto *wheel_pid = GetPidMotorWheel(i);
            if (fabsf(cmd_[i].speed) < kSpeedEps)
            {
                wheel_pid->reset();
            }
            input_[i].wheel = wheel_pid->calc(cmd_[i].speed,
                                             GetMotorWheel(i)->vel());
            g_wheel_i_dbg[i] = wheel_pid->i_term();
            GetMotorWheel(i)->setInput(input_[i].wheel);

            g_steer_out_dbg[i] = input_[i].steer;
            g_wheel_out_dbg[i] = input_[i].wheel;
            g_wheel_cmd_dbg[i] = cmd_[i].speed;
            g_wheel_vel_dbg[i] = GetMotorWheel(i)->vel();
        }
    }

} // namespace robot
