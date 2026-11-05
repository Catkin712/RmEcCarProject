/* ============================================================================
 * 【新人任务】实现 robot.hpp 中声明的状态机。
 *
 * 你要做：
 *   1. 周期入口：按「更新数据 → 状态机迁移 → 生成指令 → 下发」的顺序组织 update()。
 *   2. 三态实现：Dead 时输出全零并等待就绪条件；Resurrection 时做自检与复位（如等待
 *      电机使能、IMU 零漂收敛）；Working 时执行正常控制逻辑。
 *   3. 各工作模式（分离 / 随动 / 小陀螺）的指令生成分支。
 *
 * 约束 / 提示：
 *   - 状态切换时记得复位上一状态残留的指令，避免电机突变。
 *   - 所有下发指令统一经过本类，急停判断放在最外层，确保任何分支都先过急停。
 * ==========================================================================*/
/**
 *******************************************************************************
 * @file      : robot.cpp
 * @brief     : 状态机 + 摇杆映射
 *******************************************************************************
 */
#include "robot.hpp"

#include <cmath>

#include "gimbal_chassis_comm.hpp"   /* 新增：GimbalChassisComm 的完整定义 */
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"

namespace robot
{

    namespace
    {
        constexpr uint32_t kResurrectTicks = 100; ///< 自检保持 100 ms
        constexpr float kStickDeadband = 0.05f;
        constexpr float kMaxVx = 3.0f;         ///< m/s（待实测）
        constexpr float kMaxVy = 3.0f;         ///< m/s（待实测）
        constexpr float kMaxW = 6.0f;          ///< rad/s（待实测）
        constexpr float kStandstillVel = 1.0f; ///< 自检要求轮速低于此值

        constexpr uint8_t kSwitchUp = 1;
        constexpr uint8_t kSwitchDown = 2;
        constexpr uint8_t kSwitchMid = 3;
    } // namespace

    void Robot::init(void)
    {
        chassis_->init();
        state_ = State::kDead;
        kill_latched_ = false;
        state_ticks_ = 0;
        change_cnt_ = 0;
        cmd_vel_ = Chassis::Velocity{};
        chassis_->set_velocity(cmd_vel_);
        chassis_->set_enabled(false);
    }

    const char *Robot::state_name(void) const
    {
        switch (state_)
        {
        case State::kDead:
            return "Dead";
        case State::kResurrection:
            return "Resurrection";
        case State::kWorking:
            return "Working";
        }
        return "?";
    }

    void Robot::enter_state(State next)
    {
        if (next == state_)
            return;
        state_ = next;
        state_ticks_ = 0;
        ++change_cnt_;
        chassis_->set_enabled(false); /* 离开任何状态都先收力，Working 分支再打开 */
    }

    void Robot::read_rc(void)
    {
        auto *comm = GetGimbalChassisComm();
        const auto &d = comm->gimbal_data();

        rc_lv_ = d.rc_lv;
        rc_lh_ = d.rc_lh;
        rc_rv_ = d.rc_rv;
        rc_rh_ = d.rc_rh;
        rc_l_switch_ = static_cast<uint8_t>(d.l_switch);
        rc_r_switch_ = static_cast<uint8_t>(d.r_switch);

        /* 两个条件都要：遥控本身在线 + 链路没断 */
        rc_online_ = d.rc_online && !comm->isOffline();
    }

    Chassis::Velocity Robot::stick_to_velocity(void) const
    {
        auto deadband = [](float x)
        {
            return (fabsf(x) < kStickDeadband) ? 0.0f : x;
        };

        Chassis::Velocity v;
        /* 摇杆 → 底盘速度（2026-10-29 实测标定）：
         * - 左摇杆上下：上为正，与 vx（向前为正）同号，直接用；
         * - 左右平移：推左车往右走 → vy 取负；
         * - 自旋：推右摇杆向右车逆时针转 → w 取负（底盘约定逆时针为正）*/
        v.vx = deadband(rc_lv_) * kMaxVx;
        v.vy = -deadband(rc_lh_) * kMaxVy;
        v.w = -deadband(rc_rh_) * kMaxW;
        return v;
    }

    void Robot::update(void)
    {
        /* 1. 取遥控数据 */
        read_rc();

        /* 2. 拨杆语义 */
        kill_cmd_ = (rc_online_ && rc_r_switch_ == kSwitchDown);
        enable_cmd_ = (rc_online_ && rc_l_switch_ == kSwitchUp);

        if (kill_cmd_)
        {
            kill_latched_ = true;
        }
        else if (kill_latched_ && (rc_r_switch_ == kSwitchMid || rc_l_switch_ == kSwitchMid))
        {
            kill_latched_ = false; /* 回中位才解锁 */
        }

        /* 3. 迁移判定 */
        const bool must_dead = !rc_online_ || !enable_cmd_ || kill_latched_;

        if (must_dead)
        {
            enter_state(State::kDead);
        }
        else if (state_ == State::kDead)
        {
            enter_state(State::kResurrection);
        }

        switch (state_)
        {
        case State::kDead:
            cmd_vel_ = Chassis::Velocity{};
            chassis_->set_velocity(cmd_vel_);
            chassis_->set_enabled(false);
            break;

        case State::kResurrection:
        {
            bool still = true;
            for (int i = 0; i < Chassis::kWheelNum; ++i)
            {
                if (fabsf(GetMotorWheel(i)->vel()) > kStandstillVel)
                    still = false;
            }

            cmd_vel_ = Chassis::Velocity{};
            chassis_->set_velocity(cmd_vel_);
            chassis_->set_enabled(false);

            if (!still)
            {
                state_ticks_ = 0; /* 轮子还在转，重新计时 */
            }
            else if (++state_ticks_ >= kResurrectTicks)
            {
                enter_state(State::kWorking);
            }
            break;
        }

        case State::kWorking:
            cmd_vel_ = stick_to_velocity();
            chassis_->set_velocity(cmd_vel_);
            chassis_->set_enabled(true);
            break;
        }

        /* 4. 执行层：无论什么状态都跑一遍（Dead 时写 0） */
        chassis_->update();
    }

} // namespace robot