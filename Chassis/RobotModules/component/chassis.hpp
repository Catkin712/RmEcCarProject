/**
 *******************************************************************************
 * @file      : chassis.hpp
 * @brief     : 四舵轮底盘：逆运动学 + 执行（舵角环 / 轮速环）
 *******************************************************************************
 */
#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

#include <cmath>

namespace robot
{

    class Chassis
    {
    public:
        static constexpr int kWheelNum = 4;

        struct Velocity
        {
            float vx = 0.0f; ///< m/s，向前为正
            float vy = 0.0f; ///< m/s，向左为正
            float w = 0.0f;  ///< rad/s，逆时针为正
        };

        struct WheelCmd
        {
            float angle = 0.0f; ///< 舵角目标 rad
            float speed = 0.0f; ///< 轮速目标 rad/s（输出端）
        };

        /* 实际下发给电机的量（Ozone 观察用） */
        struct WheelInput
        {
            float steer = 0.0f; ///< 舵电机电流给定
            float wheel = 0.0f; ///< 轮电机电流给定
        };

        /* 几何与限幅 */
        static constexpr float kWheelRadius = 0.066f;
        static constexpr float kHalfBase = 0.167f;
        static constexpr float kHalfTrack = 0.167f;
        static constexpr float kMaxWheelSpeed = 40.0f;
        static constexpr float kSpeedEps = 1e-3f;
        static constexpr float kTwoPi = 6.28318530718f;

        void init(void);

        /* 使能：false 时 update() 只写 0，绝不保留上一拍输出 */
        void set_enabled(bool en) { enabled_ = en; }
        bool enabled(void) const { return enabled_; }

        void set_velocity(const Velocity &v) { vel_ = v; }
        void set_lock(bool on) { lock_ = on; }
        const Velocity &velocity(void) const { return vel_; }
        const WheelCmd &wheel_cmd(int idx) const { return cmd_[idx]; }
        const WheelInput &wheel_input(int idx) const { return input_[idx]; }

        void update(void); ///< 1 ms：逆解 + 两环 PID + setInput
        void stop(void);   ///< 立刻把八个电机置 0

        /* 纯函数，可桌面单测（任务 5 已给） */
        static void Solve(const Velocity &v, const float current_angle[kWheelNum],
                          WheelCmd out[kWheelNum], bool lock = false);

    private:
        bool lock_ = false;
        Velocity vel_{};
        WheelCmd cmd_[kWheelNum]{};
        WheelInput input_[kWheelNum]{};
        bool enabled_ = false;
    };

    namespace detail
    {
        constexpr float kPi = 3.14159265358979f;
        constexpr float kHalfPi = kPi * 0.5f;

        inline float WrapPi(float x)
        {
            return x - 2.0f * kPi * roundf(x * (1.0f / (2.0f * kPi)));
        }
    } // namespace detail

    inline void Chassis::Solve(const Velocity &v, const float cur[kWheelNum],
                               WheelCmd out[kWheelNum], bool lock)
    {
        static constexpr float kPos[kWheelNum][2] = {
            {+kHalfBase, +kHalfTrack}, /* 0 左前 */
            {-kHalfBase, +kHalfTrack}, /* 1 左后 */
            {-kHalfBase, -kHalfTrack}, /* 2 右后 */
            {+kHalfBase, -kHalfTrack}, /* 3 右前 */
        };

        if (lock) // 进自锁
        {
            for (int i = 0; i < kWheelNum; ++i)
            {
                out[i].angle = detail::WrapPi(atan2f(kPos[i][1], kPos[i][0]) + detail::kHalfPi);
                out[i].speed = 0.0f;
            }

            return;
        }

        for (int i = 0; i < kWheelNum; ++i)
        {
            const float vx = v.vx - v.w * kPos[i][1];
            const float vy = v.vy + v.w * kPos[i][0];
            const float speed = sqrtf(vx * vx + vy * vy) / kWheelRadius;

            if (speed < kSpeedEps)
            {
                out[i].angle = detail::WrapPi(cur[i]);
                out[i].speed = 0.0f;
                continue;
            }

            float angle = atan2f(vy, vx);
            if (fabsf(detail::WrapPi(angle - cur[i])) > detail::kHalfPi)
            {
                angle += detail::kPi;
                out[i].speed = -speed;
            }
            else
            {
                out[i].speed = speed;
            }
            out[i].angle = detail::WrapPi(angle);

            if (out[i].speed > kMaxWheelSpeed)
                out[i].speed = kMaxWheelSpeed;
            if (out[i].speed < -kMaxWheelSpeed)
                out[i].speed = -kMaxWheelSpeed;
        }
    }

} // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */