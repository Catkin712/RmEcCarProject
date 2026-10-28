/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-26 19:32:45
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-10-28 19:14:09
 * @FilePath: \RmEcCarProject\Chassis\src\pid.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pid.hpp"

namespace hello_world
{
    namespace pid
    {

        void Pid::init(const Config &cfg)
        {
            cfg_ = cfg;

            /* 参数兜底：不允许出现“除零”和“负限幅” */
            if (cfg_.ts <= 0.0f)
            {
                cfg_.ts = 1e-3f;
            }
            if (cfg_.out_lim < 0.0f)
            {
                cfg_.out_lim = -cfg_.out_lim;
            }
            if (cfg_.i_lim < 0.0f)
            {
                cfg_.i_lim = -cfg_.i_lim;
            }

            reset();
        }

        void Pid::reset(void)
        {
            err_ = 0.0f;
            err_last_ = 0.0f;
            i_term_ = 0.0f;
            output_ = 0.0f;
            saturated_ = false;
        }

        float Pid::calc(float ref, float fb)
        {
            err_ = ref - fb;
            float p_term = cfg_.kp * err_;

            float i_next = cfg_.ki * err_ * cfg_.ts;
            i_term_ += i_next;
            i_term_ = i_term_ > cfg_.i_lim ? cfg_.i_lim : i_term_;
            i_term_ = i_term_ < -cfg_.i_lim ? -cfg_.i_lim : i_term_;

            float d_term = cfg_.kd * (err_ - err_last_) / cfg_.ts;

            float out = p_term + i_term_ + d_term;
            if (out > cfg_.out_lim)
            {
                out = cfg_.out_lim;
                saturated_ = true;
            }
            else if (out < -cfg_.out_lim)
            {
                out = -cfg_.out_lim;
                saturated_ = true;
            }
            else saturated_ = false;

            err_last_ = err_;
            output_ = out;
            return output_;
        }

    } // namespace pid
} // namespace hello_world