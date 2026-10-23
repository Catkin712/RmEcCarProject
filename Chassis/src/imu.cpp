/**
 *******************************************************************************
 * @file      : imu.cpp
 * @brief     : IMU 封装实现（BMI088 原始读取 + Mahony 姿态 + 开机零漂）
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "imu.hpp"

namespace hello_world
{
    namespace imu
    {

        /* Private constants ---------------------------------------------------------*/
        namespace
        {
            /* 零漂采样点数：1 kHz 下 2000 点 = 2 秒。
             * 均值噪声 ∝ 1/√N；2 秒是"精度够用 + 开机可接受"的折中。 */
            constexpr uint32_t kCalibSamples = 2000u; // 噪声约为1/45

            /* Mahony 参数：kp 决定"多久信任一次加速度"，ki 是积分项（一般先给 0）。
             * 先用保守默认值，上机看姿态跟随情况再调。 */
            constexpr float kMahonyKp = 0.5f;
            constexpr float kMahonyKi = 0.0f;
        } // namespace

        /* Exported function definitions ---------------------------------------------*/
        bool Imu::init(const BMI088HWConfig &hw_config, const float rot_mat_flatten[9],
                       float samp_freq_hz)
        {
            /* 记录硬件配置（只存指针，不与芯片通信，不阻塞） */
            bmi088_.init(hw_config, rot_mat_flatten);

            /* 真正与芯片通信：识别 ID、写量程/ODR、可选自检。
             *    注意 imuInit 内部会阻塞（含自检更慢），不能在中断里调用。 */
            init_err_ = bmi088_.imuInit(false);

            if (init_err_ != kBMI088ErrStateNoErr)
            {
                return false; /* 失败就退出*/
            }

            /* 姿态解算器初始化：采样频率必须与 update() 的实际调用频率一致 */
            mahony_.init(samp_freq_hz, kMahonyKp, kMahonyKi);

            /* 清状态 */
            calib_cnt_ = 0;
            calibrated_ = false;
            for (int i = 0; i < 3; ++i)
            {
                gyro_bias_[i] = 0.0f;
                euler_[i] = 0.0f;
            }
            return true;
        }

        void Imu::update(void)
        {
            /* 读芯片：acc 单位 m/s²（含重力），gyro 单位 rad/s */
            bmi088_.getData(acc_, gyro_, nullptr);

            /* 零漂采样阶段：只累加，不更新姿态 */
            if (!calibrated_)
            {
                for (int i = 0; i < 3; ++i)
                {
                    gyro_bias_[i] += gyro_[i];
                }
                ++calib_cnt_;

                if (calib_cnt_ >= kCalibSamples)
                {
                    /* 求均值 = 零漂估计 */
                    for (int i = 0; i < 3; ++i)
                    {
                        gyro_bias_[i] /= static_cast<float>(calib_cnt_);
                    }
                    calibrated_ = true;
                }
                return; /* 零漂未完成，不送 Mahony */
            }

            /* 扣除零漂 */
            float gyro_corrected[3];
            for (int i = 0; i < 3; ++i)
            {
                gyro_corrected[i] = gyro_[i] - gyro_bias_[i];
            }

            /* 姿态更新 + 取欧拉角 */
            mahony_.update(acc_, gyro_corrected);
            mahony_.getEulerAngle(euler_);
        }

        float Imu::calib_progress(void) const
        {
            if (calibrated_)
            {
                return 1.0f;
            }
            return static_cast<float>(calib_cnt_) / static_cast<float>(kCalibSamples);
        }

    } // namespace imu
} // namespace hello_world