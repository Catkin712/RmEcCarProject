/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-11-05 22:13:10
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-11-05 22:13:15
 * @FilePath: \Project_Steer\RmEcCarProject\Gimbal\src\imu.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "imu.hpp"

namespace hello_world
{
    namespace imu
    {
        namespace
        {
            constexpr uint32_t kCalibSamples = 2000u; // 1 kHz 下 2 秒
            constexpr float kMahonyKp = 0.5f;
            constexpr float kMahonyKi = 0.0f;
        }

        bool Imu::init(const BMI088HWConfig &hw_config, const float rot_mat_flatten[9],
                       float samp_freq_hz)
        {
            bmi088_.init(hw_config, rot_mat_flatten);
            init_err_ = bmi088_.imuInit(false);
            if (init_err_ != kBMI088ErrStateNoErr)
            {
                return false;
            }

            mahony_.init(samp_freq_hz, kMahonyKp, kMahonyKi);

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
            bmi088_.getData(acc_, gyro_, nullptr);

            if (!calibrated_)
            {
                for (int i = 0; i < 3; ++i)
                {
                    gyro_bias_[i] += gyro_[i];
                }
                ++calib_cnt_;

                if (calib_cnt_ >= kCalibSamples)
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        gyro_bias_[i] /= static_cast<float>(calib_cnt_);
                    }
                    calibrated_ = true;
                }
                return; // 标定期间不送 Mahony
            }

            float gyro_corrected[3];
            for (int i = 0; i < 3; ++i)
            {
                gyro_corrected[i] = gyro_[i] - gyro_bias_[i];
            }

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