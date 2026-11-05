/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-19 19:38:57
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-11-05 22:12:28
 * @FilePath: \Project_Steer\RmEcCarProject\Gimbal\inc\imu.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

#include "BMI088.hpp"
#include "ahrs.hpp"

namespace hello_world
{
    namespace imu
    {
        class Imu
        {
        public:
            Imu(void) = default;
            ~Imu(void) = default;

            bool init(const BMI088HWConfig &hw_config, const float rot_mat_flatten[9],
                      float samp_freq_hz);

            void update(void);

            float roll(void) const { return euler_[0]; }
            float pitch(void) const { return euler_[1]; }
            float yaw(void) const { return euler_[2]; }

            bool is_ok(void) const { return init_err_ == kBMI088ErrStateNoErr; }
            BMI088ErrState init_err(void) const { return init_err_; }
            bool is_calibrated(void) const { return calibrated_; }
            float calib_progress(void) const;

            const float *acc(void) const { return acc_; }
            const float *gyro(void) const { return gyro_; }
            const float *gyro_bias(void) const { return gyro_bias_; }

        private:
            BMI088 bmi088_{};
            ahrs::Mahony mahony_{};

            float acc_[3] = {0.0f, 0.0f, 0.0f};
            float gyro_[3] = {0.0f, 0.0f, 0.0f};
            float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
            float euler_[3] = {0.0f, 0.0f, 0.0f};

            uint32_t calib_cnt_ = 0;
            bool calibrated_ = false;
            BMI088ErrState init_err_ = kBMI088ErrStateNoErr;
        };
    } // namespace imu
} // namespace hello_world

#endif