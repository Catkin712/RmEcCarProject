/**
 *******************************************************************************
 * @file      : imu.hpp
 * @brief     : IMU 设备封装（新人填写）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-12-01      Jinletian       1. 初版编写完成
 *  V2.0.0      2026-09-18      Hello World     1. 考核骨架：掏空 Imu 封装类，
 *                                                只保留 BMI088 芯片驱动 + Mahony 姿态
 *******************************************************************************
 * @attention :
 *  本文件已掏空。新人需基于以下两个已提供的组件自行实现 Imu 封装类：
 *    - BMI088：芯片级原始读取（加速度 m/s² / 角速度 rad/s / 温度）
 *    - ahrs::Mahony：姿态解算（输入角速度 + 加速度，输出四元数/姿态角）
 *  需要新人实现：
 *    - 开机零漂计算（静止采样求角速度均值）
 *    - 姿态角输出 roll()/pitch()/yaw()
 *******************************************************************************
 *  Copyright (c) 2025 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_IMU_HPP_
#define HW_COMPONENTS_DEVICES_IMU_IMU_HPP_

/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"
#include "ahrs.hpp" /* 需要 ahrs::Mahony 的完整定义（组合成员） */

namespace hello_world
{
    namespace imu
    {

        class Imu
        {
        public:
            Imu(void) = default;
            ~Imu(void) = default;

            /**
             * @brief  IMU 初始化
             * @param  hw_config        SPI 句柄与两路片选
             * @param  rot_mat_flatten  芯片坐标系→机体坐标系 的旋转矩阵（展平 9 个元素）
             * @param  samp_freq_hz     update() 的调用频率，必须与实际调用周期一致
             * @retval true = 芯片初始化成功
             * @note   【空位 N 之外的另一处】失败时不要断言，返回 false 并让上层查询 init_err()
             */
            bool init(const BMI088HWConfig &hw_config, const float rot_mat_flatten[9],
                      float samp_freq_hz);

            /**
             * @brief  周期调用（建议 1 ms）：读芯片 → 扣零漂 → 更新姿态
             * @note   SPI 读是阻塞的，约 30~50 µs，不要在 1 ms 内调用多次
             */
            void update(void);

            /* 姿态角，单位 rad（Z-Y-X 欧拉角） */
            float roll(void) const { return euler_[0]; }
            float pitch(void) const { return euler_[1]; }
            float yaw(void) const { return euler_[2]; }

            /* 状态查询 */
            bool is_ok(void) const { return init_err_ == kBMI088ErrStateNoErr; }
            BMI088ErrState init_err(void) const { return init_err_; }
            bool is_calibrated(void) const { return calibrated_; }
            float calib_progress(void) const;

            /* 调试用：原始数据与零漂估计 */
            const float *acc(void) const { return acc_; }
            const float *gyro(void) const { return gyro_; }
            const float *gyro_bias(void) const { return gyro_bias_; }

        private:
            /* 组合：Imu "有一个" 芯片 和 "有一个" 解算器 */
            BMI088 bmi088_{};
            ahrs::Mahony mahony_{};

            float acc_[3] = {0.0f, 0.0f, 0.0f};       /* m/s² */
            float gyro_[3] = {0.0f, 0.0f, 0.0f};      /* rad/s */
            float gyro_bias_[3] = {0.0f, 0.0f, 0.0f}; /* rad/s，零漂估计值 */
            float euler_[3] = {0.0f, 0.0f, 0.0f};     /* rad */

            uint32_t calib_cnt_ = 0;
            bool calibrated_ = false;
            BMI088ErrState init_err_ = kBMI088ErrStateNoErr;
        };

    } // namespace imu
} // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_IMU_HPP_ */
