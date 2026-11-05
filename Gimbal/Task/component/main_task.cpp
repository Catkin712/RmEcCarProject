/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-19 19:38:57
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-11-05 22:48:38
 * @FilePath: \Project_Steer\RmEcCarProject\Gimbal\Task\component\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "main_task.hpp"
#include "comm_task.hpp"
#include "imu.hpp"
#include "ins_imu.hpp"
#include "spi.h" /* hspi1 */
#include "tim.h" /* htim6 */

/* 调试镜像：Ozone 只能看变量，看不到函数返回值 */
static float g_imu_roll_dbg = 0.0f;
static float g_imu_pitch_dbg = 0.0f;
static float g_imu_yaw_dbg = 0.0f;
static uint32_t g_imu_init_err_dbg = 0u;
static uint8_t g_imu_calibrated_dbg = 0u;
static float g_imu_calib_progress_dbg = 0.0f;
static float g_imu_gyro_bias_dbg[3] = {0.0f, 0.0f, 0.0f};
static uint32_t g_imu_update_cnt = 0u;
static float g_imu_acc_dbg[3] = {0.0f, 0.0f, 0.0f};
static float g_imu_gyro_dbg[3] = {0.0f, 0.0f, 0.0f};

void MainTaskInit(void)
{
  CommTaskInit();

  /* IMU 初始化：只在这里做一次，不能放中断 */
  hello_world::imu::BMI088HWConfig cfg;
  cfg.hspi = &hspi1;

  /* 片选沿用 C 型板惯例：PA4 = 加速度计，PB0 = 陀螺仪。
   * 若 init_err 非 0（1=加速度计没找到，2=陀螺仪没找到），先查这两根线。 */
  cfg.acc_cs_port = GPIOA;
  cfg.acc_cs_pin = GPIO_PIN_4;
  cfg.gyro_cs_port = GPIOB;
  cfg.gyro_cs_pin = GPIO_PIN_0;

  /* 芯片翻面（z 朝下）并绕 x 轴多转 24.7°，绕 x 转 204.66° 校正。
   * 依据：水平静置 acc = (-0.15, -4.09, -8.90)。 */
  const float kRotFix[9] = {1.0f,      0.0f,      0.0f,
                            0.0f,     -0.9087f,    0.4171f,
                            0.0f,     -0.4171f,   -0.9087f};

  (void)GetImu()->init(cfg, kRotFix, 1000.0f); /* 1 kHz，与 TIM6 一致 */
  g_imu_init_err_dbg = static_cast<uint32_t>(GetImu()->init_err());
}

void MainTask(void)
{
  hello_world::imu::Imu *imu = GetImu();
  if (imu->is_ok())
  {
    imu->update();

    g_imu_roll_dbg = imu->roll();
    g_imu_pitch_dbg = imu->pitch();
    g_imu_yaw_dbg = imu->yaw();
    g_imu_calibrated_dbg = imu->is_calibrated() ? 1u : 0u;
    g_imu_calib_progress_dbg = imu->calib_progress();
    const float *bias = imu->gyro_bias();
    g_imu_gyro_bias_dbg[0] = bias[0];
    g_imu_gyro_bias_dbg[1] = bias[1];
    g_imu_gyro_bias_dbg[2] = bias[2];
    ++g_imu_update_cnt;

    const float *acc = imu->acc();
    g_imu_acc_dbg[0] = acc[0];
    g_imu_acc_dbg[1] = acc[1];
    g_imu_acc_dbg[2] = acc[2];

    const float *gyro_raw = imu->gyro();
    g_imu_gyro_dbg[0] = gyro_raw[0];
    g_imu_gyro_dbg[1] = gyro_raw[1];
    g_imu_gyro_dbg[2] = gyro_raw[2];
  }

  GimbalTask();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6)
  {
    MainTask(); /* 1 ms 控制链路：IMU + 云台环 */
    CommTask(); /* 通讯链路 */
  }
}