/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-19 19:38:57
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-10-28 20:40:27
 * @FilePath: \Project_Steer\RmEcCarProject\RobotComponents\component\gimbal_chassis_comm.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"

#include <cmath>
#include "crc8.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace
{
  int8_t ToI8(float x);
  float FromI8(int8_t v);
  void PutI16(uint8_t *p, int16_t v);
  int16_t GetI16(const uint8_t *p);
  uint8_t Crc8(const uint8_t *p, uint32_t len);
} // namespace
/* Exported function definitions ---------------------------------------------*/
robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id)
{
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

void robot::GimbalChassisComm::encode(uint8_t tx_data[8])
{
  ++tx_seq_;
  if (code_part_ == CodePart::Gimbal)
  {
    /* ---------- G2C ---------- */
    tx_data[0] = static_cast<uint8_t>(ToI8(gimbal_data_.rc_lv));
    tx_data[1] = static_cast<uint8_t>(ToI8(gimbal_data_.rc_lh));
    tx_data[2] = static_cast<uint8_t>(ToI8(gimbal_data_.rc_rv));
    tx_data[3] = static_cast<uint8_t>(ToI8(gimbal_data_.rc_rh));
    tx_data[4] = static_cast<uint8_t>(ToI8(gimbal_data_.rc_wheel));

    /* 拨杆 + 标志位*/
    tx_data[5] = static_cast<uint8_t>(
        (gimbal_data_.l_switch & 0x03u) |
        ((gimbal_data_.r_switch & 0x03u) << 2) |
        (gimbal_data_.ready ? 0x10u : 0x00u) |
        (gimbal_data_.rc_online ? 0x20u : 0x00u));

    tx_data[6] = static_cast<uint8_t>(tx_seq_);
  }
  else
  {
    /* ---------- C2G ---------- */
    tx_data[0] = chassis_data_.ready ? 0x01u : 0x00u;
    tx_data[1] = static_cast<uint8_t>(chassis_data_.mode);

    PutI16(&tx_data[2], static_cast<int16_t>(roundf(chassis_data_.yaw_ref * 1e3f)));
    PutI16(&tx_data[4], static_cast<int16_t>(roundf(chassis_data_.pitch_ref * 1e3f)));

    tx_data[6] = static_cast<uint8_t>(tx_seq_);
  }

  tx_data[7] = Crc8(tx_data, 7);
}

void robot::GimbalChassisComm::decode(const uint8_t rx_data[8])
{
  /*    解码对方帧：把 8 字节按同样布局还原成对方语义，更新本地缓存，并刷新 last_rx_tick_。
   *   - 严格与 encode() 对称；任何一端偏移写错都会让对面读到错乱数据*/

  /* CRC */
  if (Crc8(rx_data, 7) != rx_data[7])
  {
    ++crc_err_cnt_;
    return;
  }

  if (code_part_ == CodePart::Chassis) /* 底盘收到 G2C */
  {
    gimbal_data_.rc_lv = FromI8(static_cast<int8_t>(rx_data[0]));
    gimbal_data_.rc_lh = FromI8(static_cast<int8_t>(rx_data[1]));
    gimbal_data_.rc_rv = FromI8(static_cast<int8_t>(rx_data[2]));
    gimbal_data_.rc_rh = FromI8(static_cast<int8_t>(rx_data[3]));
    gimbal_data_.rc_wheel = FromI8(static_cast<int8_t>(rx_data[4]));

    gimbal_data_.l_switch = static_cast<uint8_t>(rx_data[5] & 0x03u);
    gimbal_data_.r_switch = static_cast<uint8_t>((rx_data[5] >> 2) & 0x03u);
    gimbal_data_.ready = (rx_data[5] & 0x10u) != 0;
    gimbal_data_.rc_online = (rx_data[5] & 0x20u) != 0;
  }
  else /* 云台收到 C2G */
  {
    chassis_data_.ready = (rx_data[0] & 0x01u) != 0;
    chassis_data_.mode = rx_data[1];
    chassis_data_.yaw_ref = static_cast<float>(GetI16(rx_data + 2)) * 1e-3f;
    chassis_data_.pitch_ref = static_cast<float>(GetI16(rx_data + 4)) * 1e-3f;
  }

  rx_seq_ = rx_data[6];
  ++rx_cnt_;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

void robot::GimbalChassisComm::encodeJoint(uint8_t tx_data[8])
{
  ++tx_joint_seq_;
  if (code_part_ == CodePart::Gimbal)
  {
    /* yaw 关节角在 ±π 内，1e-3 rad/LSB 足够（int16 满量程 ±32 rad） */
    PutI16(&tx_data[0], static_cast<int16_t>(roundf(joint_data_.yaw_rel * 1e3f)));
    PutI16(&tx_data[2], static_cast<int16_t>(roundf(joint_data_.pitch * 1e3f)));
    tx_data[4] = joint_data_.mode;
    tx_data[5] = static_cast<uint8_t>((joint_data_.active ? 0x01u : 0x00u) |
                                      (joint_data_.aligning ? 0x02u : 0x00u));
    tx_data[6] = tx_joint_seq_;
  }
  else
  {
    for (int i = 0; i < 7; ++i)
    {
      tx_data[i] = 0;
    }
  }
  tx_data[7] = Crc8(tx_data, 7);
}

bool robot::GimbalChassisComm::decodeJoint(const uint8_t rx_data[8])
{
  if (Crc8(rx_data, 7) != rx_data[7])
  {
    ++joint_crc_err_cnt_;
    return false;
  }
  if (code_part_ != CodePart::Chassis)
  {
    return false;
  }
  const float yaw_rel = static_cast<float>(GetI16(rx_data + 0)) * 1e-3f;
  joint_data_.yaw_rel = yaw_rel;
  joint_data_.pitch = static_cast<float>(GetI16(rx_data + 2)) * 1e-3f;
  joint_data_.mode = rx_data[4];
  joint_data_.active = (rx_data[5] & 0x01u) != 0;
  joint_data_.aligning = (rx_data[5] & 0x02u) != 0;

  /* 关节角速度：底盘跟随要用它做阻尼，否则单靠比例项会在零点附近来回荡。
   * 0x103 是 100 Hz，这里用相邻两帧的真实时间差做差分。dt 明显异常
   * （首帧、丢帧）时保留上一次的值，避免把抖动当成速度。 */
  const uint32_t now = hello_world::tick::GetTickMs();
  if (joint_prev_tick_ != 0U)
  {
    const uint32_t dt = now - joint_prev_tick_;
    if ((dt >= 5U) && (dt <= 30U))
    {
      float delta = yaw_rel - joint_prev_rel_;
      constexpr float kPi = 3.14159265358979f;
      if (delta > kPi) delta -= 2.0f * kPi;
      else if (delta < -kPi) delta += 2.0f * kPi;
      joint_data_.joint_rate = delta * 1000.0f / static_cast<float>(dt);
    }
  }
  joint_prev_rel_ = yaw_rel;
  joint_prev_tick_ = now;

  ++joint_rx_cnt_;
  return true;
}

bool robot::GimbalChassisComm::isOffline() const
{
  constexpr uint32_t kOfflineTimeoutMs = 100U;
  return (hello_world::tick::GetTickMs() - last_rx_tick_) > kOfflineTimeoutMs;
}

/* Private function definitions ----------------------------------------------*/
namespace
{
  /* 物理量 [-1,1] → int8 [-100,100]。注意：必须 roundf，不能用 static_cast 截断 */
  int8_t ToI8(float x)
  {
    if (x > 1.0f)
      x = 1.0f;
    if (x < -1.0f)
      x = -1.0f;
    return static_cast<int8_t>(roundf(x * 100.0f));
  }

  float FromI8(int8_t v) { return static_cast<float>(v) * 0.01f; }

  /* int16 → 两字节，高字节在前（大端只是约定，两端一致即可） */
  void PutI16(uint8_t *p, int16_t v)
  {
    uint16_t u = static_cast<uint16_t>(v);
    p[0] = static_cast<uint8_t>(u >> 8);
    p[1] = static_cast<uint8_t>(u & 0xFFu);
  }

  int16_t GetI16(const uint8_t *p)
  {
    uint16_t u = static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
    return static_cast<int16_t>(u); /* C++20 起无符号→有符号的补码回绕是良定义的 */
  }

  uint8_t Crc8(const uint8_t *p, uint32_t len)
  {
    hello_world::tools::crc8_check crc; /* 注意：getCrc8CheckNum 不是 const 成员 */
    return crc.getCrc8CheckNum(p, len, 0x00);
  }
} // namespace