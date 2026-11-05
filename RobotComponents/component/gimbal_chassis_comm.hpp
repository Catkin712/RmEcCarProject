/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-19 19:38:57
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-10-28 19:35:57
 * @FilePath: \Project_Steer\RmEcCarProject\RobotComponents\component\gimbal_chassis_comm.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "tick.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot
{
  /* Exported constants --------------------------------------------------------*/
  /* Exported types ------------------------------------------------------------*/

  /* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
  class GimbalChassisComm
  {
  public:
    /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
    enum class CodePart : uint8_t
    {
      Chassis = 0,
      Gimbal = 1
    };

    /* 底盘→云台（C2G）帧数据：字段由新人按考核要求设计。 */
    struct ChassisData
    {
      bool ready = false;     ///< 底盘是否就绪
      uint8_t mode = 0;       ///< 工作模式（任务 6 换成 module_state 的枚举）
      float yaw_ref = 0.0f;   ///< 期望云台 yaw，单位 rad
      float pitch_ref = 0.0f; ///< 期望云台 pitch，单位 rad
    };
    /* 云台→底盘（G2C）帧数据：字段由新人按考核要求设计。 */
    struct GimbalData
    {
      float rc_lv = 0.0f;     ///< 左摇杆竖直 [-1, 1]
      float rc_lh = 0.0f;     ///< 左摇杆水平
      float rc_rv = 0.0f;     ///< 右摇杆竖直
      float rc_rh = 0.0f;     ///< 右摇杆水平
      float rc_wheel = 0.0f;  ///< 拨轮
      uint8_t l_switch = 1;   ///< 左拨杆 1/2/3（不是 DT7 的枚举，协议层不依赖设备头文件）
      uint8_t r_switch = 1;   ///< 右拨杆 1/2/3
      bool ready = false;     ///< 云台是否就绪
      bool rc_online = false; ///< 遥控器是否在线
    };

    /* 云台→底盘 的关节状态帧（0x103）：底盘用它做“底盘跟随云台”。
     * 0x102 的 8 字节已经排满，关节角另开一帧，100 Hz 发送即可。 */
    struct JointData
    {
      float yaw_rel = 0.0f; ///< 云台相对底盘的 yaw 关节角，rad
      float pitch = 0.0f;   ///< 云台 pitch 关节角，rad（水平为 0）
      uint8_t mode = 0;     ///< 云台状态机：0 失能 / 1 随动 / 2 分离
      bool active = false;  ///< 云台是否处于工作状态
      bool aligning = false; ///< 随动模式下是否处于“底盘转到云台方向”阶段
    };

    GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

    /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
    void encode(uint8_t tx_data[8]);
    /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。 */
    void decode(const uint8_t rx_data[8]);

    /* 编码/解码关节状态帧（0x103，只有云台发、底盘收）。 */
    void encodeJoint(uint8_t tx_data[8]);
    bool decodeJoint(const uint8_t rx_data[8]);

    /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
    bool isOffline() const;

    /* 数据访问（供 RobotModules / Task 读写）。 */
    ChassisData &chassis_data() { return chassis_data_; }
    GimbalData &gimbal_data() { return gimbal_data_; }
    JointData &joint_data() { return joint_data_; }
    const ChassisData &chassis_data() const { return chassis_data_; }
    const GimbalData &gimbal_data() const { return gimbal_data_; }

    // 调试|发帧 用
    uint32_t tx_id(void) const { return tx_id_; } ///< 发帧时要用
    uint32_t rx_id(void) const { return rx_id_; }
    uint8_t tx_seq(void) const { return tx_seq_; }
    uint8_t rx_seq(void) const { return rx_seq_; }
    uint32_t rx_cnt(void) const { return rx_cnt_; }           ///< 成功解码帧数
    uint32_t crc_err_cnt(void) const { return crc_err_cnt_; } ///< CRC 错帧数
  private:
    CodePart code_part_;
    uint32_t tx_id_;
    uint32_t rx_id_;
    uint32_t last_rx_tick_; ///< 上次收到对方帧的 GetTickMs() 时间戳
    ChassisData chassis_data_;
    GimbalData gimbal_data_;
    JointData joint_data_;
    uint8_t tx_seq_ = 0; ///< 发送序号
    uint8_t tx_joint_seq_ = 0; ///< 关节帧序号
    uint8_t rx_seq_ = 0; ///< 对方最近一帧的序号
    uint32_t rx_cnt_ = 0;
    uint32_t crc_err_cnt_ = 0;
    uint32_t joint_rx_cnt_ = 0;     ///< 0x103 有效帧数
    uint32_t joint_crc_err_cnt_ = 0; ///< 0x103 CRC 错帧数
  public:
    uint32_t joint_rx_cnt(void) const { return joint_rx_cnt_; }
    uint32_t joint_crc_err_cnt(void) const { return joint_crc_err_cnt_; }
  };
  /* Exported variables --------------------------------------------------------*/
  /* Exported function prototypes ----------------------------------------------*/
} // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
