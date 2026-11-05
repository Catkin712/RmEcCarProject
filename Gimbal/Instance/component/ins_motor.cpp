/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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
#include "ins_motor.hpp"

#include "motor_DaMiao.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {

/* 达妙电机的 ID（1~15）：出厂值或上位机设定的，第一次上机必须实测确认。
 * 判据：上电后 g_yaw_angle_dbg / g_pitch_angle_dbg 一直是 0，就是这里猜错了。
 * 去 Ozone 看这两组计数器：
 *   yaw   → HW_can 的 can1_unknown_cnt / can1_last_unknown_id
 *   pitch → can2_unknown_cnt / can2_last_unknown_id
 * 收到的 ID 减去 0x10 就是实际电机 ID（反馈 ID 恒为 id + 0x10）。*/
constexpr uint8_t kYawId = 1;
constexpr uint8_t kPitchId = 2;

/* 关节有没有额外的同步带/齿轮：直连填 1，有传动就填实际传动比 */
constexpr float kYawExReduRat = 1.0f;
constexpr float kPitchExReduRat = 1.0f;

/* yaw 是 360° 连续旋转：角度范围必须无限，否则解码时圈数会被折进减速比
 * （见 motor_DaMiao.cpp:201-203）。
 * pitch 有机械限位：用 [-π, π)，再在关节层做软件限位。*/
const hello_world::motor::OptionalParams kYawParams = {
    hello_world::motor::InputType::kTorq, /* MIT 模式用不到，占位 */
    hello_world::motor::AngleRange::kNegInfToPosInf,
    hello_world::motor::kDirFwd, /* 实测后可能要改 kDirRev */
    false,                       /* remove_build_in_reducer */
    0.0f,                        /* angle_offset：零位偏差，实测后填 */
    kYawExReduRat,               /* ex_redu_rat */
};

const hello_world::motor::OptionalParams kPitchParams = {
    hello_world::motor::InputType::kTorq,
    hello_world::motor::AngleRange::kNegPiToPosPi,
    hello_world::motor::kDirFwd,
    false,
    0.0f,
    kPitchExReduRat,
};

/* 第三个参数 auto_enable=true：第一次 encode 时自动发使能指令。
 * 达妙电机失能状态下不反馈数据，所以想读角度就必须使能；
 * 配合 GimbalTask 里的零力矩 MIT 指令，使能后也不会发力。*/
hello_world::motor::DM_J4310 g_yaw(kYawId, kYawParams, true);
hello_world::motor::DM_J4310 g_pitch(kPitchId, kPitchParams, true);

}  // namespace

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
hello_world::motor::Motor *GetMotorYaw() { return &g_yaw; }

hello_world::motor::Motor *GetMotorPitch() { return &g_pitch; }

uint8_t GetYawMotorId(void) { return kYawId; }

uint8_t GetPitchMotorId(void) { return kPitchId; }

/* Private function definitions ----------------------------------------------*/
