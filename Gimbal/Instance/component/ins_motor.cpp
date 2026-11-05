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
namespace
{
    constexpr uint8_t kYawId = 1;
    constexpr uint8_t kPitchId = 2;

    /* 关节有没有额外的同步带/齿轮：直连填 1，有传动就填实际传动比 */
    constexpr float kYawExReduRat = 1.0f;
    constexpr float kPitchExReduRat = 1.0f;

    /* 2026-11-05 实测：云台朝正前时 yaw 读数为 -1.6219。
     * decodeFdb 里 angle_ = normAngle(actual - angle_offset)，所以填 -1.6219
     * 能让"正前"变成 0。 */
    constexpr float kYawAngleOffset = -1.6219f;

    /* 2026-11-05 实测：枪管/镜头水平时 pitch 读数为 -2.9122。
     * 同样填 offset，让"水平"变成 0；由此得上限 +0.231（抬 13.2°）、
     * 下限 -0.137（压 7.9°），与实测的 -2.6815 / -3.0494 一致。 */
    constexpr float kPitchAngleOffset = -2.9122f;

    /* yaw 是 360° 连续旋转：角度范围必须无限，否则解码时圈数会被折进减速比
     * pitch 有机械限位：用 [-π, π)，再在关节层做软件限位。*/
    const hello_world::motor::OptionalParams kYawParams = {
        hello_world::motor::InputType::kTorq, /* MIT 模式用不到，占位 */
        hello_world::motor::AngleRange::kNegInfToPosInf,
        hello_world::motor::kDirFwd, /* 实测：顺时针转读数减小 → 逆时针为正，不用改 */
        false,                       /* remove_build_in_reducer */
        kYawAngleOffset,             /* angle_offset：正前 = 0 rad */
        kYawExReduRat,               /* ex_redu_rat */
    };

    const hello_world::motor::OptionalParams kPitchParams = {
        hello_world::motor::InputType::kTorq,
        hello_world::motor::AngleRange::kNegPiToPosPi,
        hello_world::motor::kDirFwd,
        false,
        kPitchAngleOffset, /* angle_offset：水平 = 0 rad */
        kPitchExReduRat,
    };

    hello_world::motor::DM_J4310 g_yaw(kYawId, kYawParams, true);
    hello_world::motor::DM_J4310 g_pitch(kPitchId, kPitchParams, true);

} // namespace

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
