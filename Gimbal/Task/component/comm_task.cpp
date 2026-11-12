/**
 *******************************************************************************
 * @file      :comm_task.cpp
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
#include "comm_task.hpp"

#include <cmath> /* fabsf */

#include "DT7.hpp"          /* DT7::rc_xxx()，协议层不直接依赖它 */
#include "HW_can.hpp"       /* CanFilter_Init / CAN_Send_Msg */
#include "can.h"            /* hcan1 / hcan2 */
#include "ins_comm.hpp"     /* GetRemoteControl */
#include "ins_motor.hpp"    /* GetMotorYaw / GetMotorPitch */
#include "imu.hpp"          /* 完整类型：yaw() / is_ok() */
#include "ins_imu.hpp"      /* GetImu */
#include "motor_DaMiao.hpp" /* 完整类型：setMitInput / encode / tx_id */
#include "usart.h"          /* huart3 */
#include "iwdg.h"
#include "tick.hpp" /* hello_world::tick::GetTickMs() */

#include "gimbal_chassis_comm.hpp"     /* 完整类型：encode()/tx_id() */
#include "ins_chassis_gimbal_comm.hpp" /* GetGimbalChassisComm */

namespace
{
  /* 云台控制参数（2026-11-05 实测标定） */
  /* pitch 机械行程（水平为 0 的关节角，见 ins_motor.cpp 的 kPitchAngleOffset）：
   * 实测上限 -2.6815 → +0.231（抬 13.2°），下限 -3.0494 → -0.137（压 7.9°）。
   * 软件限位两端各留约 1.7° 余量，防止顶死结构。 */
  constexpr float kPitchLimitMax = 0.20f;   /* 抬头极限 */
  constexpr float kPitchLimitMin = -0.10f;  /* 低头极限 */
  constexpr float kPitchRate = 1.5f;        /* 摇杆满程时俯仰角速度，rad/s */
  constexpr float kYawRate = 6.0f;          /* 摇杆满程时 yaw 角速度，rad/s */
  constexpr float kRcDeadBand = 0.05f;      /* 摇杆死区 */
  constexpr float kPitchKp = 40.0f;         /* MIT 内置 PD：kp ∈ [0, 500] */
  constexpr float kPitchKd = 1.0f;          /* kd ∈ [0, 5] */
  constexpr float kYawFollowKp = 6.0f;      /* 随动：误差(rad) → 角速度(rad/s) */
  constexpr float kYawFollowVelMax = 8.0f;  /* 随动最大跟踪角速度，rad/s */
  constexpr float kYawErrDeadBand = 0.005f; /* 误差死区，防止到位后嗡嗡抖 */
  constexpr float kYawVelKd = 2.0f;         /* 速度环阻尼（随动/小陀螺共用） */
  /* 小陀螺模式：锁 IMU 绝对朝向的比例环 */
  constexpr float kYawFreeKp = 6.0f;           /* 误差(rad) → 角速度(rad/s) */
  constexpr float kYawFreeVelMax = 12.0f;      /* 最大补偿角速度，rad/s */
  constexpr float kYawFreeErrDeadBand = 0.01f; /* 约 0.6°，防止到位后嗡嗡抖 */
  constexpr float kPi = 3.14159265358979f;
  constexpr uint32_t kGimbalJointTxId = 0x103U; /* 云台→底盘：关节状态帧 */
  constexpr float kDt = 0.001f;                 /* TIM6 = 1 kHz，固定步长 */

  /* 摇杆死区 + 重新线性映射：死区外从 0 平滑到 1，避免"一过死区就跳" */
  inline float ApplyDeadBand(float v)
  {
    const float a = (v < 0.0f) ? -v : v;
    if (a < kRcDeadBand)
    {
      return 0.0f;
    }
    return (v > 0.0f) ? (v - kRcDeadBand) / (1.0f - kRcDeadBand)
                      : (v + kRcDeadBand) / (1.0f - kRcDeadBand);
  }

  inline float Bound(float v, float lo, float hi)
  {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
  }

  /* 把角度折回 (-π, π]，与电机层的 normAngle 语义一致 */
  inline float WrapPi(float a)
  {
    while (a > kPi)
    {
      a -= 2.0f * kPi;
    }
    while (a < -kPi)
    {
      a += 2.0f * kPi;
    }
    return a;
  }
} // namespace

/* Private variables ---------------------------------------------------------*/
static uint8_t dbus_rx_buf[hello_world::remote_control::kRcRxDataLen];
/* 三个状态量：放在文件作用域、用 static（不用匿名 namespace），
 * 这样 Ozone 的表达式里直接写 g_comm_armed 就能找到，不会显示成
 * "(anonymous namespace)::g_comm_armed"。 */
static volatile bool g_comm_armed = false;         /* 是否已收到过有效帧 */
static uint32_t g_boot_tick = 0;                   /* CommTaskInit 里记录的开机时刻 */
static constexpr uint32_t kStartupGraceMs = 2000U; /* 开机宽限期 */
/* 看门狗判据：最近一次"有效通讯"的时刻。喂狗只看它够不够新：
 * 通讯新鲜 = 解包中断活着；喂狗在主循环 = 主循环也活着。缺一个就复位。 */
static volatile uint32_t g_last_comm_tick = 0;
static constexpr uint32_t kCommTimeoutMs = 50U; /* 两帧之间最长允许间隔（DBUS 100 Hz） */
/* 复位原因快照：RCC_CSR 的 bit29 = IWDGRSTF。复位后变量会清，所以开机立刻存下来 */
static volatile uint32_t g_reset_cause = 0;
/* 故障注入开关：Ozone 里置 1 就模拟"主循环卡死"，用来验证看门狗真的会复位 */
static volatile uint32_t g_dbg_hang = 0;
/* 复位后安全闩：必须先把左拨杆离开最上、再推到最上，才允许使能。
 * 否则看门狗复位/重新上电时，若拨杆还停在上方，云台会立刻重新抱死。 */
static bool g_seen_safe = false;                /* 见过"不在最上"的合法帧 */
static bool g_arm_latch = false;                /* 已完成一次 上→下→上 */
static volatile uint8_t g_arm_unlocked_dbg = 0; /* 1 = 允许使能 */
/* Ozone 观察用 */
static volatile uint32_t rc_rx_cnt = 0;  /* 成功解包的帧数 */
static volatile uint32_t rc_err_cnt = 0; /* 长度不对、被丢弃的次数 */
static volatile uint32_t tick_now = 0;
static volatile uint32_t link_tx_cnt = 0; /* 已发出的 0x102 帧数 */

/* 云台关节自检用的观察量（Ozone 输入名字即可） */
static volatile uint32_t g_yaw_tx_cnt = 0; /* yaw MIT 帧成功入邮箱次数 */
/* 0x103 关节状态帧的发送计数与镜像 */
static uint32_t g_joint_div = 0;
static volatile uint32_t g_joint_tx_cnt = 0;
static volatile float g_joint_yaw_dbg = 0.0f;
static volatile uint32_t g_pitch_tx_cnt = 0;    /* pitch MIT 帧成功入邮箱次数 */
static volatile float g_yaw_angle_dbg = 0.0f;   /* yaw 反馈角（rad） */
static volatile float g_pitch_angle_dbg = 0.0f; /* pitch 反馈角（rad） */
static volatile float g_yaw_vel_dbg = 0.0f;
static volatile float g_pitch_vel_dbg = 0.0f;
static volatile uint8_t g_yaw_enabled_dbg = 0; /* 1=已使能 */
static volatile uint8_t g_pitch_enabled_dbg = 0;
static volatile uint32_t g_yaw_tx_id_dbg = 0; /* 本帧用的发送 ID */
static volatile uint32_t g_pitch_tx_id_dbg = 0;
/* 控制环的中间量（也是 Ozone 观察点） */
static volatile float g_pitch_des_dbg = 0.0f;   /* pitch 目标角（rad） */
static volatile float g_yaw_vel_des_dbg = 0.0f; /* yaw 目标角速度（rad/s） */
static volatile uint8_t g_gimbal_active_dbg = 0;

/* 云台状态机：0=失能/急停，1=随动，2=小陀螺 */
enum class GimbalMode : uint8_t
{
  kDisabled = 0U,
  kFollow = 1U,
  kSpin = 2U,
};
static GimbalMode g_mode = GimbalMode::kDisabled;
static volatile uint8_t g_mode_dbg = 0U;    /* Ozone 观察：当前模式 */
static volatile float g_yaw_des_dbg = 0.0f; /* 随动模式的相对角目标（rad） */
static volatile float g_yaw_err_dbg = 0.0f; /* 随动的角度误差（rad） */
/* 小陀螺模式：锁定的绝对朝向（IMU yaw，rad）与观察量 */
static float g_yaw_free_lock = 0.0f; /* 进入小陀螺时锁定的绝对朝向 */
/* 随动（底盘跟随）的两段式状态 */
static float g_yaw_follow_lock = 0.0f;             /* 对齐阶段锁定的绝对朝向 */
static bool g_follow_aligning = false;             /* true = 正在等底盘转过来 */
static volatile uint8_t g_follow_aligning_dbg = 0; /* Ozone：1 = 对齐阶段 */
static constexpr float kAlignEnterRad = 0.12f;     /* 超过这个误差 → 进入对齐阶段 */
static constexpr float kAlignExitRad = 0.03f;      /* 低于这个误差 → 认为已对齐 */
static constexpr float kAlignExitRate = 0.10f;     /* 关节角速度低于此值才算"底盘真的停了"，rad/s */
static volatile float g_align_rate_dbg = 0.0f;     /* 关节相对角速度，不能直接当作底盘转速 */
/* 运动补偿试验开关：默认关闭。仅在静止时开启，先核对估计转速的方向/量程。
 * 此开关仅管随动第一阶段；小陀螺始终使用运动补偿。 */
static volatile uint8_t g_yaw_motion_comp_enable = 1U;
static volatile uint8_t g_yaw_rate_valid_dbg = 0U;
static volatile float g_yaw_abs_rate_dbg = 0.0f;         /* IMU 欧拉 yaw 变化率，rad/s */
static volatile float g_chassis_yaw_rate_est_dbg = 0.0f; /* 底盘转速估计，rad/s */
static volatile float g_yaw_comp_rate_dbg = 0.0f;        /* 本拍叠加的关节速度补偿 */
static float g_chassis_yaw_rate_filtered = 0.0f;

/* TIM6 每 1 ms 调用：先由已校正安装方向的陀螺仪求欧拉 yaw 变化率，
 * 再用 omega_chassis = omega_gimbal - joint_rate 估计底盘运动。
 * 适用于车体近水平、yaw 关节轴近似竖直的测试条件；坡面需另做坐标变换。 */
static void UpdateYawMotionEstimate(float joint_rate)
{
  const auto *imu = GetImu();
  g_yaw_rate_valid_dbg = 0U;
  g_yaw_abs_rate_dbg = 0.0f;
  g_chassis_yaw_rate_est_dbg = 0.0f;

  if (imu->is_ok() && imu->is_calibrated())
  {
    const float *gyro = imu->gyro();
    const float *bias = imu->gyro_bias();
    const float roll = imu->roll();
    const float cos_pitch = cosf(imu->pitch());
    /* 避免欧拉角接近竖直时除以接近零的数。 */
    if (std::isfinite(roll) && std::isfinite(cos_pitch) &&
        fabsf(cos_pitch) > 0.5f)
    {
      const float abs_rate = ((gyro[1] - bias[1]) * sinf(roll) +
                              (gyro[2] - bias[2]) * cosf(roll)) /
                             cos_pitch;
      if (std::isfinite(abs_rate) && std::isfinite(joint_rate))
      {
        constexpr float kMotionFilterTau = 0.020f; /* 20 ms 初始试验值，待实测 */
        constexpr float kMotionFilterAlpha = kDt / (kMotionFilterTau + kDt);
        const float chassis_rate = abs_rate - joint_rate;
        g_chassis_yaw_rate_filtered +=
            kMotionFilterAlpha * (chassis_rate - g_chassis_yaw_rate_filtered);
        g_yaw_abs_rate_dbg = abs_rate;
        g_chassis_yaw_rate_est_dbg = g_chassis_yaw_rate_filtered;
        g_yaw_rate_valid_dbg = 1U;
        return;
      }
    }
  }
  g_chassis_yaw_rate_filtered = 0.0f;
}
static volatile float g_yaw_free_yaw_dbg = 0.0f; /* IMU 当前绝对朝向 */
static volatile float g_yaw_free_err_dbg = 0.0f; /* 锁定值 − 当前值（rad） */
/* 排查用：Ozone 里把 g_dbg_force_follow 改成 1，可无视拨杆强制进随动 */
static volatile uint8_t g_dbg_force_follow = 0U;
static volatile uint8_t g_rc_l_switch_dbg = 0U; /* 左拨杆原始值 1=上 3=中 2=下 */
static volatile uint8_t g_rc_r_switch_dbg = 0U; /* 右拨杆原始值 */

/* Private function prototypes -----------------------------------------------*/
bool RcIsOnline(void); /* 定义在本文件末尾，CommTask 要先用 */

/* Exported function definitions ---------------------------------------------*/
void CommTaskInit(void)
{
  g_boot_tick = hello_world::tick::GetTickMs();
  /* 读复位原因再清零：Ozone 里 g_reset_cause 的 bit29(IWDGRSTF)=1 就是看门狗复位 */
  g_reset_cause = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* 云台 CAN1：底盘 0x101 + yaw 达妙反馈，FIFO0/bank0 全收；
   * 云台 CAN2：pitch 达妙反馈，FIFO1/bank14 全收。 */
  CanFilter_Init(&hcan1);
  (void)HAL_CAN_Start(&hcan1);
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  (void)HAL_CAN_Start(&hcan2);
  (void)HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 启动 DBUS 接收（空闲中断 + DMA）
  (void)HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3, dbus_rx_buf, hello_world::remote_control::kRcRxDataLen);
}

void CommTask(void)
{
  robot::GimbalChassisComm *comm = GetGimbalChassisComm();
  hello_world::remote_control::DT7 *rc = GetRemoteControl();

  /* ① 把遥控器语义搬进协议结构：
   *    协议层（RobotComponents）不 include DT7，转换只在 Task 层做。 */
  comm->gimbal_data().rc_lv = rc->rc_lv();
  comm->gimbal_data().rc_lh = rc->rc_lh();
  comm->gimbal_data().rc_rv = rc->rc_rv();
  comm->gimbal_data().rc_rh = rc->rc_rh();
  comm->gimbal_data().rc_wheel = rc->rc_wheel();
  comm->gimbal_data().l_switch = static_cast<uint8_t>(rc->rc_l_switch());
  comm->gimbal_data().r_switch = static_cast<uint8_t>(rc->rc_r_switch());
  comm->gimbal_data().ready = true;
  comm->gimbal_data().rc_online = RcIsOnline();

  /* ② 打包并发出去（1 kHz，8 字节） */
  uint8_t frame[8] = {0};
  comm->encode(frame);
  (void)CAN_Send_Msg(&hcan1, frame, comm->tx_id(), 8);
  link_tx_cnt = link_tx_cnt + 1U;
}

/**
 * @brief   云台关节环 + 模式状态机（1 kHz）
 * @note    模式由左/右拨杆决定：
 *            左拨杆最上            → 允许使能，否则失能（零力矩可手转）
 *            右拨杆 上 / 中 / 下   → 随动 / 小陀螺 / 急停
 *          yaw：随动分两阶段，小陀螺锁绝对朝向并补偿底盘运动；
 *          pitch：与 yaw 模式无关，使能时是带软件限位的位置环。
 */
void GimbalTask(void)
{
  hello_world::motor::DM_J4310 *yaw =
      static_cast<hello_world::motor::DM_J4310 *>(GetMotorYaw());
  hello_world::motor::DM_J4310 *pitch =
      static_cast<hello_world::motor::DM_J4310 *>(GetMotorPitch());
  const hello_world::remote_control::DT7 *rc = GetRemoteControl();

  /* ① 先把反馈搬进 Ozone 能看的变量 */
  g_yaw_angle_dbg = yaw->angle();
  g_pitch_angle_dbg = pitch->angle();
  g_yaw_vel_dbg = yaw->vel();
  g_pitch_vel_dbg = pitch->vel();
  UpdateYawMotionEstimate(g_yaw_vel_dbg);
  g_yaw_comp_rate_dbg = 0.0f;
  g_yaw_enabled_dbg = yaw->is_enabled() ? 1U : 0U;
  g_pitch_enabled_dbg = pitch->is_enabled() ? 1U : 0U;

  /* ② 定模式：遥控在线 + 左拨杆最上 + 通过安全闩 才允许工作 */
  const bool rc_link = RcIsOnline();
  const bool l_up = (rc->rc_l_switch() == hello_world::remote_control::kSwitchStateUp);

  /* 安全闩只认"遥控在线时看到的拨杆状态"：上电瞬间 DBUS 还没数据，
   * 此时读到的是默认 0，不能算数。 */
  if (rc_link)
  {
    if (!l_up)
    {
      g_seen_safe = true; /* 看到过非"最上"的状态，说明操作者手在杆上 */
      g_arm_latch = false;
    }
    else if (g_seen_safe)
    {
      g_arm_latch = true; /* 先离杆再推到最上 = 一次明确的使能动作 */
    }
  }
  g_arm_unlocked_dbg = (g_arm_latch && l_up) ? 1U : 0U;

  const bool rc_ok = rc_link && g_arm_latch && l_up;

  GimbalMode mode = GimbalMode::kDisabled;
  if (rc_ok)
  {
    switch (rc->rc_r_switch())
    {
    case hello_world::remote_control::kSwitchStateUp:
      mode = GimbalMode::kFollow; /* 随动：云台相对底盘锁角 */
      break;
    case hello_world::remote_control::kSwitchStateMid:
      mode = GimbalMode::kSpin; /* 小陀螺：底盘自旋，云台锁绝对朝向 */
      break;
    default: /* 右拨杆最下 = 急停 */
      mode = GimbalMode::kDisabled;
      break;
    }
  }

  /* 排查开关：Ozone 里把 g_dbg_force_follow 置 1，可强制进随动 */
  if (g_dbg_force_follow != 0U)
  {
    mode = GimbalMode::kFollow;
  }
  /* 绝对朝向保持不可用时禁止小陀螺，0x103 会通知底盘撤销自旋。 */
  if (mode == GimbalMode::kSpin && g_yaw_rate_valid_dbg == 0U)
  {
    mode = GimbalMode::kDisabled;
  }
  g_rc_l_switch_dbg = static_cast<uint8_t>(rc->rc_l_switch());
  g_rc_r_switch_dbg = static_cast<uint8_t>(rc->rc_r_switch());

  /* ③ 切换瞬态：进随动/失能时把目标对齐当前角，避免"一使能就甩头" */
  if (mode != g_mode)
  {
    g_yaw_des_dbg = g_yaw_angle_dbg;
    g_pitch_des_dbg = Bound(g_pitch_angle_dbg, kPitchLimitMin, kPitchLimitMax);
    /* 清掉上一次随动的"对齐中"状态。
     * 不清的话：在随动里对齐到一半切去小陀螺，g_follow_aligning 还留着 true，
     * 再切回随动时不会重新锁存朝向，而是拿旧的锁去算误差——云台会一路
     * 拽回"小陀螺前的角度"。清掉后由 kFollow 分支按当前 rel 重新判定/锁存。 */
    g_follow_aligning = false;
    g_follow_aligning_dbg = 0U;
    if (mode == GimbalMode::kFollow)
    {
      /* 切进随动的第一拍：只要云台相对底盘有可见夹角，就先进"对齐段"。
       * 此时云台锁住**当前绝对朝向**（原地不动），底盘负责转过来；
       * 而不是让云台自己转过去对齐底盘（那样云台会"甩"一下，像随机动作）。 */
      const float rel_now = WrapPi(g_yaw_angle_dbg);
      if (fabsf(rel_now) > kAlignExitRad)
      {
        g_follow_aligning = true;
        g_yaw_follow_lock = GetImu()->yaw();
      }
    }
    if (mode == GimbalMode::kSpin)
    {
      /* 进入小陀螺的瞬间：把"当前绝对朝向"记为锁定值。
       * 不这么做的话，第一次进小陀螺就会朝 IMU 的零位（开机时的朝向）拽。 */
      g_yaw_free_lock = GetImu()->yaw();
    }
    g_mode = mode;
  }
  g_mode_dbg = static_cast<uint8_t>(mode);
  g_gimbal_active_dbg = (mode != GimbalMode::kDisabled) ? 1U : 0U;

  float yaw_vel_des = 0.0f;

  switch (mode)
  {
  case GimbalMode::kFollow:
  {
    /* 随动 = 底盘跟随云台，分两段：
     *   段 1（对齐）：|θ_rel| 大 → 云台锁 **IMU 绝对朝向**，转子先不动，
     *        让底盘自己去转，把 θ_rel 拉回 0；
     *   段 2（锁定）：|θ_rel| 小 → 云台改锁 **关节角 0**，与底盘刚性相连，
     *        之后底盘一转，云台被带着一起转。
     * 摇杆在随动模式下不再驱动云台：右摇杆交给底盘自旋。 */
    const float rel = WrapPi(g_yaw_angle_dbg);

    /* 关节角速度。对齐阶段云台锁着绝对朝向，底盘还在转的话，
     * 云台确实静止时，相对角速度约等于底盘转速的负值。
     * 云台也在运动时，单看相对速度不能证明底盘停稳。 */
    const float joint_rate = yaw->vel();
    g_align_rate_dbg = joint_rate;

    if (!g_follow_aligning && (fabsf(rel) > kAlignEnterRad))
    {
      /* 进入对齐段：以"此刻的绝对朝向"为锁，避免云台回弹到更早的朝向 */
      g_follow_aligning = true;
      g_yaw_follow_lock = GetImu()->yaw();
    }
    else if (g_follow_aligning && (fabsf(rel) < kAlignExitRad) &&
             (fabsf(joint_rate) < kAlignExitRate))
    {
      /* 角度对齐 + 底盘基本停住，才把云台从"锁绝对"释放成"锁关节"。
       * 只看角度就释放的话，底盘还带着惯性转，云台会被拖着走一段角度。 */
      g_follow_aligning = false;
    }
    g_follow_aligning_dbg = g_follow_aligning ? 1U : 0U;

    float err = 0.0f;
    if (g_follow_aligning && GetImu()->is_ok())
    {
      /* 锁绝对朝向：底盘转过来，θ_rel 自然减小 */
      err = WrapPi(g_yaw_follow_lock - GetImu()->yaw());
    }
    else
    {
      /* 已对齐（或 IMU 不可用）：锁关节角 0，跟底盘刚性相连 */
      err = -rel;
    }
    g_yaw_err_dbg = err;
    g_yaw_des_dbg = 0.0f;

    if (fabsf(err) > kYawErrDeadBand)
    {
      yaw_vel_des = Bound(kYawFollowKp * err, -kYawFollowVelMax, kYawFollowVelMax);
    }
    /* 第一阶段：把绝对角环要求的速度转换成关节相对速度。
     * 即使角度误差进入死区，底盘还在转也需要反向补偿。
     * 默认关闭；开启后仍使用原有最终速度限幅。 */
    if (g_follow_aligning && g_yaw_motion_comp_enable != 0U &&
        g_yaw_rate_valid_dbg != 0U)
    {
      g_yaw_comp_rate_dbg = -g_chassis_yaw_rate_est_dbg;
      yaw_vel_des = Bound(yaw_vel_des + g_yaw_comp_rate_dbg,
                          -kYawFollowVelMax, kYawFollowVelMax);
    }
    (void)yaw->setMitInput(0.0f, yaw_vel_des, 0.0f, kYawVelKd, 0.0f);
    break;
  }
  case GimbalMode::kSpin:
  {
    /* 小陀螺：锁 **IMU 绝对朝向**（不是相对底盘的角度）。
     * yaw 关节编码器只有相对角，要锁绝对朝向只能靠板载 IMU。
     * 右摇杆水平改变绝对朝向目标，松手后保持新的目标。
     * 与随动同样不用 MIT 位置模式，只把电机当速度执行器。 */
    const float yaw_abs = GetImu()->yaw();
    /* 遥控角速度积分成世界系朝向目标；回中后目标不再变化。
     * 沿用原摇杆方向，折回 ±pi 避免长时间累加损失浮点精度。 */
    g_yaw_free_lock = WrapPi(g_yaw_free_lock -
                             ApplyDeadBand(rc->rc_rh()) * kYawRate * kDt);
    g_yaw_free_yaw_dbg = yaw_abs;
    const float err = WrapPi(g_yaw_free_lock - yaw_abs);
    g_yaw_free_err_dbg = err;
    if (fabsf(err) > kYawFreeErrDeadBand)
    {
      yaw_vel_des = Bound(kYawFreeKp * err, -kYawFreeVelMax, kYawFreeVelMax);
    }
    else
    {
      yaw_vel_des = 0.0f;
    }
    /* 绝对速度目标转为关节相对速度；死区内也需要抵消底盘运动。 */
    g_yaw_comp_rate_dbg = -g_chassis_yaw_rate_est_dbg;
    yaw_vel_des = Bound(yaw_vel_des + g_yaw_comp_rate_dbg,
                        -kYawFreeVelMax, kYawFreeVelMax);
    (void)yaw->setMitInput(0.0f, yaw_vel_des, 0.0f, kYawVelKd, 0.0f);
    break;
  }
  default:
  {
    /* 失能/急停：零力矩，可手转 */
    (void)yaw->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    break;
  }
  }

  /* ④ pitch：与 yaw 模式解耦；只要不是急停就按摇杆积分 + 限位
   *    摇杆用右摇杆竖直轴 rc_rv：左摇杆竖直轴归底盘前后，别再抢；
   *    上推 = 抬头 = 关节角往 +kPitchLimitMax 走。 */
  if (mode != GimbalMode::kDisabled)
  {
    g_pitch_des_dbg = Bound(
        g_pitch_des_dbg + ApplyDeadBand(rc->rc_rv()) * kPitchRate * kDt,
        kPitchLimitMin, kPitchLimitMax);
    (void)pitch->setMitInput(g_pitch_des_dbg, 0.0f, kPitchKp, kPitchKd, 0.0f);
  }
  else
  {
    g_pitch_des_dbg = Bound(g_pitch_angle_dbg, kPitchLimitMin, kPitchLimitMax);
    (void)pitch->setMitInput(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  g_yaw_vel_des_dbg = yaw_vel_des;

  /* ⑤ 组帧下发 */
  uint8_t frame[8] = {0};
  if (yaw->encode(frame))
  {
    (void)CAN_Send_Msg(&hcan1, frame, yaw->tx_id(), 8);
    g_yaw_tx_cnt = g_yaw_tx_cnt + 1U;
  }
  g_yaw_tx_id_dbg = yaw->tx_id();

  if (pitch->encode(frame))
  {
    (void)CAN_Send_Msg(&hcan2, frame, pitch->tx_id(), 8);
    g_pitch_tx_cnt = g_pitch_tx_cnt + 1U;
  }
  g_pitch_tx_id_dbg = pitch->tx_id();

  /* ⑥ 云台→底盘关节状态帧（0x103，100 Hz）
   * 0x102 的 8 字节已排满，关节角另开一帧；底盘拿它做“跟随云台”。
   * 100 Hz 够用：底盘跟随是慢环，没必要 1 kHz 占总线。 */
  ++g_joint_div;
  if (g_joint_div >= 10U)
  {
    g_joint_div = 0U;
    robot::GimbalChassisComm *comm = GetGimbalChassisComm();
    /* 关节角是多圈的，协议里统一放 ±π 内的值，别把多圈角传给底盘 */
    comm->joint_data().yaw_rel = WrapPi(g_yaw_angle_dbg);
    comm->joint_data().pitch = g_pitch_angle_dbg;
    comm->joint_data().mode = static_cast<uint8_t>(g_mode);
    comm->joint_data().active = (g_mode != GimbalMode::kDisabled);
    comm->joint_data().aligning = g_follow_aligning;

    uint8_t joint_frame[8] = {0};
    comm->encodeJoint(joint_frame);
    if (CAN_Send_Msg(&hcan1, joint_frame, kGimbalJointTxId, 8) == HAL_OK)
    {
      g_joint_tx_cnt = g_joint_tx_cnt + 1U;
    }
    g_joint_yaw_dbg = g_yaw_angle_dbg;
  }
}

/* Private function definitions ----------------------------------------------*/
/* 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收； */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                           uint16_t Size)
{
  if (huart != &huart3)
  {
    return;
  }
  if (huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }
  /* ②  只处理完整的一帧：长度必须等于 18。
   *    线路中途空闲 Size<18 丢弃并计数。 */
  if (Size == hello_world::remote_control::kRcRxDataLen)
  {
    GetRemoteControl()->decode(dbus_rx_buf);
    rc_rx_cnt = rc_rx_cnt + 1U;

    CommTaskNotifyCommOk(); // 有效帧
  }
  else
  {
    rc_err_cnt = rc_err_cnt + 1U;
  }

  /*    重启接收：DMA 为 NORMAL 时 HAL 已在 IDLE 里把状态复位成 READY，
   *    所以这里可以安全重启。每次重启都对齐到"下一帧的起点"。 */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(
      &huart3, dbus_rx_buf, hello_world::remote_control::kRcRxDataLen);
}

/**
 * @brief  遥控器是否在线
 * @retval true = 最近 100 ms 内收到过新帧
 * @note   rc_rx_cnt 是 static（本文件私有），所以这个判断只能放在本文件。
 *         不能只看"计数为 0"：上电初期本来就是 0，判据是"一段时间内有没有变化"。
 */
bool RcIsOnline(void)
{
  constexpr uint32_t kRcTimeoutMs = 100U;

  static uint32_t last_cnt = 0;
  static uint32_t last_tick = 0;
  static bool seen = false;

  uint32_t cnt = rc_rx_cnt; /* volatile 读一次，后面都用这个快照 */

  if (cnt != last_cnt)
  {
    last_cnt = cnt;
    last_tick = hello_world::tick::GetTickMs();
    seen = true; /* 收到过帧之后，时间判据才有意义 */
    return true;
  }
  if (!seen)
  {
    return false;
  }
  /* 无符号减法：计到 2^32 回绕依然给出正确的时间差 */
  return (hello_world::tick::GetTickMs() - last_tick) < kRcTimeoutMs;
}

extern "C" void CommTaskNotifyCommOk(void)
{
  /* 只记"有效通讯发生的时刻"，绝不在这里喂狗：
   * 本函数跑在 USART 中断里，中断活着不代表主循环活着。 */
  g_comm_armed = true;
  g_last_comm_tick = hello_world::tick::GetTickMs();
}

extern "C" void CommTaskWatchdog(void)
{
  /* 故障注入：Ozone 把 g_dbg_hang 写 1，本函数就卡在这里不再喂狗。
   * 主循环死了、中断还活着的情况下，看门狗必须能把板子拉回来。 */
  if (g_dbg_hang != 0U)
  {
    for (;;)
    {
    }
  }

  /* Ozone 只能 watch 变量、不能 watch 函数返回值：
   * 每圈把当前时间写进 tick_now，调试时看它涨不涨即可 */
  tick_now = hello_world::tick::GetTickMs();

  /* 两条判据同时成立才喂狗：
   *   1) 主循环还在跑 —— 本函数就是主循环调的；
   *   2) 通讯还新鲜   —— 最近 50 ms 内收到过有效帧。
   * 开机 2 s 内放宽，给遥控器/底盘留出上电与首帧的时间。 */
  const bool comm_fresh =
      g_comm_armed && ((tick_now - g_last_comm_tick) < kCommTimeoutMs);
  const bool in_grace = (tick_now - g_boot_tick) < kStartupGraceMs;

  if (comm_fresh || in_grace)
  {
    (void)HAL_IWDG_Refresh(&hiwdg);
  }
}
