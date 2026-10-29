/**
 *******************************************************************************
 * @file      : robot.hpp
 * @brief     : 底盘板顶层状态机：Dead / Resurrection / Working
 *******************************************************************************
 */
#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

#include <cstdint>

#include "chassis.hpp"

/* ============================================================================
声明底盘板顶层状态机类 Robot（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Robot 类，作为本板每个控制周期（1ms）的入口：汇总各模块数据，
 *      生成控制指令并下发给执行部件。
 *   2. 实现「生命状态」三态切换：Dead（上电未就绪 / 离线）→ Resurrection（恢复
 *      自检）→ Working（正常控制），并定义各状态进入 / 退出时的复位动作。
 *   3. 在 Working 态下按手动控制源与工作模式（分离 / 随动 / 小陀螺）生成底盘运动指令。
 *   4. 提供注册各模块指针的接口（Chassis、Imu、4 轮电机、4 舵电机、GimbalChassisComm），并在每个周期按正确顺序更新数据。
 *
 * 约束 / 提示：
 *   - 底盘板与云台板之间只能通过 GimbalChassisComm 的
 *     CAN 帧交换信息，不能直接互相调用函数或共享变量。
 *   - 控制周期入口由 Task/component/main_task.cpp 的定时中断调用，本类自己不要开中断、
 *     不要死等延时。
 *   - 急停：遥控器拨到急停位时，必须保证所有电机指令归零（硬性验收项）。
 *   - 状态机的基类 / 迁移机制要自己设计，本骨架不提供（裁判系统 / 超电 / 蜂鸣器等组件本骨架也没有，可忽略）。
 * ==========================================================================*/

namespace robot
{

class Robot
{
 public:
  enum class State : uint8_t
  {
    kDead = 0,          ///< 未使能 / 离线 / 急停
    kResurrection = 1,  ///< 自检中（输出 0）
    kWorking = 2,       ///< 正常控制
  };

  explicit Robot(Chassis* chassis) : chassis_(chassis) {}

  void init(void);
  void update(void);  ///< 1 ms，由 MainTask 调用

  State state(void) const { return state_; }
  const char* state_name(void) const;

 private:
  void enter_state(State next);
  void read_rc(void);
  Chassis::Velocity stick_to_velocity(void) const;

  Chassis* chassis_ = nullptr;
  State state_ = State::kDead;
  bool kill_latched_ = false;  ///< 急停锁存
  uint32_t state_ticks_ = 0;
  uint32_t change_cnt_ = 0;

  /* Ozone 观察用的实变量 */
  float rc_lv_ = 0.0f;
  float rc_lh_ = 0.0f;
  float rc_rv_ = 0.0f;
  float rc_rh_ = 0.0f;
  uint8_t rc_l_switch_ = 3;
  uint8_t rc_r_switch_ = 3;
  bool rc_online_ = false;
  bool enable_cmd_ = false;
  bool kill_cmd_ = false;
  Chassis::Velocity cmd_vel_{};
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */