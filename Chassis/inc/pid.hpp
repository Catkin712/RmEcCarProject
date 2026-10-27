/*
 * @Author: Catkin712 1465852795@qq.com
 * @Date: 2026-10-26 19:23:18
 * @LastEditors: Catkin712 1465852795@qq.com
 * @LastEditTime: 2026-10-26 19:23:33
 * @FilePath: \RmEcCarProject\Chassis\inc\pid.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      : pid.hpp
 * @brief     : 通用 PID 控制器（位置式，带积分/输出限幅）
 *******************************************************************************
 */
#ifndef HELLO_WORLD_PID_HPP_
#define HELLO_WORLD_PID_HPP_

namespace hello_world
{
namespace pid
{

class Pid
{
 public:
  /* 参数集合：按“物理量纲”定义，换采样率不用重调 */
  struct Config
  {
    float kp = 0.0f;       ///< 比例增益
    float ki = 0.0f;       ///< 积分增益，单位 1/s
    float kd = 0.0f;       ///< 微分增益，单位 s
    float ts = 0.001f;     ///< 采样周期，单位 s（必须等于实际调用周期）
    float out_lim = 0.0f;  ///< 输出限幅，输出被钳到 [-out_lim, +out_lim]
    float i_lim = 0.0f;    ///< 积分项限幅，同上
  };

  Pid(void) = default;
  explicit Pid(const Config& cfg) { init(cfg); }

  void init(const Config& cfg);              ///< 配参数 + 清状态
  float calc(float ref, float fb);           ///< 每拍调用一次，返回限幅后的 u
  void reset(void);                          ///< 清掉历史（模式切换时用）

  float output(void) const { return output_; }
  float i_term(void) const { return i_term_; }
  bool saturated(void) const { return saturated_; }

 private:
  Config cfg_{};
  float err_ = 0.0f;        ///< e(k)
  float err_last_ = 0.0f;   ///< e(k-1)
  float i_term_ = 0.0f;     ///< 积分累计值（已经乘过 ki*ts）
  float output_ = 0.0f;     ///< 本拍输出（限幅后）
  bool saturated_ = false;  ///< 上一拍输出有没有被限幅
};

}  // namespace pid
}  // namespace hello_world

#endif /* HELLO_WORLD_PID_HPP_ */