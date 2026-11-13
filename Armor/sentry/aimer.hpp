// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#pragma once

#include "config.hpp"
#include "target.hpp"

/**
 * @file aimer.hpp
 * @brief 当前 AutoArmor 的预测瞄准：在 Target 副本上外推状态、选择装甲板并迭代求弹道。
 * 不使用历史 ArmorPlanner 的五次多项式，也不在此发送串口或决定是否发射。
 */
namespace sentry_aim
{
/**
 * @brief 某次未来时刻选中的三维板中心及朝向，不是图像像素，也不是最终补偿后指令。
 */
struct AimPoint
{
  bool valid = false; ///< 是否确实选到了几何上可用的候选；无效时 xyza 的零值不可作为真实目标。
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero(); ///< [x,y,z,板朝向a]，m/m/m/rad，位置与 Target 使用同一参考系。
};

/**
 * @brief 依据目标平移/自转和飞行时间生成 yaw/pitch 建议，同时保存慢转目标的选板锁定索引。
 * 预测作用于 Target 的副本，不推进 Tracker 持有的实时模型；选板锁定属于本 Aimer 的跨帧状态。
 */
class Aimer
{
public:
  /// 保存并校验配置副本，初始无锁定板，也没有有效调试瞄准点。
  explicit Aimer(const Config & config) : config_(config) { config_.validate(); }
  /**
   * @brief 从观测时刻外推处理/系统延迟，再迭代计入弹丸飞行时间，生成一次绝对角度建议。
   * @param target 已估计的十一维目标模型，const 引用输入；内部复制后预测，不修改调用方状态。
   * @param timestamp 本次图像/观测的 steady_clock 时刻，应不早于模型已有时刻。
   * @param muzzle_speed 弹丸初速度，m/s，必须有限且为正；使用无空气阻力的 tools::Trajectory。
   * @param processing_delay 图像到本次处理之间的额外耗时，s，范围 [0,0.5]；
   * 不需要再加 config_.predict_time，本函数会同时计入该配置延迟。
   * @return 成功时 control=true、yaw/pitch 为 rad；yaw 加补偿并环绕，pitch 包含弹道与补偿。
   * 目标发散、前哨转速不在配置区间、无可选板、无弹道解或最多 10 次飞行时间迭代
   * 未达到 0.001 s 收敛门槛时返回默认无效 Command，不生成发射许可。
   * debug_aim_point 每次调用先清空，仅在最终求解成功后写为收敛迭代选中的预测板。
   */
  Command aim(const Target & target, std::chrono::steady_clock::time_point timestamp,
              double muzzle_speed, double processing_delay = 0);
  /// 换目标、失效或外层复位时取消慢转锁板，并清空调试预测点；不修改 Target 或配置。
  void reset() { lock_id_ = -1; debug_aim_point = {}; }
  AimPoint debug_aim_point; ///< 最近一次成功 aim 的预测 xyz/板朝向，供 Shooter 和界面观察；不是串口 yaw/pitch。

private:
  Config config_;    ///< 预测延迟、角度补偿、选板窗口、转速门槛等参数的构造时副本。
  int lock_id_ = -1; ///< 普通慢转目标锁定的模型板索引；-1 表示没有，非类别 ID；高速策略会清除此锁。
  /**
   * @brief 在给定预测时刻的模型板列表中选择一个可瞄板，返回 AimPoint。
   * @param target 应已预测到需要评估的时刻，函数不再推进其时间。
   * 未曾换到非零模型板或第 0 板距离超过 5 m 时，当前实现直接选第 0 板。
   * 否则普通慢转目标在朝向与中心方位差不超过 60 度的候选中保持/建立锁板；
   * 快转目标及前哨根据 coming/leaving 角窗和旋转方向选择，过快时缩窄窗口。
   * 分支判断可用平滑 omega()，方向判断仍沿用原始状态 w；无候选返回 valid=false。
   */
  AimPoint choose_aim_point(const Target & target);
};
}  // namespace sentry_aim
