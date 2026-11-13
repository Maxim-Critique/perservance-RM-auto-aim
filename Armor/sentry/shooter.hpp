// Command-history ordering follows sp_vision_25 main bd9f5e7. See LICENSE and SOURCES.md.
#pragma once

#include "aimer.hpp"

/**
 * @file shooter.hpp
 * @brief 当前 AutoArmor 中“是否允许发射”的视觉判定器，不是发射机构驱动。
 * 使用 Tracker 的状态/真实观测标记、Aimer 的有效预测点、反馈角度及历史建议共同判断。
 */
namespace sentry_aim
{
/**
 * @brief 只有目标已确认、本帧确有观测、瞄准结果稳定且当前云台对齐时，返回发射许可。
 * temp_lost 预测帧不允许开火；调用方可再叠加人工空格等外部许可，最后由 adapter 编码。
 * 本类不读取按键，不控制摩擦轮，不处理弹量/热量或裁判状态，也不确认实际发射。
 */
class Shooter
{
public:
  /// 保存并校验参数副本；历史命令初始为空，所以第一个有效建议不能立即获得许可。
  explicit Shooter(const Config & config) : config_(config) { config_.validate(); }
  /**
   * @brief 根据本次建议和上一有效建议返回一次布尔发射许可，并更新历史。
   * @param command 本次 Aimer 的绝对 yaw/pitch 建议（rad），control 必须有效。
   * @param point 同次瞄准的预测板，valid 且四个分量有限；不在此重新求解弹道。
   * @param target 当前模型借用指针，不得为空或已发散；须在本次调用期间有效。
   * @param gimbal [反馈yaw,反馈pitch,反馈roll]，rad，须与 Command 使用相同参考系和正方向。
   * 当前实现检查三项有限，但仅用 yaw/pitch 做对齐比较。
   * @param tracking 外层是否处于 Tracker::State::tracking；detecting/temp_lost 应传 false。
   * @param observed 本帧是否确实关联到实测板，不能把预测沿用当作 true。
   * @return 满足 auto_fire 开关、输入有效性、历史稳定性与当前对齐条件时返回 true。
   * 近/远容差按目标中心水平距离选择，当前 yaw/pitch 误差须各小于 tolerance，
   * 与上个命令的角度变化须各小于 2*tolerance；yaw 差使用周期归一化。
   * 先保存旧命令副本，再记录本次命令并比较：因此有效但跳变的命令仍被记为下一帧基准；
   * 输入/追踪门控失败则清空历史，恢复后的首个命令仍返回 false。
   */
  bool shoot(const Command & command, const AimPoint & point, const Target * target,
             const Eigen::Vector3d & gimbal, bool tracking, bool observed);
  /// 清除命令历史，供失效、换目标、模式/颜色切换时调用；不改变配置或目标模型。
  void reset() { last_command_.reset(); }

private:
  Config config_;                       ///< 自动发射开关、近远距离和对齐阈值的构造时快照。
  std::optional<Command> last_command_; ///< 上次通过输入/追踪门控的建议；不要求该次 shoot 返回 true，也不是实测反馈。
};
}  // namespace sentry_aim
