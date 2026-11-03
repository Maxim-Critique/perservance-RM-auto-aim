// Adapted from TongjiSuperPower/sp_vision_25 main bd9f5e7. See LICENSE and SOURCES.md.
#include "shooter.hpp"

#include "math_tools.hpp"

namespace sentry_aim
{
bool Shooter::shoot(const Command & command, const AimPoint & point, const Target * target,
                    const Eigen::Vector3d & gimbal, bool tracking, bool observed)
{
  if (!command.control || !point.valid || !point.xyza.allFinite() || !target ||
      !tracking || !observed || !config_.auto_fire || target->diverged() ||
      !gimbal.allFinite() || !std::isfinite(command.yaw) || !std::isfinite(command.pitch)) {
    reset();
    return false;
  }
  const auto state = target->ekf_x();
  const double tolerance = (std::hypot(state[0], state[2]) > config_.judge_distance ?
    config_.second_tolerance : config_.first_tolerance) * CV_PI / 180.0;
  const auto previous = last_command_;
  // Compare first, then store, including on a rejected jump so stable commands can recover.
  last_command_ = command;
  return previous &&
    std::abs(tools::limit_rad(previous->yaw - command.yaw)) < tolerance * 2 &&
    std::abs(previous->pitch - command.pitch) < tolerance * 2 &&
    std::abs(tools::limit_rad(gimbal[0] - command.yaw)) < tolerance &&
    std::abs(gimbal[1] - command.pitch) < tolerance;
}
}  // namespace sentry_aim
