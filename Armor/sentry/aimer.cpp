// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#include "aimer.hpp"

#include <algorithm>
#include "math_tools.hpp"
#include "trajectory.hpp"

namespace sentry_aim
{
namespace { constexpr double radians = CV_PI / 180.0; }

Command Aimer::aim(const Target & source, std::chrono::steady_clock::time_point timestamp,
                   double muzzle_speed, double processing_delay)
{
  debug_aim_point = {};
  if (source.diverged() || !std::isfinite(muzzle_speed) || muzzle_speed <= 0 ||
      !std::isfinite(processing_delay) || processing_delay < 0 || processing_delay > 0.5) return {};
  const auto future = timestamp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(config_.predict_time + processing_delay));
  auto target = source;
  target.predict(future);
  auto point = choose_aim_point(target);
  if (!point.valid) return {};
  tools::Trajectory trajectory(muzzle_speed, point.xyza.head<2>().norm(), point.xyza[2]);
  if (trajectory.unsolvable) return {};

  // Every iteration predicts a fresh copy from the same firing instant.
  double flight = trajectory.fly_time;
  for (int iter = 0; iter < 10; ++iter) {
    auto predicted = target;
    predicted.predict(future + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(flight)));
    point = choose_aim_point(predicted);
    if (!point.valid || !point.xyza.allFinite()) return {};
    trajectory = tools::Trajectory(muzzle_speed, point.xyza.head<2>().norm(), point.xyza[2]);
    if (trajectory.unsolvable) return {};
    if (std::abs(trajectory.fly_time - flight) < 0.001) {
      debug_aim_point = point;
      return {true, tools::limit_rad(std::atan2(point.xyza[1], point.xyza[0]) +
                                    config_.yaw_offset * radians),
                    trajectory.pitch + config_.pitch_offset * radians};
    }
    flight = trajectory.fly_time;
  }
  return {};
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  const auto x = target.ekf_x();
  const auto armors = target.armor_xyza_list();
  if (armors.empty()) return {};
  if (!target.jumped || armors.front().head<3>().norm() > 5) return {true, armors.front()};
  const double center_yaw = std::atan2(x[2], x[0]);
  std::vector<double> delta;
  for (const auto & a : armors) delta.push_back(tools::limit_rad(a[3] - center_yaw));
  const double omega = target.omega().value_or(x[7]);

  if (std::abs(omega) <= config_.min_spin_speed && target.name != ArmorName::outpost) {
    std::vector<int> visible;
    for (std::size_t i = 0; i < armors.size(); ++i)
      if (std::abs(delta[i]) <= 60 * radians) visible.push_back(static_cast<int>(i));
    if (visible.empty()) return {};
    if (std::find(visible.begin(), visible.end(), lock_id_) == visible.end())
      lock_id_ = *std::min_element(visible.begin(), visible.end(), [&](int a, int b) {
        return std::abs(delta[a]) < std::abs(delta[b]);
      });
    return {true, armors[lock_id_]};
  }
  lock_id_ = -1;
  const double reduction = std::abs(omega) >= 14 ? 20 : std::abs(omega) > 8 ? 10 : 0;
  const double coming = std::max(0.0, config_.coming_angle - reduction) * radians;
  const double leaving = std::max(0.0, config_.leaving_angle - reduction * 0.5) * radians;
  for (std::size_t i = 0; i < armors.size(); ++i) {
    if (std::abs(delta[i]) > coming) continue;
    if ((x[7] > 0 && delta[i] < leaving) || (x[7] < 0 && delta[i] > -leaving))
      return {true, armors[i]};
  }
  return {};
}
}  // namespace sentry_aim
