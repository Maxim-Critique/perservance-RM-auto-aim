// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#pragma once

#include "config.hpp"
#include "target.hpp"

namespace sentry_aim
{
struct AimPoint
{
  bool valid = false;
  Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
};

class Aimer
{
public:
  explicit Aimer(const Config & config) : config_(config) { config_.validate(); }
  Command aim(const Target & target, std::chrono::steady_clock::time_point timestamp,
              double muzzle_speed, double processing_delay = 0);
  void reset() { lock_id_ = -1; debug_aim_point = {}; }
  AimPoint debug_aim_point;

private:
  Config config_;
  int lock_id_ = -1;
  AimPoint choose_aim_point(const Target & target);
};
}  // namespace sentry_aim
