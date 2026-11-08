// Command-history ordering follows sp_vision_25 main bd9f5e7. See LICENSE and SOURCES.md.
#pragma once

#include "aimer.hpp"

namespace sentry_aim
{
class Shooter
{
public:
  explicit Shooter(const Config & config) : config_(config) { config_.validate(); }
  bool shoot(const Command & command, const AimPoint & point, const Target * target,
             const Eigen::Vector3d & gimbal, bool tracking, bool observed);
  void reset() { last_command_.reset(); }

private:
  Config config_;
  std::optional<Command> last_command_;
};
}  // namespace sentry_aim
