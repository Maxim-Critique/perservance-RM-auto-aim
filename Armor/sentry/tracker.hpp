// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#pragma once

#include <optional>
#include "config.hpp"
#include "target.hpp"

namespace sentry_aim
{
class Tracker
{
public:
  enum class State { lost, detecting, tracking, temp_lost };
  explicit Tracker(const Config & config) : config_(config) { config_.validate(); }
  const Target * track(std::vector<Armor> armors, std::chrono::steady_clock::time_point t,
                       Color enemy, cv::Size image_size);
  void reset();
  State state() const { return state_; }
  bool observed() const { return observed_; }
  unsigned generation() const { return generation_; }

private:
  Config config_;
  State state_ = State::lost;
  std::optional<Target> target_;
  std::optional<std::chrono::steady_clock::time_point> last_timestamp_;
  std::optional<Color> enemy_;
  int detect_count_ = 0, lost_count_ = 0;
  bool observed_ = false;
  unsigned generation_ = 0;
  void set_target(const Armor & armor, std::chrono::steady_clock::time_point t);
};
}  // namespace sentry_aim
