#include "config.hpp"

#include <cmath>
#include <algorithm>
#include <set>
#include <stdexcept>
#include <vector>

namespace sentry_aim
{
Config Config::from_yaml(const YAML::Node & yaml)
{
  Config c;
  if (const auto tx = yaml["Transmit"]) {
    c.predict_time = tx["predict_time"].as<double>(c.predict_time);
    c.yaw_offset = tx["compensate_yaw"].as<double>(0);
    c.pitch_offset = tx["compensate_pitch"].as<double>(0);
  }
  if (const auto n = yaml["Sentry"]) {
    c.min_detect_count = n["min_detect_count"].as<int>(c.min_detect_count);
    c.max_temp_lost_count = n["max_temp_lost_count"].as<int>(c.max_temp_lost_count);
    c.outpost_max_temp_lost_count = n["outpost_max_temp_lost_count"].as<int>(c.outpost_max_temp_lost_count);
    c.max_frame_gap = n["max_frame_gap"].as<double>(c.max_frame_gap);
    c.max_distance = n["max_distance"].as<double>(c.max_distance);
    c.coming_angle = n["coming_angle"].as<double>(c.coming_angle);
    c.leaving_angle = n["leaving_angle"].as<double>(c.leaving_angle);
    c.min_spin_speed = n["min_spin_speed"].as<double>(c.min_spin_speed);
    c.first_tolerance = n["first_tolerance"].as<double>(c.first_tolerance);
    c.second_tolerance = n["second_tolerance"].as<double>(c.second_tolerance);
    c.judge_distance = n["judge_distance"].as<double>(c.judge_distance);
    c.auto_fire = n["auto_fire"].as<bool>(c.auto_fire);
    if (n["priority_ids"]) {
      const auto ids = n["priority_ids"].as<std::vector<int>>();
      if (ids.size() != c.priority_ids.size())
        throw std::invalid_argument("Sentry.priority_ids must contain all eight A IDs");
      std::copy(ids.begin(), ids.end(), c.priority_ids.begin());
    }
  }
  c.validate();
  return c;
}

void Config::validate() const
{
  const auto positive = [](double x) { return std::isfinite(x) && x > 0; };
  if (min_detect_count < 1 || max_temp_lost_count < 0 || outpost_max_temp_lost_count < 0 ||
      !positive(max_frame_gap) || !positive(max_distance) ||
      !std::isfinite(predict_time) || predict_time < 0 || predict_time > 0.5 ||
      !std::isfinite(yaw_offset) || !std::isfinite(pitch_offset) ||
      !positive(coming_angle) || coming_angle > 90 ||
      !std::isfinite(leaving_angle) || leaving_angle < 0 || leaving_angle > coming_angle ||
      !positive(min_spin_speed) || !positive(first_tolerance) ||
      !positive(second_tolerance) || !positive(judge_distance))
    throw std::invalid_argument("invalid Sentry configuration");
  std::set<int> ids(priority_ids.begin(), priority_ids.end());
  if (ids != std::set<int>({1, 2, 3, 4, 5, 6, 7, 8}))
    throw std::invalid_argument("Sentry.priority_ids must be a permutation of A IDs 1..8");
}
}  // namespace sentry_aim
