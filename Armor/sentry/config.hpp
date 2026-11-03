#pragma once

#include <array>
#include <yaml-cpp/yaml.h>

namespace sentry_aim
{
struct Config
{
  int min_detect_count = 5;
  int max_temp_lost_count = 15;
  int outpost_max_temp_lost_count = 75;
  double max_frame_gap = 0.1;
  double max_distance = 12;
  double predict_time = 0.07;
  double yaw_offset = 0;
  double pitch_offset = 0;
  double coming_angle = 70;
  double leaving_angle = 20;
  double min_spin_speed = 2;
  double first_tolerance = 1.5;
  double second_tolerance = 1.5;
  double judge_distance = 2;
  bool auto_fire = true;
  // A's IDs, in priority order. Referee invincibility data is not present in A's input contract.
  std::array<int, 8> priority_ids{{1, 3, 4, 7, 6, 8, 2, 5}};

  static Config from_yaml(const YAML::Node & yaml);
  void validate() const;
};
}  // namespace sentry_aim
