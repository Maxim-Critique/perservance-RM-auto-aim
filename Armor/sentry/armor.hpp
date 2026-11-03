// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

namespace sentry_aim
{
enum class Color { red, blue };
enum class ArmorType { big, small };
enum class ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
using ArmorPriority = int;

// A's detector and calibrated solver supply these observations; no B hardware types cross this boundary.
struct Armor
{
  Color color = Color::blue;
  ArmorType type = ArmorType::small;
  ArmorName name = ArmorName::not_armor;
  ArmorPriority priority = 100;
  cv::Point2f center{};
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();
};

struct Command
{
  bool control = false;
  double yaw = 0;    // radians, left positive
  double pitch = 0;  // radians, up positive
};
}  // namespace sentry_aim
