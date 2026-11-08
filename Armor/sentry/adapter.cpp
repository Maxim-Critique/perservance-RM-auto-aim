#include "adapter.hpp"

#include <algorithm>
#include <limits>

namespace sentry_aim
{
std::optional<Color> color_from_a(int color)
{
  if (color == ENEMY_BLUE) return Color::blue;
  if (color == ENEMY_RED) return Color::red;
  return std::nullopt;
}

ArmorName name_from_a(const std::string & number)
{
  if (number == "1") return ArmorName::one;
  if (number == "2") return ArmorName::two;
  if (number == "3") return ArmorName::three;
  if (number == "4") return ArmorName::four;
  if (number == "5") return ArmorName::five;
  if (number == "guard" || number == "sentry") return ArmorName::sentry;
  if (number == "outpost") return ArmorName::outpost;
  if (number == "base") return ArmorName::base;
  return ArmorName::not_armor;
}

uint8_t id_for_a(ArmorName name)
{
  switch (name) {
    case ArmorName::one: return 1;
    case ArmorName::two: return 2;
    case ArmorName::three: return 3;
    case ArmorName::four: return 4;
    case ArmorName::five: return 5;
    case ArmorName::outpost: return 6;
    case ArmorName::sentry: return 7;
    case ArmorName::base: return 8;
    default: return 0;
  }
}

std::optional<Armor> observation_from_a(const ::Armor & a, int color, const Config & config)
{
  const auto enemy = color_from_a(color);
  const auto name = name_from_a(a.number);
  if (!enemy || name == ArmorName::not_armor || a.type == ::ArmorType::INVALID ||
      !a.abs_pose.allFinite() || !std::isfinite(a.yaw)) return std::nullopt;
  Armor result;
  result.color = *enemy;
  result.name = name;
  result.type = a.type == ::ArmorType::LARGE ? ArmorType::big : ArmorType::small;
  result.center = a.center;
  const auto priority = std::find(config.priority_ids.begin(), config.priority_ids.end(), id_for_a(name));
  result.priority = static_cast<int>(priority - config.priority_ids.begin());
  result.xyz_in_world = a.abs_pose;
  // A: plate = center + r*[cos(yaw),sin(yaw)]. B: plate = center - r*[cos(yaw),sin(yaw)].
  result.ypr_in_world[0] = tools::limit_rad(a.yaw + CV_PI);
  result.ypd_in_world = tools::xyz2ypd(a.abs_pose);
  return result;
}

void write_command(const Command & command, ArmorName name, bool fire, Data_Get & output)
{
  constexpr double scale = 180.0 / CV_PI * 1e4;
  const double yaw = std::isfinite(command.yaw) ? std::remainder(command.yaw, 2 * CV_PI) * scale : 0;
  const double pitch = command.pitch * scale;
  if (!command.control || !id_for_a(name) || !std::isfinite(command.yaw) ||
      !std::isfinite(pitch) || pitch < std::numeric_limits<int32_t>::min() ||
      pitch > std::numeric_limits<int32_t>::max()) {
    output.Vision2Control();
    return;
  }
  output.Vision2Control(static_cast<int32_t>(yaw), static_cast<int32_t>(pitch), id_for_a(name), 1, fire);
}
}  // namespace sentry_aim
