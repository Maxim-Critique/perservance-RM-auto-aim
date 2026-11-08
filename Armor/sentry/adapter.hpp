#pragma once

#include <optional>

#include "ArmorTypes.h"
#include "Base.h"
#include "armor.hpp"
#include "config.hpp"

namespace sentry_aim
{
std::optional<Color> color_from_a(int color);
ArmorName name_from_a(const std::string & number);
uint8_t id_for_a(ArmorName name);
std::optional<Armor> observation_from_a(const ::Armor & armor, int color, const Config & config);
void write_command(const Command & command, ArmorName name, bool fire, Data_Get & output);
}  // namespace sentry_aim
