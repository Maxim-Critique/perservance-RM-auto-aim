#ifndef FUSION_VENDOR_TOOLS__LOGGER_HPP
#define FUSION_VENDOR_TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>

namespace tongji_tools
{
std::shared_ptr<spdlog::logger> logger();

}  // namespace tongji_tools

#endif  // FUSION_VENDOR_TOOLS__LOGGER_HPP