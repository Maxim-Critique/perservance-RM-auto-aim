#pragma once

#include <optional>

#include "ArmorTypes.h"
#include "Base.h"
#include "armor.hpp"
#include "config.hpp"

/**
 * @file adapter.hpp
 * @brief 当前 AutoArmor 的数据桥：旧检测/标定接口与 sentry 追踪/瞄准接口之间的显式转换。
 * 输入 ::Armor 与输出 sentry_aim::Armor 同名但含义/朝向约定不同；本层处理类别、单位和协议映射。
 */
namespace sentry_aim
{
/**
 * @brief 将旧敌方颜色编码映射为追踪枚举，不按枚举底层数值直接强转。
 * @param color ENEMY_BLUE=0 或 ENEMY_RED=1。
 * @return 对应 Color，其他数值返回 nullopt，调用方应丢弃该观测或复位跟踪。
 */
std::optional<Color> color_from_a(int color);

/**
 * @brief 将分类字符串 1~5、guard/sentry、outpost、base 转成目标枚举。
 * guard 与 sentry 都映射为哨兵；未知标签（含 negative）返回 not_armor。
 */
ArmorName name_from_a(const std::string & number);

/**
 * @brief 转回旧电控使用的类别 ID：数字 1~5、前哨 6、哨兵 7、基地 8，其他为 0。
 * 返回的是协议类别，不是目标实例 ID，也与 ArmorName 的枚举下标不同。
 */
uint8_t id_for_a(ArmorName name);

/**
 * @brief 将已解算的旧装甲板转换为一条追踪观测，不执行 PnP 或时间同步。
 * @param armor 读取全局 ::Armor 的 number/type/center/abs_pose/yaw，位置须为 m，yaw 须为 rad。
 * @param color 与 color_from_a 相同的旧敌色编码。
 * @param config 使用 priority_ids 的数组下标给本观测生成优先级。
 * @return 无效颜色、类别、板类型或非有限位置/yaw 时返回 nullopt；有效时保留像素中心与位置，
 * 重算 ypd，并用 limit_rad(armor.yaw+pi) 转换板朝向，适配两套模型相反的径向约定。
 * ypr 的 pitch/roll 保持默认 0；confidence 不在本桥中再次做阈值筛选。
 */
std::optional<Armor> observation_from_a(const ::Armor & armor, int color, const Config & config);

/**
 * @brief 将绝对角度建议与独立发射许可写入 Data_Get，不直接写串口。
 * @param command yaw/pitch 单位 rad；yaw 先按 2*pi 环绕，再一起转换为“度乘 10000”的 int32。
 * @param name 用于生成协议 ID；not_armor 不能产生有效目标包。
 * @param fire Shooter 或调用方给出的布尔发射许可，编码为 1/0，本函数不重新做开火判定。
 * @param output 已构造的数据包；写角度、ID/found/fire，保留原有导航字节。
 * 正常输出 found=1；control=false、无效类别、非有限角度或 pitch 编码越界时清空视觉字段。
 * 数据包索引 2~5/6~9 为大端 yaw/pitch，17/18/19 为 ID/found/fire；没有电控执行确认。
 */
void write_command(const Command & command, ArmorName name, bool fire, Data_Get & output);
}  // namespace sentry_aim
