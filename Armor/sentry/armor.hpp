// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

/**
 * @file armor.hpp
 * @brief 当前 sentry 追踪链路内部的观测与瞄准指令类型，由 adapter 连接旧检测/解算数据。
 * 不要与全局命名空间中的 ::Armor、::ArmorType 混用，也不要直接强转两套枚举。
 * 数据流：::Armor -> observation_from_a -> sentry_aim::Armor -> Tracker/Target -> Command。
 */
namespace sentry_aim
{
/// 追踪器敌我筛选的颜色枚举；顺序 red/blue 与 ENEMY_BLUE=0、ENEMY_RED=1 不同。
enum class Color { red, blue };
/// 模型使用的板尺寸；big/small 的顺序与旧全局 ArmorType 的 SMALL/LARGE 不同。
enum class ArmorType { big, small };
/// 目标类别，不是跟踪 UUID 或串口 ID；串口映射必须调用 id_for_a，不能直接发送枚举下标。
enum class ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
/// 选择目标的排序权重，数值越小优先级越高；通常来自 Config.priority_ids 中的位置。
using ArmorPriority = int;

/**
 * @brief 一块已经过检测与位姿解算、可以参与追踪的装甲板观测。
 *
 * 位置单位 m，角度 rad，center 为像素。world 指沿用旧 Slover 姿态补偿的参考系，
 * 位置没有在 adapter 中额外加底盘导航平移，不能据此当作导航 map 系绝对位置。
 * 类别/颜色/位置等字段由 adapter 校验并转换；本结构不携带相机、串口或 ROS 对象。
 */
struct Armor
{
  Color color = Color::blue;                         ///< 实测板颜色，用于保留与本帧 enemy 一致的目标。
  ArmorType type = ArmorType::small;                 ///< 几何大小板，参与目标模型板数选择与同类关联。
  ArmorName name = ArmorName::not_armor;              ///< 识别类别；not_armor 表示未识别或不可用于建模。
  ArmorPriority priority = 100;                      ///< 目标优先级，默认 100 为占位；adapter 按配置生成较小索引。
  cv::Point2f center{};                              ///< 原图板中心 (u,v)，像素，用于画面中心排序与左侧板选择。
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero(); ///< 板中心在姿态补偿参考系中的 (x,y,z)，m。
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero(); ///< 板朝向 (yaw,pitch,roll)，rad；当前 adapter 只写 yaw，其余为 0。
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero(); ///< 从参考原点指向板中心的 (方位角,仰角,三维距离)，rad/rad/m。
};

/**
 * @brief Aimer 产生的绝对角度建议，经 adapter 转为旧串口数据包。
 * 不包含开火字段；开火由 Shooter 另行判断，也不表示电控已接收或执行。
 */
struct Command
{
  bool control = false; ///< 是否存在有效控制建议；false 时不能把默认的零角度解释为指向原点。
  double yaw = 0;       ///< 姿态参考系下的目标方位角，rad，约定朝左为正；Aimer 会加补偿并归一化。
  double pitch = 0;     ///< 含弹道/安装补偿的目标俯仰角，rad，约定抬头为正；不是目标位置的几何仰角。
};
}  // namespace sentry_aim
