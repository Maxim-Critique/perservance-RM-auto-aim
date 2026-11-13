#pragma once

#include <array>
#include <yaml-cpp/yaml.h>

/**
 * @file config.hpp
 * @brief 当前 Tracker、Aimer、Shooter 使用的配置快照，默认值可由 YAML 覆盖。
 * 注意配置角度门槛与补偿以“度”存储，Command/状态向量内则使用 rad，转换在使用处完成。
 */
namespace sentry_aim
{
/**
 * @brief 自瞄核心参数集合，不包含相机内参、相机设备或导航配置。
 * Tracker/Aimer/Shooter 构造时各复制一份并 validate，修改外部 Config 不会自动热更新既有对象。
 */
struct Config
{
  int min_detect_count = 5;          ///< 从 detecting 确认为 tracking 所需连续观测次数，含建目标首帧；至少为 1。
  int max_temp_lost_count = 15;      ///< 普通目标短时丢失允许的连续空观测帧数，超过才放弃；不是毫秒。
  int outpost_max_temp_lost_count = 75; ///< 前哨站专用的连续空观测帧数上限，可与普通目标不同。
  double max_frame_gap = 0.1;        ///< 最大连续帧间隔，s；超过或时间戳不递增时 Tracker 复位。
  double max_distance = 12;          ///< 观测板距参考原点的最大三维距离，m；Tracker 另硬编码剔除小于 0.1 m 的点。
  double predict_time = 0.08;        ///< Transmit.predict_time：额外系统预测延迟，s；与处理延迟和弹丸飞行时间共同使用。
  double yaw_offset = 0;             ///< Transmit.compensate_yaw：最终 yaw 的加法补偿，度，正负按接口约定。
  double pitch_offset = 0;           ///< Transmit.compensate_pitch：最终弹道 pitch 的加法补偿，度。
  double coming_angle = 70;          ///< 高速旋转选板的进入侧角度窗口，度；(0,90]，高速时实现会进一步缩小。
  double leaving_angle = 20;         ///< 高速旋转选板的离开侧角度门槛，度；[0,coming_angle]，与旋转方向共同决定可选板。
  double min_spin_speed = 2;         ///< 普通目标慢转/快转选板策略的分界，rad/s；前哨不走普通慢转分支。
  double first_tolerance = 2.5;       ///< 目标中心水平距离不超过 judge_distance 时，yaw/pitch 对齐容差，度。
  double second_tolerance = 2.5;      ///< 超过 judge_distance 时的对齐容差，度；历史命令变化门槛为该容差的两倍。
  double outpost_min_speed = 2.0;    ///< 前哨允许产生控制建议的 |状态 omega| 下限，rad/s；门控的是控制，不仅是开火。
  double outpost_max_speed = 3.0;    ///< 上述前哨角速度绝对值上限，rad/s；允许区间包含两端，取状态索引 7。
  double judge_distance = 2;         ///< 近/远开火容差的切换距离，m，按目标中心 hypot(xc,yc) 判断。
  bool auto_fire = true;             ///< 允许 Shooter 产生视觉发射许可；不等于绕过外部人工许可、裁判或电控约束。
  /// 旧协议 ID 的优先级排列，数组前部优先；必须恰好包含 1~8 且不重复，不含裁判无敌信息。
  std::array<int, 8> priority_ids{{1, 3, 4, 7, 6, 8, 2, 5}};

  /**
   * @brief 先使用成员默认值，再读取 Transmit 的延迟/角度补偿和 Sentry 下的其余配置。
   * @param yaml 完整工程 YAML 根节点，而不是只传 yaml["Sentry"]。
   * @return 已调用 validate 的配置；缺失字段使用默认值，类型错误或不合法值向外抛异常。
   * priority_ids 若出现，必须为八个 ID 的列表，不接受局部优先级覆盖。
   */
  static Config from_yaml(const YAML::Node & yaml);
  /**
   * @brief 校验计数范围、有限数、正值条件、预测延迟 [0,0.5] s、选板角度顺序及 ID 排列。
   * 不合法时抛 std::invalid_argument；这是配置结构校验，不验证实机标定或实际瞄准精度。
   */
  void validate() const;
};
}  // namespace sentry_aim
