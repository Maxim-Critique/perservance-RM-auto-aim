// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#pragma once

#include <optional>
#include "config.hpp"
#include "target.hpp"

/**
 * @file tracker.hpp
 * @brief 当前 AutoArmor 使用的主相机单目标跟踪状态机，输入为 adapter 转换后的世界系观测。
 * 不包含 USB 全感知搜索、导航、串口写入或发射；这些由外层模块协调。
 */
namespace sentry_aim
{
/**
 * @brief 选择目标类别、管理观测确认/丢失并维护一个 Target 模型。
 * 基本流程：lost -> detecting -> tracking -> temp_lost；短时重获回 tracking，超时回 lost。
 * 每个采图时刻调用一次 track；无检测也应传空列表，以推进预测和丢失计数。
 */
class Tracker
{
public:
  /**
   * @brief 状态反映确认与观测连续性，不等于是否一定返回目标指针。
   * lost：无模型；detecting：已有候选但未满确认次数；tracking：已确认且本帧可见；
   * temp_lost：已确认目标短时不可见，仅做预测。detecting/temp_lost 也可能返回非空 Target。
   */
  enum class State { lost, detecting, tracking, temp_lost };
  /// 保存配置副本并检查有效性；初始无目标，状态为 lost。
  explicit Tracker(const Config & config) : config_(config) { config_.validate(); }
  /**
   * @brief 对一帧观测做筛选、目标选择、状态预测和单板更新，返回当前模型的借用指针。
   * @param armors 可为空，按值接收后剔除颜色/类别/有限性/距离无效的项并排序。
   * @param t 本帧图像对应的 steady_clock 时刻；不递增或间隔超过 max_frame_gap 时重建跟踪。
   * @param enemy 本次要跟踪的敌方颜色；颜色改变也会复位。
   * @param image_size 图像宽高，像素，用于优先级相同时按距画面中心远近排序。
   * @return 当前 Target；丢失、检测确认期间断帧或模型发散时可能为 nullptr。
   * 指针由本对象拥有，会在 reset 或换目标时失效，外部不应跨帧长期持有。
   * 已跟踪时仅更高优先级目标可主动抢占；同类同尺寸多块观测中，选图像最左侧一块更新。
   * 开火方还需同时检查 state()==tracking 与 observed()，不能仅凭返回非空判定可以开火。
   */
  const Target * track(std::vector<Armor> armors, std::chrono::steady_clock::time_point t,
                       Color enemy, cv::Size image_size);
  /// 清空模型、时刻、颜色和计数并进入 lost；generation_ 不清零，以便外层识别后续新模型。
  void reset();
  /// 查询当前四态枚举，不修改跟踪。
  State state() const { return state_; }
  /// 本帧是否用实测板初始化或更新了模型；temp_lost 预测帧为 false，不表示已满足确认帧数。
  bool observed() const { return observed_; }
  /// 模型代次，每次 set_target 增加；外层据此清空 Aimer 锁板和 Shooter 命令历史。
  unsigned generation() const { return generation_; }

private:
  Config config_;                         ///< 构造时复制的筛选、确认、丢失和优先级配置。
  State state_ = State::lost;              ///< 当前跟踪状态，不含外层的 USB 搜索/500 ms 保持状态。
  std::optional<Target> target_;           ///< 当前唯一目标的十一维 EKF 模型，空值表示尚未建立。
  std::optional<std::chrono::steady_clock::time_point> last_timestamp_; ///< 上次 track 时刻，用于检测乱序和长间隔。
  std::optional<Color> enemy_;             ///< 上一次请求的敌色，识别颜色切换时防止沿用旧模型。
  int detect_count_ = 0, lost_count_ = 0;   ///< 前者计候选连续可见次数（首帧为 1）；后者计连续不可见次数。
  bool observed_ = false;                  ///< 当前帧是否确有同类、同尺寸观测被接入，而非仅沿用预测。
  unsigned generation_ = 0;                ///< 模型建立/切换的累计代次，非目标类别 ID。
  /**
   * @brief 用单块观测创建新 Target，重置计数并增加代次。
   * 大板 3/4/5 号用二板模型，前哨/基地用三板模型，其余用四板模型。
   * 初始半径分别为前哨 0.2765 m、基地 0.3205 m、其他 0.2 m；初始协方差为十一维。
   * min_detect_count==1 时直接 tracking，否则进入 detecting。
   */
  void set_target(const Armor & armor, std::chrono::steady_clock::time_point t);
};
}  // namespace sentry_aim
