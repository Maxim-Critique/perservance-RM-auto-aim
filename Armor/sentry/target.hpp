// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#ifndef SENTRY_AIM__TARGET_HPP
#define SENTRY_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "extended_kalman_filter.hpp"

/**
 * @file target.hpp
 * @brief 当前主追踪链路的整车状态模型，使用 tools::ExtendedKalmanFilter。
 * 与 Armor/include/ArmorTarget.h 的历史 Aim / 十维状态不同，本模型为十一维：
 * [xc, vx, yc, vy, z, vz, a, w, r, l, h]。
 * xc/yc/z 为旋转中心平面位置和基准板高度（m），vx/vy/vz 为 m/s，
 * a 为索引 0 板朝向（rad），w 为角速度（rad/s），r 为基准半径（m），
 * l=r2-r1、h=z2-z1（m）。四板奇数索引使用 r+l、z+h；二/三板忽略 l/h 几何偏移。
 */
namespace sentry_aim
{

/**
 * @brief 用一块板的观测初始化整车模型，随后以平移/旋转匀速模型预测并融合单板观测。
 *
 * 板位置采用 center - radius*(cos(a),sin(a))，与旧运动模型的加法约定不同，
 * adapter 已通过板朝向加 pi 适配。位置沿用 Slover 的姿态补偿参考系，不自动接导航位置。
 * 典型调用顺序为 Tracker 先 predict(t)，再 update(armor)；本对象不处理图像或串口。
 */
class Target
{
public:
  ArmorName name;           ///< 创建模型时的识别类别，供关联、预测特判和协议输出使用。
  ArmorType armor_type;     ///< 创建时的大小板类型；Tracker 要求后续关联观测同类型。
  ArmorPriority priority;   ///< 创建时保存的优先级，数值越小越优先，供更高优先级目标抢占。
  bool jumped;             ///< 曾匹配到非 0 号模型板后保持 true；不是仅本帧是否切板的瞬时标志。
  int last_id;             ///< 最近一次 update 匹配的模型板索引，范围 0~armor_num_-1；不是识别类别/电控 ID。

  /**
   * @brief 由单块观测建立中心位置、几何初值与十一维 EKF。
   * @param armor 已解算并经 adapter 转换的观测，提供类别、优先级、xyz 和板 yaw。
   * @param t 当前观测的 steady_clock 时刻，作为模型内部时间起点。
   * @param radius 基准初始半径，m；中心初始化为观测位置加朝向径向偏移。
   * @param armor_num 模型板数，只接受 2、3、4；由 Tracker 根据类别/大小板决定。
   * @param P0_dig 十一个初始协方差对角元素，顺序与文件说明一致，不是标准差。
   * 板数或协方差长度不合法时抛 std::invalid_argument；速度、角速度及 l/h 初始为 0。
   */
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);

  /**
   * @brief 将本对象的状态与协方差推进到绝对时刻 t，并更新内部 t_。
   * @param t 必须不早于上次模型时刻；倒退或非法时间差会抛 std::invalid_argument。
   * 平移为 xyz 匀速，朝向为 a+w*dt 并归一化；几何参数保持不变。
   * 过程噪声按普通/前哨取不同强度。前哨在 convergened() 成立且 |w|>2 时，
   * 先按当前符号将 w 设为 +/-2.51 rad/s，再完成预测。
   */
  void predict(std::chrono::steady_clock::time_point t);
  /**
   * @brief 把一块观测匹配到模型中的板索引，完成 EKF 更新与角速度统计。
   * @param armor 应为同一目标类别/尺寸、同一参考系且与模型当前时刻对应的观测。
   * 候选先按距参考原点远近排序，在最近至多三块模型板中比较“板朝向差+位置方位角差”。
   * 匹配后更新 last_id、切板计数、jumped，再以位置球坐标与板 yaw 做四维观测更新。
   * 本函数不会自行 predict 或校验类别/颜色，通常由 Tracker 统一处理。
   */
  void update(const Armor & armor);

  /// 返回平滑角速度估计，rad/s；累计 150 次更新前为空，Aimer 此时回退使用状态 w。
  std::optional<double> omega() const;

  /// 复制返回当前十一维状态；只读该副本不会改变内部滤波器，索引 7 才是角速度。
  Eigen::VectorXd ekf_x() const;
  /// 返回内部通用 EKF 的只读引用，可观察 x/P 和统计量；引用生命周期依赖当前 Target。
  const tools::ExtendedKalmanFilter & ekf() const;
  /**
   * @brief 按当前状态展开所有模型装甲板，不额外推进时间。
   * @return 每项 [x,y,z,板朝向a]，单位 m/m/m/rad，数量为 armor_num_。
   * 相邻板角度差 2*pi/armor_num_，四板中索引 1/3 使用第二组半径与高度。
   */
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  /**
   * @brief 判断状态/协方差是否非有限，或 r 与 r+l 是否超出开区间 (0.05,0.5) m。
   * 二/三板也检查 r+l；这是当前数值/几何检查，不是完整的跟踪质量或命中率证明。
   */
  bool diverged() const;

  /**
   * @brief 旧拼写保留的收敛标志更新/查询接口，内部可能将 is_converged_ 置 true。
   * 当前条件是 update_count_>3 且 !diverged()，不是注释历史说法中的“旋转两圈”。
   * 一旦置 true 就保持，直到整个 Target 被替换；predict 用它决定前哨定速特判。
   */
  bool convergened();

  bool isinit = false; ///< 预留公有初始化标记；当前内部流程没有将其置 true，不代表 EKF 已确认收敛。

  /// 原样返回 isinit，不触发初始化、预测或收敛判断。
  bool checkinit();

private:
  int armor_num_;     ///< 模型板数 2/3/4，决定相位间隔及是否启用交替半径/高度。
  int switch_count_;  ///< update 匹配索引不同于 last_id 的累计次数；当前主要保留作内部统计。
  int update_count_;  ///< 成功进入 update 的累计次数，用于 convergened() 的当前计数条件。

  bool is_switch_, is_converged_; ///< 前者表示最近一次观测是否换模型板；后者是保持式收敛标记。

  tools::ExtendedKalmanFilter ekf_; ///< 实际使用的通用 EKF，持有十一维 x 和 11x11 P，区别于历史全局同名类。
  std::deque<double> omega_queue_;  ///< 更新后 w 的统计队列；满 150 项求均值，再移除最早一半 75 项。
  std::optional<double> omega_;     ///< 最近一次队列均值，rad/s；没有形成完整窗口前为空，非逐帧即时速度。
  std::chrono::steady_clock::time_point t_; ///< 当前 EKF 状态所对应的单调时刻，不是 UTC 或曝光参数。

  /**
   * @brief 对已关联的 id 构造四维 z=[位置yaw,位置pitch,距离,板yaw] 并调用通用 EKF 更新。
   * 根据距离/朝向构造 R，用 h_jacobian 线性化，对角度残差归一化；id 必须是有效模型索引。
   */
  void update_ypda(const Armor & armor, int id);
  /// 将一次滤波后的角速度样本（rad/s）加入统计队列，在完整窗口时更新 omega_。
  void update_omega(double omega);

  /// 从十一维状态 x 和模型索引 id 计算该板中心 xyz（m），采用中心减径向偏移的约定。
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  /// 返回四维球坐标/板朝向观测对十一维状态的 4x11 雅可比，包含 xyz 到 ypd 的链式求导。
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace sentry_aim

#endif  // SENTRY_AIM__TARGET_HPP
