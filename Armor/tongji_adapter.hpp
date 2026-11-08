#pragma once
#include <memory>
#include <list>
#include "Base.h"
#include "ArmorPredictor.h"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/tracker.hpp"

namespace fusion {
// 不压缩为旧的十维状态：为原车 Planner 提供同济完整运动模型快照。
class TongjiModel final : public KinematicModel {
public:
  explicit TongjiModel(const auto_aim::Target &target);
  Armors GetArmors(double predict_time = 0) override;
private:
  Eigen::VectorXd state_;
};

class TongjiAdapter {
public:
  TongjiAdapter(const std::string &car_config, const std::string &algorithm_config,
                bool load_model = true);
  std::shared_ptr<KinematicModel> process(const cv::Mat &image, const Data_Save &data,
                                        bool input_rgb);
  // 离线测试注入角点，不包含任何设备或通信对象。
  std::shared_ptr<KinematicModel> update(std::list<auto_aim::Armor> armors,
                                       const Data_Save &data, cv::Size image_size);
  std::list<auto_aim::Armor> detect_only(const cv::Mat &image, bool input_rgb);
  void reset();
  bool resolve_type(auto_aim::Armor &armor, int mode) const;
  bool accept_type(auto_aim::Armor &armor, int mode) const;
  auto_aim::Solver &solver() { return *solver_; }
  std::string state() const { return tracker_ ? tracker_->state() : "lost"; }
  const std::list<auto_aim::Armor> &detections() const { return detections_; }
  int target_id() const { return target_id_; }
  static Eigen::Matrix3d original_rotation(const Data_Save &data);
  static cv::Mat to_bgr(const cv::Mat &image, bool input_rgb);
private:
  YAML::Node config_;
  std::unique_ptr<auto_aim::YOLO> yolo_;
  std::unique_ptr<auto_aim::Solver> solver_;
  std::unique_ptr<auto_aim::Tracker> tracker_;
  std::list<auto_aim::Armor> detections_;
  time_point last_timestamp_{};
  int last_mode_ = -1, last_color_ = -1, target_id_ = 0;
};
} // namespace fusion
