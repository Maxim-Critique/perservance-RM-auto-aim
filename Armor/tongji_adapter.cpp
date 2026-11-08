#include "tongji_adapter.hpp"
#include <filesystem>
#include <cmath>
#include <stdexcept>
#include <limits>

namespace fusion {
TongjiModel::TongjiModel(const auto_aim::Target &target) : state_(target.ekf_x()) {
  number = static_cast<int>(target.armor_xyza_list().size());
  center = {state_[0], state_[2]}; velocity = {state_[1], state_[3]};
  phase = state_[6]; palstance = state_[7];
  radius = {state_[8], state_[8] + state_[9]};
  height = {state_[4], state_[4] + state_[10]};
}
Armors TongjiModel::GetArmors(double dt) {
  Armors result(number);
  for (int i = 0; i < number; ++i) {
    const bool alternating = number == 4 && i % 2;
    const double a = state_[6] + state_[7] * dt + i * 2 * CV_PI / number;
    const double r = state_[8] + (alternating ? state_[9] : 0);
    result[i].abs_pose = {state_[0] + state_[1] * dt - r * std::cos(a),
                          state_[2] + state_[3] * dt - r * std::sin(a),
                          state_[4] + state_[5] * dt + (alternating ? state_[10] : 0)};
    result[i].predict_yaw = a;
  }
  return result;
}
Eigen::Matrix3d TongjiAdapter::original_rotation(const Data_Save &d) {
  // 与 A::Slover::UpdateStatus 相同：Rz(yaw) Ry(-pitch) Rx(roll)。
  return (Eigen::AngleAxisd(d.camera_yaw_angle, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(-d.camera_pitch_angle, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(d.camera_roll_angle, Eigen::Vector3d::UnitX())).toRotationMatrix();
}
cv::Mat TongjiAdapter::to_bgr(const cv::Mat &image, bool rgb) {
  if (image.empty()) return {};
  if (image.type() != CV_8UC3) throw std::invalid_argument("Expected CV_8UC3 image");
  cv::Mat result;
  if (rgb) cv::cvtColor(image, result, cv::COLOR_RGB2BGR);
  else result = image;
  return result;
}
TongjiAdapter::TongjiAdapter(const std::string &car_path, const std::string &algorithm_path,
                           bool load_model) {
  const auto car = YAML::LoadFile(car_path);
  config_ = YAML::LoadFile(algorithm_path);
  config_["camera_matrix"] = car["Camera"]["camera_matrix"];
  config_["distort_coeffs"] = car["Camera"]["distort_coeffs"];
  config_["R_camera2gimbal"] = car["Transform"]["R_camera2gimbal"];
  config_["t_camera2gimbal"] = car["Transform"]["t_camera2gimbal"];
  for (auto key : {"camera_matrix", "R_camera2gimbal"})
    if (config_[key].size() != 9) throw std::invalid_argument(key);
  if (config_["distort_coeffs"].size() != 5 || config_["t_camera2gimbal"].size() != 3)
    throw std::invalid_argument("Invalid original-car calibration vector length");
  const double margin = config_["type_min_error_margin_px"].as<double>();
  const double ratio = config_["type_max_error_ratio"].as<double>();
  if (config_["inference_threads"].as<int>() <= 0 || !std::isfinite(margin) || margin <= 0 || !std::isfinite(ratio) || ratio <= 0 || ratio >= 1)
    throw std::invalid_argument("Invalid armor type hypothesis thresholds");
  for (auto key : {"initial_radius_m", "outpost_initial_radius_m"}) {
    const double radius = config_[key].as<double>();
    if (!std::isfinite(radius) || radius <= 0.05 || radius >= 0.5)
      throw std::invalid_argument("Radius outside Tongji Target model bounds");
  }
  for (auto key : {"camera_matrix", "distort_coeffs", "R_camera2gimbal", "t_camera2gimbal"})
    for (double value : config_[key].as<std::vector<double>>())
      if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite original calibration");
  const auto matrix = config_["camera_matrix"].as<std::vector<double>>();
  const auto rotation = config_["R_camera2gimbal"].as<std::vector<double>>();
  const Eigen::Matrix<double,3,3,Eigen::RowMajor> R(rotation.data());
  if (matrix[0] <= 0 || matrix[4] <= 0 || (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()>1e-5 ||
      std::abs(R.determinant()-1)>1e-5) throw std::invalid_argument("Invalid original calibration matrix");
  for (auto key : {"yolo11_model_path", "yolov8_model_path", "yolov5_model_path", "classify_model"}) {
    std::filesystem::path path(config_[key].as<std::string>());
    if (path.is_relative()) path = std::filesystem::absolute(std::filesystem::path(algorithm_path).parent_path() / path);
    config_[key] = path.string();
    if (load_model && !std::filesystem::is_regular_file(path))
      throw std::runtime_error("Missing model: " + path.string());
  }
  solver_ = std::make_unique<auto_aim::Solver>(config_);
  if (load_model) yolo_ = std::make_unique<auto_aim::YOLO>(config_, false);
}
void TongjiAdapter::reset() {
  tracker_.reset(); detections_.clear(); last_timestamp_ = {}; last_mode_ = -1;
  last_color_ = -1; target_id_ = 0;
}
bool TongjiAdapter::accept_type(auto_aim::Armor &a, int mode) const {
  if (a.name < auto_aim::one || a.name >= auto_aim::not_armor ||
      (a.type != auto_aim::big && a.type != auto_aim::small)) return false;
  // C's 38-class table labels hero as small; A rejects SMALL/1, not number 1 itself.
  if (a.name == auto_aim::one) a.type = auto_aim::big;
  if (mode == 3) return a.name == auto_aim::outpost && a.type == auto_aim::small;
  if (mode != 0 || a.name == auto_aim::outpost || a.name == auto_aim::base) return false;
  // Preserve A's actual non-SENTRY compilation rules. Guard is permitted only SMALL.
  if (a.type == auto_aim::big &&
      (a.name == auto_aim::sentry || a.name == auto_aim::three || a.name == auto_aim::four)) return false;
  return true;
}
bool TongjiAdapter::resolve_type(auto_aim::Armor &armor, int mode) const {
  if (armor.points.size() != 4) return false;
  // 以 YOLO 语义为先验；在原车姿态和实际尺寸下分别检验两种模型。
  // 不使用图像宽高比阈值：俯视引起的投影变化通过相机姿态进入重投影。
  auto score = [&](auto_aim::Armor &candidate) {
    try {
      solver_->solve(candidate);
      const auto points = solver_->reproject_armor(candidate.xyz_in_world,
        candidate.ypr_in_world[0], candidate.type, candidate.name);
      double sum = 0;
      for (size_t i = 0; i < 4; ++i) sum += cv::norm(points[i] - candidate.points[i]);
      return sum / 4;
    } catch (const std::exception &) { return std::numeric_limits<double>::infinity(); }
  };
  auto prior = armor;
  if (prior.name == auto_aim::one) prior.type = auto_aim::big;
  const double prior_error = score(prior);
  if (prior.name == auto_aim::one) {
    if (!std::isfinite(prior_error) || !accept_type(prior, mode)) return false;
    armor = prior; return true;
  }
  auto alternate = armor;
  alternate.type = prior.type == auto_aim::small ? auto_aim::big : auto_aim::small;
  const double alternate_error = score(alternate);
  const double margin = config_["type_min_error_margin_px"].as<double>();
  const double ratio = config_["type_max_error_ratio"].as<double>();
  // 只在另一个合法类型明显更符合姿态假设时覆盖先验；模棱两可保留先验。
  if (accept_type(alternate, mode) && std::isfinite(alternate_error) && alternate_error + margin < prior_error &&
      alternate_error < ratio * prior_error) armor = alternate;
  else if (accept_type(prior, mode) && std::isfinite(prior_error)) armor = prior;
  else return false;
  return true;
}
std::list<auto_aim::Armor> TongjiAdapter::detect_only(const cv::Mat &image, bool rgb) {
  if (image.empty()) return {};
  if (!yolo_) throw std::logic_error("YOLO model not loaded");
  return yolo_->detect(to_bgr(image, rgb));
}
std::shared_ptr<KinematicModel> TongjiAdapter::process(const cv::Mat &image, const Data_Save &d,
                                                     bool rgb) {
  if (image.empty()) return update({}, d, {1, 1});
  if (!yolo_) throw std::logic_error("YOLO model not loaded");
  return update(yolo_->detect(to_bgr(image, rgb)), d, image.size());
}
std::shared_ptr<KinematicModel> TongjiAdapter::update(std::list<auto_aim::Armor> armors,
                                                   const Data_Save &d, cv::Size size) {
  if ((d.armor_or_buff != 0 && d.armor_or_buff != 3) ||
      (d.color != 0 && d.color != 1) || !std::isfinite(d.camera_yaw_angle) ||
      !std::isfinite(d.camera_pitch_angle) || !std::isfinite(d.camera_roll_angle) ||
      d.timestamp.time_since_epoch().count() <= 0 || size.width <= 0 || size.height <= 0) {
    reset(); return {};
  }
  const double dt = std::chrono::duration<double>(d.timestamp - last_timestamp_).count();
  if (last_mode_ != static_cast<int>(d.armor_or_buff) || last_color_ != d.color ||
      (last_timestamp_ != time_point{} && (dt <= 0 || dt > 0.1))) reset();
  if (!tracker_) {
    config_["enemy_color"] = d.color == 0 ? "blue" : "red";
    tracker_ = std::make_unique<auto_aim::Tracker>(config_, *solver_);
  }
  last_timestamp_ = d.timestamp; last_mode_ = static_cast<int>(d.armor_or_buff); last_color_ = d.color;
  tracker_->set_image_center({size.width / 2.f, size.height / 2.f});
  solver_->set_original_car_rotation(original_rotation(d));
  for (auto it = armors.begin(); it != armors.end();) {
    bool valid = it->points.size() == 4 &&
                 std::isfinite(it->confidence) && it->confidence >= config_["min_confidence"].as<double>();
    for (const auto &p : it->points) valid = valid && std::isfinite(p.x) && std::isfinite(p.y);
    if (valid) valid = cv::isContourConvex(it->points) && std::abs(cv::contourArea(it->points)) > 1;
    if (valid) {
      try {
        valid = resolve_type(*it, last_mode_) && it->xyz_in_world.allFinite() && it->ypr_in_world.allFinite() &&
                it->xyz_in_gimbal.norm() > 0.1 && it->xyz_in_gimbal.norm() < 12;
      } catch (const cv::Exception &) { valid = false; }
        catch (const std::runtime_error &) { valid = false; }
    }
    if (!valid) it = armors.erase(it);
    else { it->priority = auto_aim::first; ++it; } // A has no Tongji infantry exclusion policy.
  }
  auto targets = tracker_->track(armors, d.timestamp);
  detections_ = std::move(armors);
  if (targets.empty() || tracker_->state() != "tracking") return {};
  auto &target = targets.front();
  const auto x = target.ekf_x();
  if (x.size() != 11 || !x.allFinite() || target.diverged()) return {};
  if (last_mode_ == 3 && (std::abs(x[7]) < 2 || std::abs(x[7]) > 3)) return {};
  target_id_ = target.name <= auto_aim::five ? static_cast<int>(target.name) + 1 :
               target.name == auto_aim::sentry ? 7 : target.name == auto_aim::outpost ? 6 : 8;
  return std::make_shared<TongjiModel>(target);
}
} // namespace fusion
