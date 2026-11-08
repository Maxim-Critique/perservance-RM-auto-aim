#include "solver.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 50e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 130e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

struct ArmorReprojectionCost
{
  ArmorReprojectionCost(
    const Eigen::Vector3d & world_point, const cv::Point2d & observed_pixel,
    const Eigen::Matrix3d & R_gimbal2world, const Eigen::Matrix3d & R_camera2gimbal,
    const Eigen::Vector3d & t_camera2gimbal, const cv::Mat & camera_matrix, double pitch_rad)
  : point_in_armor_(world_point),
    pixel_(observed_pixel),
    R_g2w_(R_gimbal2world),
    R_c2g_(R_camera2gimbal),
    t_c2g_(t_camera2gimbal),
    K_(camera_matrix),
    pitch_(pitch_rad)
  {
  }

  template <typename T>
  bool operator()(const T * const yaw_ptr, const T * const pos_ptr, T * residuals) const
  {
    T yaw = yaw_ptr[0];

    T cos_yaw = ceres::cos(yaw);
    T sin_yaw = ceres::sin(yaw);
    T cos_pitch = ceres::cos(T(pitch_));
    T sin_pitch = ceres::sin(T(pitch_));

    // 构造 R_armor2world
    Eigen::Matrix<T, 3, 3> R_aw;
    R_aw << cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch, sin_yaw * cos_pitch, cos_yaw,
      sin_yaw * sin_pitch, -sin_pitch, T(0), cos_pitch;

    // 构造 R_armor2camera 和 t_armor2camera
    Eigen::Matrix<T, 3, 3> R_g2w = R_g2w_.cast<T>();
    Eigen::Matrix<T, 3, 3> R_c2g = R_c2g_.cast<T>();
    Eigen::Matrix<T, 3, 3> R_ac = R_c2g.transpose() * R_g2w.transpose() * R_aw;
    Eigen::Matrix<T, 3, 1> t_aw(pos_ptr[0], pos_ptr[1], pos_ptr[2]);
    Eigen::Matrix<T, 3, 1> t_cg = t_c2g_.cast<T>();
    Eigen::Matrix<T, 3, 1> t_ac = R_c2g.transpose() * (R_g2w.transpose() * t_aw - t_cg);

    // 投影点
    Eigen::Matrix<T, 3, 1> p_cam = R_ac * point_in_armor_.cast<T>() + t_ac;

    T x = p_cam(0) / p_cam(2);
    T y = p_cam(1) / p_cam(2);

    T fx = T(K_.at<double>(0, 0));
    T fy = T(K_.at<double>(1, 1));
    T cx = T(K_.at<double>(0, 2));
    T cy = T(K_.at<double>(1, 2));

    T u = fx * x + cx;
    T v = fy * y + cy;

    residuals[0] = u - T(pixel_.x);
    residuals[1] = v - T(pixel_.y);
    return true;
  }

  static ceres::CostFunction * Create(
    const Eigen::Vector3d & world_point, const cv::Point2d & pixel, const Eigen::Matrix3d & R_g2w,
    const Eigen::Matrix3d & R_c2g, const Eigen::Vector3d & t_c2g, const cv::Mat & K, double pitch)
  {
    return (new ceres::AutoDiffCostFunction<ArmorReprojectionCost, 2, 1, 3>(
      new ArmorReprojectionCost(world_point, pixel, R_g2w, R_c2g, t_c2g, K, pitch)));
  }

private:
  const Eigen::Vector3d point_in_armor_;
  const cv::Point2d pixel_;
  const Eigen::Matrix3d R_g2w_;
  const Eigen::Matrix3d R_c2g_;
  const Eigen::Vector3d t_c2g_;
  const cv::Mat & K_;
  const double pitch_;
};

Solver::Solver(const YAML::Node & config) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = config;

  R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

//solvePnP（获得姿态）
void Solver::solve(Armor & armor) const
{
  // iterate ekfpnp
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  if (armor.points.size() != 4 || !cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE) || !cv::checkRange(cv::Mat(tvec)) || tvec[2] <= 0)
    throw std::runtime_error("Invalid Tongji PnP solution");

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tongji_tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tongji_tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tongji_tools::xyz2ypd(armor.xyz_in_world);

  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  const bool lsqr = false;  // upstream Ceres residual is incompatible; see migration report
  if (!lsqr) {
    tongji_tools::logger()->info("failed to optimize by least squares within 2ms");
    optimize_yaw(armor);
  }
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tongji_tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tongji_tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tongji_tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

bool Solver::optimize_by_least_squares(Armor & armor) const
{
  // 初始值
  double yaw = armor.ypr_in_world[0];
  double pos[3] = {armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2]};

  // pitch angle
  double pitch = (armor.name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;

  // 构建 Ceres 问题
  ceres::Problem problem;

  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  for (int i = 0; i < 4; ++i) {
    ceres::CostFunction * cost_function = ArmorReprojectionCost::Create(
      Eigen::Vector3d(object_points[i].x, object_points[i].y, 0),
      armor.points[i],  // 实际观测的图像坐标
      R_gimbal2world_, R_camera2gimbal_, t_camera2gimbal_, camera_matrix_, pitch);

    problem.AddResidualBlock(cost_function, nullptr, &yaw, pos);
  }

  // 选项与求解
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  // options.num_threads = 4;                     // 设置线程数
  options.max_solver_time_in_seconds = 0.002;  //最大求解时间2ms

  // 执行求解
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 判断收敛
  if (summary.termination_type == ceres::CONVERGENCE) {
    armor.ypr_in_world[0] = yaw;
    armor.xyz_in_world = Eigen::Vector3d(pos[0], pos[1], pos[2]);
    return true;
  }

  return false;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tongji_tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tongji_tools::square(pixel_dis * std::sin(inclined)) +
      tongji_tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim