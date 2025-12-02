#include "Coordinate.h"
#include "Base.h"
#include "SerialPort.h"

Coordinate_converter C_C;

// 弧度化角度
double R2D(double rad)
{
    return rad * 180 / PI;
}

// 角度化弧度
double D2R(double degree)
{
    return degree * PI / 180;
}
Coordinate_converter::Coordinate_converter(){
    tf_mat.setIdentity();
    camera_gimbal << -0.10, 0.05, 0;
}



// 将装甲板坐标从相对坐标转化为绝对坐标
Eigen::Vector3d Coordinate_converter::cam2abs(Eigen::Vector3d position)
{
    return this->tf_mat * position + camera_gimbal;
}

// 将装甲板坐标从绝对坐标转化为相对坐标
Eigen::Vector3d Coordinate_converter::abs2cam(Eigen::Vector3d position)
{
    return this->tf_mat.inverse() * position - camera_gimbal;
}
 
// 更新此刻的云台位资
void Coordinate_converter::GetTFMat(const double &x, const double &y, const double &z)
{
    // this->tf_mat = GetRotMatXYZ(x,y,z);
    const double cosx = cos(x), sinx = sin(x);
    const double cosy = cos(y), siny = sin(y);
    const double cosz = cos(z), sinz = sin(z);
    
    this->tf_mat << cosy*cosz,           cosz*sinx*siny - cosx*sinz,           cosx*cosz*siny + sinx*sinz,
                    cosy*sinz,           cosx*cosz + sinx*siny*sinz,          -cosz*sinx + cosx*siny*sinz,
                        -siny,                      cosy*sinx,                                  cosx*cosy;
}

Eigen::Vector3d Coordinate_converter::xyz2ypd(const Eigen::Vector3d& xyz)
{
  auto x = xyz.x(), y = xyz.y(), z = xyz.z();
  auto yaw = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}

Eigen::Matrix4d Coordinate_converter::xyz2ypd_jacobian(const Eigen::Vector3d& xyz)
{
  auto x = xyz.x(), y = xyz.y(), z = xyz.z();

  auto dyaw_dx = -y / (x * x + y * y);
  auto dyaw_dy = x / (x * x + y * y);
  auto dyaw_dz = 0.0;

  auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

  auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

  // clang-format off
  Eigen::Matrix4d J{
    {dyaw_dx, dyaw_dy, dyaw_dz, 0},
    {dpitch_dx, dpitch_dy, dpitch_dz, 0},
    {ddistance_dx, ddistance_dy, ddistance_dz, 0},
    {0, 0, 0, 1}
  };
  // clang-format on

  return J;
}

cv::Point3f Coordinate_converter::changeCoor(Eigen::Vector3d point)
{
    Vector3d trans_mat;
    trans_mat<< 0, 0, 0.03;
    Vector3d target_in_base = trans_mat + point;
    return cv::Point3f(target_in_base(0), target_in_base(1), target_in_base(2));
}

Gimbal_Angle Coordinate_converter::GetAngle(Eigen::Vector3d position)
{
    Gimbal_Angle solve_angle;
    cv::Point3f gun_base = changeCoor(position);
    double pitch_in_camera = std::atan2(gun_base.z, std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
    double yaw_in_camera = std::atan2(gun_base.y, gun_base.x);
    solve_angle.target_pitch = pitch_in_camera;
    solve_angle.target_yaw = yaw_in_camera;
    double dis = std::sqrt(gun_base.x * gun_base.x + gun_base.z * gun_base.z);
    double angle_fix = 0;
    if (std::abs(9.8 * dis * std::pow(std::cos(solve_angle.target_pitch), 2)) / std::pow(bullet_speed, 2) - std::sin(solve_angle.target_pitch) <= 1)
    {
        angle_fix = 0.5 * (std::asin((9.8 * dis * std::pow(std::cos(solve_angle.target_pitch), 2)) / std::pow(bullet_speed, 2) - std::sin(solve_angle.target_pitch)) + solve_angle.target_pitch);
    }
    else
    {
        std::cout << "[AngleSolver] Wrong distance in angle_fix" << std::endl;
    }
    return solve_angle;
}

double GetDistance(cv::Point2d p, cv::Point2d q) {
    return std::sqrt(std::pow(p.x - q.x, 2) + std::pow(p.y - q.y, 2));
}

double GetTilt(Vector2d a,Vector2d b){
    return (a(0)-b(0))/(a(1)-b(1));
}

double j(0);
std::vector<cv::Point2f> draw_point2;
std::vector<cv::Point2f> draw_point3;

void Paint::KF_Curve(double real_data, double predict_data)
{

    cv::Mat Paper = cv::Mat(cv::Size(640, 400), CV_8UC3, cv::Scalar(255, 255, 255));

    vec_real.push_back(real_data);
    vec_pridect.push_back(predict_data);
    int once = 1;
    if (vec_real.size() * once > 640)
    {
        vec_real.erase(vec_real.begin());
        vec_pridect.erase(vec_pridect.begin());
    }

    for (size_t i = 0; i < vec_real.size() - 1; i++)
    {
        cv::line(Paper, cv::Point2f(i * once, vec_real[i]+200), 
                cv::Point2f((i + 1) * once, vec_real[i + 1]+200), 
                cv::Scalar(255, 0, 0));
        cv::line(Paper, cv::Point2f(i * once, vec_pridect[i]+200), 
                cv::Point2f((i + 1) * once, vec_pridect[i + 1]+200), 
                cv::Scalar(120, 5, 127));
    }

    cv::putText(Paper, "observe :", cv::Point2f(500, 15), 1.25, 1, cv::Scalar(0, 0, 0));
    cv::line(Paper, cv::Point2f(610, 12), cv::Point2f(620, 12), cv::Scalar(255, 0, 0));
    cv::putText(Paper, "predict :", cv::Point2f(500, 30), 1.25, 1, cv::Scalar(0, 0, 0));
    cv::line(Paper, cv::Point2f(610, 30), cv::Point2f(620, 30), cv::Scalar(0, 0, 255));

    cv::imshow("KF", Paper);
    waitKey(1);
}





void Paint::Speed_Curve(double real_data)
{

    cv::Mat Paper = cv::Mat(cv::Size(640, 400), CV_8UC3, cv::Scalar(255, 255, 255));

    vec_speed.push_back(real_data);
    int once = 2;
    if (vec_speed.size() * once > 640)
    {
        vec_speed.erase(vec_speed.begin());
    }

    for (size_t i = 0; i < vec_speed.size() - 1; i++)
    {
        cv::line(Paper, cv::Point2f(i * once, vec_speed[i] +200), 
                cv::Point2f((i + 1) * once, vec_speed[i + 1] +200), 
                cv::Scalar(255, 0, 0));
        
    }

    cv::imshow("Speed", Paper);
    waitKey(1);
}