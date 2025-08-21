#include "camera_to_imu.h"
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
}

// 将装甲板坐标从相对坐标转化为绝对坐标
cv::Point3d Coordinate_converter::cam2abs(cv::Point3d position)
{
    Vector4d target(position.x, position.y, position.z, 1.0);
    Vector4d abs_target = this->tf_mat * target;
    return cv::Point3d(abs_target(0), abs_target(1), abs_target(2));
}

// 将装甲板坐标从绝对坐标转化为相对坐标
cv::Point3d Coordinate_converter::abs2cam(cv::Point3d position)
{
    Vector4d target(position.x, position.y, position.z, 1.0);
    Vector4d cam_target = this->tf_mat.inverse() * target;
    return cv::Point3d(cam_target(0), cam_target(1), cam_target(2));
}
 
// 更新此刻的云台位资
void Coordinate_converter::GetTFMat(double x,double y,double z){
    Matrix4d rot_mat = GetRotMatXYZ(x,y,z);
    Matrix4d trans_mat = GetTransMat(-0.10, 0.05, 0);
    this->tf_mat = rot_mat * trans_mat;
}
    
// 三分法工具函数
Matrix4d Coordinate_converter::GetTrichMat(double yaw,Point3d point){
    Matrix4d TrichMat;
    double cosy = cos(PI/12);
    double siny = sin(PI/12);
    double cosz = cos(yaw);
    double sinz = -sin(yaw);
    TrichMat << cosz*cosy,sinz,-cosz*siny,  point.x,
               -sinz*cosy,cosz, sinz*siny,  point.y,
                     siny,   0,      cosy,  point.z,
                        0,   0,         0,        1;
    return TrichMat;
}

Matrix4d GetRotMatX(double x)
{
    Matrix4d MatX;
    MatX << 1,      0,       0, 0,
            0, cos(x), -sin(x), 0,
            0, sin(x),  cos(x), 0,
            0,      0,       0, 1;
    return MatX;
}

Matrix4d GetRotMatY(double y)
{
    Matrix4d MatY;
    MatY << cos(y), 0, sin(y), 0,
                 0, 1,      0, 0,
           -sin(y), 0, cos(y), 0,
                 0, 0,      0, 1;
    return MatY;
}

Matrix4d GetRotMatZ(double z)
{
    Matrix4d MatZ;
    MatZ << cos(z), -sin(z), 0, 0,
            sin(z),  cos(z), 0, 0,
                0,       0,  1, 0,
                0,       0,  0, 1;
    return MatZ;
}

Matrix4d GetTransMat(double x, double y, double z)
{
    Matrix4d Mat;
    Mat << 1, 0, 0, x,
           0, 1, 0, y,
           0, 0, 1, z,
           0, 0, 0, 1;
    return Mat;
}

Matrix4d GetRotMatXYZ(double x, double y, double z)
{
    return GetRotMatZ(z) * GetRotMatY(y) * GetRotMatX(x);
}

cv::Point3f Coordinate_converter::changeCoor(cv::Point3f point)
{
    Matrix4d trans_mat = GetTransMat(0, 0, 0.03);
    Vector4d target(point.x, point.y, point.z, 1.0);
    Vector4d target_in_base = trans_mat * target;
    return cv::Point3f(target_in_base(0), target_in_base(1), target_in_base(2));
}

Gimbal_Angle Coordinate_converter::GetAngle(cv::Point3f position)
{
    Gimbal_Angle solve_angle;
    cv::Point3f gun_base = changeCoor(position);
    double pitch_in_camera = std::atan2(gun_base.z, std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
    double yaw_in_camera = std::atan2(gun_base.y, gun_base.x);
    std::unique_lock<std::mutex> lock(ds_mutex);  
    solve_angle.pitch = pitch_in_camera + ds.camera_pit_angle;
    solve_angle.yaw = yaw_in_camera + ds.camera_yaw_angle;
    lock.unlock();
    double dis = std::sqrt(gun_base.x * gun_base.x + gun_base.z * gun_base.z);
    double angle_fix = 0;
    if (std::abs(9.8 * dis * std::pow(std::cos(solve_angle.pitch), 2)) / std::pow(bullet_speed, 2) - std::sin(solve_angle.pitch) <= 1)
    {
        angle_fix = 0.5 * (std::asin((9.8 * dis * std::pow(std::cos(solve_angle.pitch), 2)) / std::pow(bullet_speed, 2) - std::sin(solve_angle.pitch)) + solve_angle.pitch);
    }
    else
    {
        std::cout << "[AngleSolver] Wrong distance in angle_fix" << std::endl;
    }
    return solve_angle;
}

double GetDistance(cv::Point3d p)
{
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
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