
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <find_armor.h>
#include "Base.h"
#include "camera_to_imu.h"
#include "BA_Optimizer.h"
#include <ceres/ceres.h>
using namespace cv;
class YawPnP {
public:
    YawPnP() {}
    void setWorldPoints(const std::vector<cv::Point3f>& object_points);
    void setImagePoints(const std::vector<cv::Point2f>& image_points);
    std::vector<Eigen::Vector4d> getMapping(double append_yaw) const;
    std::vector<Eigen::Vector2d> getProject(const std::vector<Eigen::Vector4d>& P_world) const;
    void ConnectPoints(Armor armor,double yaw, Mat& img);
    double GetError(double angle);
    bool GetScope();
    double TriangleArea(const Eigen::Vector2d& p1, 
                           const Eigen::Vector2d& p2, 
                           const Eigen::Vector2d& p3);
    // 成员变量
    double sys_yaw;
    Point3d pose;
    std::vector<Eigen::Vector2d> P_pixel;
    std::vector<Eigen::Vector4d> P_world;
    Eigen::Matrix3d Kc;
    Eigen::Matrix4d T;
    Eigen::Matrix4d T_inv;
};

Eigen::Matrix<double, 4, 4> getTransformationMatrix(double x, double y, double z, const Eigen::Vector3d& translation);
void tf_Vec4d(const cv::Mat& input_mat, Eigen::Matrix<double, 4, 1>& output_eigen);
void tf_Mat3f(const cv::Mat& input_mat, Eigen::Matrix<double, 3, 3>& output_eigen);
double computeVariance(const std::vector<double>& diffs);//计算方差
void solveYawPnP(Armor &armor,Mat &img,Gimbal_Angle &initial_abs); //三分法接口