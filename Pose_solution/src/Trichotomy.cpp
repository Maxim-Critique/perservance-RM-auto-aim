#include "Trichotomy.h"
#include <cmath>
#include "SerialPort.h"
#include "Angel_Calculate.h"
#include <numeric>
#include <atomic>
#include <future>
#include <cmath>
#include <algorithm>

using namespace std;

// 全局变量声明
namespace {
    std::atomic<double> text_yaw(0.0);
    int trackbar_value = 50;  // 滑动条当前值（0-100）
    bool control_window_initialized = false;
}

constexpr double GOLDEN_RATIO = 0.618033988749895;

// 滑动条回调函数
void onTrackbar(int value, void* userdata) {
    const double min_val = -5.0 * CV_PI / 12.0;
    const double max_val =  5.0 * CV_PI / 12.0;
    // 映射：0->min_val, 100->max_val
    text_yaw.store(min_val + (max_val - min_val) * (value / 100.0));
    trackbar_value = value;
}

// 初始化控制窗口（线程安全）
void initControlWindow() {
    static std::mutex init_mutex;
    std::lock_guard<std::mutex> lock(init_mutex);
    
    if (!control_window_initialized) {
        cv::namedWindow("Yaw Control", cv::WINDOW_NORMAL);
        cv::resizeWindow("Yaw Control", 500, 100);
        
        // 创建滑动条（使用NULL作为值指针）
        cv::createTrackbar("Yaw Adjust", "Yaw Control", 
                          nullptr, 100, onTrackbar);
        
        // 设置初始位置
        cv::setTrackbarPos("Yaw Adjust", "Yaw Control", trackbar_value);
        onTrackbar(trackbar_value, nullptr);
        
        control_window_initialized = true;
    }
}

double GetYawAngle(YawPnP& yaw_pnp, double sys_yaw, double angle) {

    // 搜索范围
    double min_yaw = -5.0 * PI / 12.0 + sys_yaw + PI;
    double max_yaw = 5.0 * PI / 12.0 + sys_yaw + PI;
    
    double left = min_yaw;
    double right = max_yaw;
    while(true){
        if(abs(left-right)<D2R(4)){
            break;
        }
        double mid1 = left + (right - left) / 3.0;
        double mid2 = right - (right - left) / 3.0;
        double error1 = yaw_pnp.GetError(mid1);
        double error2 = yaw_pnp.GetError(mid2);

        // cout << " mid1 " <<R2D(mid1)<<" mid2 " <<R2D(mid2)<<" error1 "<<error1<<" error2 "<<error2<<endl;
        if (error1 < error2) {
            right = mid2;
        } else {
            left = mid1;
        }
    }
    double a = (left+right)/2 - PI - sys_yaw;
    double b = -a + sys_yaw + PI;
    // cout<<yaw_pnp.GetError((left+right)/2)<<"  "<<yaw_pnp.GetError(b)<<endl;
    if(yaw_pnp.GetError((left+right)/2)<yaw_pnp.GetError(b)){
        
        return (left+right)/2;
    }else{
        return b;
    }
    // if(yaw_pnp.GetError(angle+PI+sys_yaw)<yaw_pnp.GetError(-angle+PI+sys_yaw)){
    //     cout<<yaw_pnp.GetError(angle+PI+sys_yaw)<<"  "<<yaw_pnp.GetError(-angle+PI+sys_yaw)<<endl;
    //     return angle+PI+sys_yaw;
    // }else{
    //     cout<<yaw_pnp.GetError(angle+PI+sys_yaw)<<"  "<<yaw_pnp.GetError(-angle+PI+sys_yaw)<<endl;
    //     return -angle+PI+sys_yaw;
    // }
}

// 三分法接口        
void solveYawPnP(Armor &armor, Mat &img, Gimbal_Angle &initial_abs) {
    pnp_armor.readConfig(armor);
    YawPnP* yaw_pnp = new YawPnP();
    std::unique_lock<std::mutex> lock(ds_mutex);  
    yaw_pnp->sys_yaw = ds.camera_yaw_angle;
    lock.unlock();
    yaw_pnp->setImagePoints(armor.points);
    yaw_pnp->setWorldPoints(pnp_armor.world_points);
    
    // 初始PNP解算
    cv::Mat rvec, tvec, m_Rvec;
    cv::solvePnP(pnp_armor.world_points, armor.points, 
                pnp_armor.Intrinsic_Matrix, pnp_armor.Distortion, 
                rvec, tvec, false, cv::SOLVEPNP_IPPE);
    
    // 获取初始位姿
    armor.cam_pose = {*tvec.ptr<double>(2), -*tvec.ptr<double>(0), -*tvec.ptr<double>(1)};
    armor.abs_pose = C_C.cam2abs(armor.cam_pose);
    initial_abs = C_C.GetAngle(armor.cam_pose);
    
    // 获取初始Yaw角
    cv::Rodrigues(rvec, m_Rvec);
    double Z = asin(-1 * m_Rvec.at<double>(1, 2));
    double angle = asin(m_Rvec.at<double>(1, 3) / cos(Z));
    yaw_pnp->pose = armor.abs_pose;
    tf_Mat3f(pnp_armor.Intrinsic_Matrix, yaw_pnp->Kc);
    // 使用优化后的yaw角
    armor.yaw = GetYawAngle(*yaw_pnp, yaw_pnp->sys_yaw, angle);
    


    YAML::Node config = YAML::LoadFile(Yaml);
    int show = config["CAMERA"]["show"].as<int>();
    
    // 可视化
    if(show){
        // initControlWindow();
        // double current_yaw = text_yaw.load() + yaw_pnp->sys_yaw + PI;
        double current_yaw = armor.yaw;
        yaw_pnp->ConnectPoints(armor, current_yaw, img);
    }
    delete yaw_pnp;
}

double YawPnP::GetError(double angle){
    auto P_mapping = getMapping(angle);
    auto P_project = getProject(P_mapping);
    double error = 0;
    for (int i=0; i<4 ; ++i){
        error += (P_project[i][0]-P_pixel[i][0])*(P_project[i][0]-P_pixel[i][0]) + (P_project[i][1]-P_pixel[i][1])*(P_project[i][1]-P_pixel[i][1]);
    }
    return error;
    // auto P_mapping = getMapping(angle);
    // auto P_project = getProject(P_mapping);

    // // 海伦公式误差计算
    // double helen_err = 0.0;
    // for (int i = 0; i < 4; i++) {
    //     // 计算重投影三角形与检测点三角形的面积差
    //     helen_err += TriangleArea(
    //         P_project[i], 
    //         P_project[(i + 1) % 4], 
    //         P_pixel[i]
    //     );
    //     helen_err += TriangleArea(
    //         P_pixel[i], 
    //         P_pixel[(i + 1) % 4], 
    //         P_project[i]
    //     );
    // }

    // // 可选：根据装甲板长宽比调整权重
    // double width = (P_pixel[0] - P_pixel[1]).norm() + (P_pixel[2] - P_pixel[3]).norm();
    // double height = (P_pixel[1] - P_pixel[2]).norm() + (P_pixel[3] - P_pixel[0]).norm();
    // double ratio = height / width; // 长宽比权重

    // return helen_err * ratio;
}

bool YawPnP::GetScope(){
    double a =  (P_pixel[0][1]-P_pixel[1][1]) / (P_pixel[0][0]-P_pixel[1][0]);
    double b =  (P_pixel[2][1]-P_pixel[3][1]) / (P_pixel[2][0]-P_pixel[3][0]);
    cout<<a<<"   "<<b<<endl;
    return (a+b)>0;
}

//绘制重投影图像
void YawPnP::ConnectPoints(Armor armor, double yaw, Mat& img){
    auto P_mapping = getMapping(yaw);
    auto P_project = getProject(P_mapping); 
    for(int i = 0; i < 4; i++) {
        // Draw connecting lines
        cv::line(img, 
                 cv::Point(P_project[i](0), P_project[i](1)),
                 cv::Point(P_project[(i+1)%4](0), P_project[(i+1)%4](1)),
                 cv::Scalar(125, 0, 125), 2);
        cv::line(img, 
                cv::Point(P_pixel[i](0), P_pixel[i](1)),
                cv::Point(P_pixel[(i+1)%4](0), P_pixel[(i+1)%4](1)),
                cv::Scalar(0, 255, 0), 2);
    }
    putText(img,"Dis: "+to_string(armor.cam_pose.x)+"Angle :"+to_string(R2D(_std_radian(yaw-PI-sys_yaw)))+
        "Error: "+to_string(GetError(yaw)),armor.points[1], FONT_HERSHEY_COMPLEX, 0.6, Scalar(205, 100, 225), 1);
    
    putText(img, "BEFORE_EKF: " + to_string(armor.cam_pose.x) + "     " + to_string(armor.cam_pose.y) + "    " + to_string(armor.cam_pose.z), 
        Point(50, 700), FONT_HERSHEY_COMPLEX, 0.8, Scalar(115, 0, 125), 1);
}

double computeVariance(const std::vector<double>& diffs) {
    if (diffs.empty()) {
        return 0.0;
    }
    double mean = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
    double variance = 0.0;
    for (double val : diffs) {
        variance += (val - mean) * (val - mean);
    }
    variance /= diffs.size();
    return variance;
}

void tf_Mat3f(const cv::Mat& input_mat, Eigen::Matrix<double, 3, 3>& output_eigen) {
    output_eigen << input_mat.at<float>(0, 0), input_mat.at<float>(0, 1), input_mat.at<float>(0, 2), 
                    input_mat.at<float>(1, 0), input_mat.at<float>(1, 1), input_mat.at<float>(1, 2), 
                    input_mat.at<float>(2, 0), input_mat.at<float>(2, 1), input_mat.at<float>(2, 2);
}

void YawPnP::setWorldPoints(const std::vector<cv::Point3f>& object_points) { 
    for (const auto& p : object_points) {
        P_world.push_back(Eigen::Vector4d(0, p.x, p.y, 1));
    }
}

void YawPnP::setImagePoints(const std::vector<cv::Point2f>& image_points) { 
    for (const auto& p : image_points) {
        P_pixel.push_back(Eigen::Vector2d(p.x, p.y));
    }
}

std::vector<Eigen::Vector4d> YawPnP::getMapping(double append_yaw) const {
    Eigen::Matrix4d M;
    std::vector<Eigen::Vector4d> P_mapping;

    M = C_C.GetTrichMat(append_yaw,pose);
    for (const auto& p : P_world) {
        P_mapping.push_back(M * p);
    }
    return P_mapping;
}

std::vector<Eigen::Vector2d> YawPnP::getProject(const std::vector<Eigen::Vector4d>& P_world) const {
    std::vector<Eigen::Vector2d> P_project;
    for (const auto& p : P_world) {
        Eigen::Vector3d p_camera = (C_C.tf_mat.inverse()* p).head(3);
        Eigen::Vector3d p_pnp;
        p_pnp << -p_camera.y(), -p_camera.z(),p_camera.x();
        Eigen::Vector3d p_project = Kc * p_pnp;
        P_project.push_back(p_project.head(2) / p_pnp(2));
    }
    return P_project;
}

// 计算三角形面积（海伦公式）
double YawPnP::TriangleArea(const Eigen::Vector2d& p1, 
                           const Eigen::Vector2d& p2, 
                           const Eigen::Vector2d& p3){
    double a = (p2 - p1).norm();  // 边长 p1-p2
    double b = (p3 - p2).norm();  // 边长 p2-p3
    double c = (p1 - p3).norm();  // 边长 p3-p1
    double p = (a + b + c) / 2.0; // 半周长
    return sqrt(p * (p - a) * (p - b) * (p - c)); // 海伦公式
}