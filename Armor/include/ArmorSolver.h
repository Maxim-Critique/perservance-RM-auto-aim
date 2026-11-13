#ifndef ARMOR_SOLVER_H
#define ARMOR_SOLVER_H

#pragma once
#include <queue>
#include "Base.h"
#include "SerialPort.h"
#include <fstream>
#include "ArmorTypes.h"
#include <yaml-cpp/yaml.h>
using namespace std;
using namespace cv;

/**
 * @file ArmorSolver.h
 * @brief 当前 AutoArmor 仍使用的装甲板 PnP、姿态补偿和板朝向搜索接口。
 * 像素点来自 ArmorDetector，长度为 m，内部角度为 rad，不在此处执行串口缩放。
 * 已确认调用链为 AutoArmor -> UpdateStatus/Solve -> observation_from_a -> Tracker；
 * 其位置和板朝向输出是现行十一维追踪模型的观测来源。
 */

/**
 * @brief 将四个图像角点解算成板中心位置，并用重投影搜索板相对朝向。
 *
 * 坐标链：OpenCV 相机系点 -> 配置外参指定的云台系点 -> 云台姿态旋转后的世界参考系点。
 * OpenCV 相机系采用 x 向图像右、y 向图像下、z 沿光轴向前；云台轴向由外参定义。
 * abs_pose 的“世界系”只施加 R_gimbal2world_，未叠加导航/底盘的世界平移，
 * 因而不能直接当成导航 map 系目标坐标。外参、反馈正负方向和零点须由整机统一。
 * 名称 Slover 为原工程拼写，在注释补充时保留原接口。
 */
class Slover
{
public:
    /**
     * @brief 读取 Camera 相机内参/五项畸变和 Transform 相机到云台外参，建立大小板物点。
     * @param config_path YAML 路径，默认 Base.h 的 YamlPath。
     * 当前板模型采用宽 0.23/0.13 m、高 0.05 m 的四个平面点；这是源码模型尺寸，
     * 不是在此确认实物标定正确。缺失配置或类型错误可能向外抛出异常。
     */
    explicit Slover(const std::string & config_path = YamlPath);
    /**
     * @brief 根据本帧 yaw/pitch/roll 更新姿态旋转矩阵及 sys_yaw。
     * @param ds 图像对应时刻的云台反馈，角度已换算为 rad。
     * 主位置变换按 Rz(yaw)*Ry(-pitch)*Rx(roll) 组合，另建立重投影用的 roll/pitch 补偿矩阵。
     * 本接口不做时间同步；调用方必须在 Solve 前提供与图像对应的反馈。
     */
    void UpdateStatus(Data_Save ds);
    /**
     * @brief 对一块已检测装甲板做 PnP 和相对 yaw 搜索，原地补全解算字段。
     * @param armor 输入有效 type、四个按 [左下、左上、右上、右下] 排列的像素角点与类别。
     * 成功后写 gim_pose、abs_pose、abs_ypd、yaw、predict_yaw、re_points。
     * @return false 表示输入退化、PnP 失败、目标落在相机后方或部分结果非有限；
     * false 时不能继续把本次位姿当作有效结果，OpenCV 本身的异常仍可能抛出。
     * 当前相对 yaw 搜索范围为 [-70,69] 度、步长 1 度，内部换算为 rad 计算重投影误差。
     */
    bool Solve(Armor &armor);
    /**
     * @brief 能量机关 PnP 的历史预留声明，当前 ArmorSolver.cpp 没有对应定义或调用。
     * @param Buff_points 按接口命名预留的能量机关图像点。
     * @param pose 预留位置输出引用。
     * @param Rpose 预留旋转/姿态输出引用。
     * @param Aimpose 预留瞄准位置输出引用。
     * @param roll 预留 roll 引用。
     * @param yaw 预留 yaw 引用。
     * 因缺少实现，不能从本声明保证这些输出的坐标系、单位和有效性；勿视为已完成的大符解算接口。
     */
    void BuffPnP(vector<Point2f> Buff_points, Point3d &pose, Point3d &Rpose, Point3d &Aimpose, double &roll, double &yaw);
private:
    /**
     * @brief 用候选板朝向和当前目标 tvec 把物点重新投影到图像。
     * @param yaw 候选板在姿态补偿参考系中的相对朝向，rad，不是串口 yaw 指令。
     * @return 与输入四角点顺序一致的像素点，供 Solve 累加像素距离选择最佳朝向。
     * 普通板/前哨按 is_outpost_ 采用不同固定倾角；依赖最近 Solve 的物点与平移缓存。
     */
    vector<Point2f> Reprojection(const double &yaw);
    vector<Point3f> world_points;       ///< 当前目标选用的板局部平面四点，m，z=0；命名 world 不代表导航世界点。
    vector<Point3f> world_points_big;   ///< 大板物点缓存：半宽 0.115 m、半高 0.025 m。
    vector<Point3f> world_points_small; ///< 小板物点缓存：半宽 0.065 m、半高 0.025 m。
    cv::Mat camera_matrix_;            ///< 3x3 相机内参 K，焦距/主点以像素计，从 Camera.camera_matrix 读取。
    cv::Mat distort_coeffs_;           ///< 五项 OpenCV 畸变系数 [k1,k2,p1,p2,k3]，无量纲。
    Eigen::Matrix3d R_camera2gimbal_;   ///< 相机点旋转到云台系的 3x3 外参，行优先读取 YAML 列表。
    Eigen::Vector3d t_camera2gimbal_;   ///< 相机到云台点变换中相加的平移外参，m，在云台系表达。
    Eigen::Matrix3d R_gimbal2world_;    ///< 由当前反馈构造的云台到世界参考系旋转，不含底盘世界平移。
    double sys_yaw;                    ///< 当前反馈的云台 yaw，rad，参与板朝向的世界系表达。
    cv::Mat rvec, tvec;                ///< 最近 PnP 的物体到相机 Rodrigues 旋转向量（rad）与平移（m）。
    bool is_outpost_ = false;          ///< 当前 armor.number 是否为 outpost，决定重投影的固定板倾角。
    Eigen::Vector3d p_t_camera2gimbal_; ///< 优化辅助平移：实现计算为 R_camera2gimbal_^T * t_camera2gimbal_，m。
    Eigen::Matrix3d p_R_armor2world_;   ///< 优化辅助旋转 Rz(roll)*Rx(pitch)，不含 sys_yaw；不是每块板的完整世界姿态。
    Eigen::Vector3d p_t_armor2world_;   ///< 保存 p_R_armor2world_*相机位置+p_t_camera2gimbal_；当前重投影仍用原 tvec。

};

#endif
