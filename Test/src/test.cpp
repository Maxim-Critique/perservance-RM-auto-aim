#include "AIM.h"
#include "Base.h"
#include "test.h"
#include <random>
#include <algorithm>
#include <cstdlib>  
#include <ctime>   
// 使用录制数据进行仿真
// #define USE_DATA

// 观察点坐标
cv::Point2f observation_point(0, 0);
// 仿真画面
cv::Mat background;
// MPC解算数据
cv::Mat figure_mpc = cv::Mat(cv::Size(1024, 512), CV_8UC3, cv::Scalar(255, 255, 255));
// MPC解算计数（超出图表范围后自动重置）
int cnt = 0;
// 原点（画面中心点）
cv::Point2f base_point = cv::Point2f(512, 512);

// 仿真参数
cv::Point2f center(3, 0);  // 中心点初始位置
float z[2] = {0.15, 0.1};  // 装甲板高度
float r[2] = {0.2, 0.3};   // 装甲板旋转半径
float palstance = 4;       // 角速度
float scaling = 100;       // 放大比例
double angle = 0;          // 起始角度
float dt = 1e-2;           // 每帧时间
float sigma = 0.03;        // 噪声大小
std::default_random_engine rng;

// 路径点定义
std::vector<cv::Point2f> waypoints = {
    cv::Point2f(2, 2),
    cv::Point2f(-2, 2),
    cv::Point2f(-2, -2),
    cv::Point2f(2, -2)
};
int current_target_index = 0;
float movement_speed = 0.8; // 移动速度

/**
 * @brief 获取当前时间（秒）
 */
double now()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}

Rate::Rate() : Rate(30) {}

/**
 * @brief 帧率控制器构造函数
 */
Rate::Rate(unsigned int r)
:   m_LoopRate(r),
    m_start_time_with_usec(0)
{}

/**
 * @brief 控制帧率
 */
void Rate::sleep()
{
    double period = 1. / m_LoopRate;
    double cur_time = now();
    double expect_end_time = m_start_time_with_usec + period;

    if (expect_end_time > cur_time)
    {
        usleep((expect_end_time - cur_time) * 1000000);
    }
    m_start_time_with_usec = now();
}

/**
 * @brief 获取当前时间（微秒）
 */
unsigned int getTimeUSecNow()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
} 

// 帧率控制器
Rate rate(100);

/**
 * @brief 标准化角度到[-π, π]范围
 */
float normalizeAngle(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

/**
 * @brief 计算两点间距离
 */
float getDistance(const Point3d& p) {
    return sqrt(pow(p.x - observation_point.x, 2) + 
                pow(p.y - observation_point.y, 2));
}

/**
 * @brief 生成仿真装甲板数据并只保留距离最近的1个
 */
Armors generateSimulatedArmors() {
    Armors armors;
    armors.resize(4);
    
    // 更新中心位置（在四个顶点之间移动）
    cv::Point2f target = waypoints[current_target_index];
    cv::Point2f direction = target - center;
    float distance = cv::norm(direction);
    
    if (distance < 0.1f) {
        // 到达目标点，切换到下一个点
        current_target_index = (current_target_index + 1) % waypoints.size();
        target = waypoints[current_target_index];
        direction = target - center;
    }

    // 标准化方向向量并乘以速度
    if (distance > 0) {
        direction = (direction / distance) * movement_speed * dt;
        center += direction;
    }
    
    // 更新角度（保持旋转）
    angle += PI * dt * palstance;
    
    // 生成4个装甲板
    for (size_t i = 0; i < armors.size(); i++) {
        // 添加高斯噪声
        std::normal_distribution<float> dist_pos(0, sigma);
        std::normal_distribution<float> dist_angle(0, sigma * 10);
        
        // 计算装甲板位置
        float armor_r = r[i % 2];
        float armor_z = z[i % 2];
        float armor_angle = angle + 0.5 * i * PI;
        
        armors[i].abs_pose = {
            center.x + armor_r * cos(armor_angle) + dist_pos(rng),
            center.y + armor_r * sin(armor_angle) + dist_pos(rng),
            armor_z + dist_pos(rng)
        };
        armors[i].abs_ypd = C_C.xyz2ypd(armors[i].abs_pose);
        // 计算装甲板朝向角
        armors[i].yaw = normalizeAngle(armor_angle + dist_angle(rng));
    }
    
    // 按距离观察点的距离排序（从近到远）
    std::sort(armors.begin(), armors.end(), 
        [](const Armor& a, const Armor& b) {
            return a.abs_pose.norm() < b.abs_pose.norm();
        });
    
    // 只保留距离最近的1个装甲板
    armors.resize(1 + rand() % 2);
    
    // 在仿真图上绘制装甲板
    for (const auto& armor : armors) {
        cv::circle(background, 
                  base_point + cv::Point2f(armor.abs_pose.x(), armor.abs_pose.y()) * scaling, 
                  3, cv::Scalar(255, 0, 0), -1, 8);
        cv::line(background,
                base_point + cv::Point2f(armor.abs_pose.x(), armor.abs_pose.y()) * scaling,
                base_point + cv::Point2f(armor.abs_pose.x(), armor.abs_pose.y()) * scaling + 
                    cv::Point2f(10 * cos(armor.yaw), 10 * sin(armor.yaw)),
                cv::Scalar(255, 0, 0), 2, 8);
    }
    
    // 绘制中心点和观察点
    cv::circle(background, base_point + center * scaling, 5, cv::Scalar(0, 0, 0), -1, 8);
    cv::circle(background, base_point + observation_point * scaling, 5, cv::Scalar(127, 127, 127), -1, 8);
    
    return armors;
}

int Test() {
    srand(time(nullptr));
    
    // 初始化随机数生成器
    rng.seed(time(NULL));
    
    // 初始化预测器和模型
    std::unique_ptr<Aim> aim_ptr;
    std::shared_ptr<KinematicModel> m_status;
    std::unique_ptr<MPC> mpc = std::make_unique<MPC>();
    io::SerialPort serial_("/dev/ttyACM0");
    Data_Get dg;
    while (true) {
        // 清空背景
        background = cv::Mat(cv::Size(1024, 1024), CV_8UC3, cv::Scalar(255, 255, 255));
        
        // 生成仿真装甲板数据
        Armors armors = generateSimulatedArmors();
        
        // 如果是第一次运行，初始化预测器
        if (!aim_ptr) {
            aim_ptr = std::make_unique<Aim>(armors[0]);
            m_status = std::make_shared<StandardModel>(aim_ptr->target_state);
        }
        
        // 预测器解算
        m_status = aim_ptr->Resolve(armors, dt);
        // 获取当前状态估计
        Armors filter_armors = m_status->GetArmors(0);
        // MPC轨迹规划
        auto target = mpc->TrajectoryPlanning(*m_status);
        dg.Vision2Control((target.manage_yaw)*1e4,target.target_pitch*1e4,target.yaw_vel*1e4,target.yaw_acc*1e4,1,0);
        serial_.SendData(dg);
        // 绘制状态估计和预测结果
        for (size_t i = 0; i < filter_armors.size(); i++) {
            // 当前状态估计量（红色）
            cv::circle(background, 
                    base_point + cv::Point2f(filter_armors[i].abs_pose.x(), filter_armors[i].abs_pose.y()) * scaling, 
                    3, cv::Scalar(0, 0, 255), -1, 8);
            
            // 计算无噪声的理想装甲板位置（绿色）
            float armor_r = r[i % 2];
            float armor_angle = angle + 0.5 * i * PI;
            
            cv::Point2f ideal_position(
                center.x + armor_r * cos(armor_angle),
                center.y + armor_r * sin(armor_angle)
            );
            
            cv::circle(background, 
                    base_point + ideal_position * scaling, 
                    2, cv::Scalar(0, 255, 0), -1, 10);
        }
        
        // 显示目标装甲板（紫色圆圈）
        cv::circle(background, 
                  base_point + cv::Point2f(m_status->center.x, m_status->center.y) * scaling, 
                  10, cv::Scalar(255, 0, 255), 2, 8);
        
        // 显示结果
        cv::imshow("background", background);
        
        // 检查按键输入
        char c = cv::waitKey(1);
        if (c == 'q') {
            break;
        }
        
        // 控制帧率
        rate.sleep();
    }
    
    return 0;
}