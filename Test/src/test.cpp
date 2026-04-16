#include "AIM.h"
#include "Base.h"
#include "test.h"
#include <random>
#include <algorithm>
#include <cstdlib>  
#include <ctime>   
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
#include "Planner.h"
// 使用录制数据进行仿真
// #define USE_DATA
int random_seed = 42;
#ifdef USE_DATA
// 数据文件路径
const std::string DATA_FILE_PATH = "/home/killer_machine/WRITE/build/input_data.txt";
// 记录最后时间戳
double last_timestamp = 0;
// 数据索引
size_t data_index = 0;
// 存储的数据
std::vector<std::pair<double, Armors>> recorded_data;
#endif

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

// 仿真参数（这些就是真实状态）
cv::Point2f center(5, 0);              // 中心点（固定）
float z[3] = {1.114, 1.126, 1.318};    // 前哨站三块装甲板高度
float outpost_radius = 0.225;           // 前哨站旋转半径
float palstance = 0.8;                 // 角速度系数，实际角速度=0.8*pi rad/s
float scaling = 80;       // 放大比例
double angle = 0;          // 起始角度
float dt = 1e-2;           // 默认时间步长
float sigma = 0.02;        // 噪声大小
std::default_random_engine rng;

#ifdef USE_DATA
/**
 * @brief 从文件加载录制的数据
 */
bool loadRecordedData() {
    std::ifstream in(DATA_FILE_PATH);
    if (!in.is_open()) {
        std::cout << "无法打开数据文件: " << DATA_FILE_PATH << std::endl;
        return false;
    }
    
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream iss(line);
        double timestamp;
        
        // 读取时间戳
        if (!(iss >> timestamp)) {
            continue;
        }
        
        // 读取装甲板数据
        Armors armors_in_frame;
        Armor armor;
        
        while (true) {
            // 读取位置 (x, y, z)
            if (!(iss >> armor.abs_pose.x() >> armor.abs_pose.y() >> armor.abs_pose.z())) {
                break;
            }
            
            // 读取yaw
            if (!(iss >> armor.yaw)) {
                break;
            }

            // 转换为ypd
            armor.abs_ypd = tools::xyz2ypd(armor.abs_pose);
            armors_in_frame.push_back(armor);
            
            // 检查是否还有数据
            if (iss.peek() == EOF || iss.peek() == '\n') {
                break;
            }
        }
        
        if (!armors_in_frame.empty()) {
            recorded_data.emplace_back(timestamp, armors_in_frame);
        }
    }
    
    in.close();
    
    if (recorded_data.empty()) {
        std::cout << "数据文件为空或格式错误" << std::endl;
        return false;
    }
    
    std::cout << "成功加载 " << recorded_data.size() << " 帧数据" << std::endl;
    return true;
}

/**
 * @brief 从录制的数据获取装甲板
 */
Armors getRecordedArmors(double& frame_dt) {
    if (recorded_data.empty() || data_index >= recorded_data.size()) {
        // 如果没有数据或已播放完，返回空装甲板列表
        return Armors();
    }
    
    auto& [recorded_timestamp, armors] = recorded_data[data_index];
    frame_dt = recorded_timestamp;  // 使用录制数据的时间戳作为dt
    
    data_index++;
    
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
    
    cv::circle(background, base_point + observation_point * scaling, 5, cv::Scalar(127, 127, 127), -1, 8);
    
    return armors;
}
#endif

/**
 * @brief 获取当前时间（秒）
 */
double now()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}

Rate::Rate() : Rate(100) {}

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

bool init_flag = false;
// 帧率控制器
Rate rate(100);

/**
 * @brief 生成仿真装甲板数据并只保留距离最近的1个
 */
Armors generateSimulatedArmors() {
    Armors armors;
    armors.resize(3);

    // 前哨站中心固定，x/y方向速度为0
    angle += PI * dt * palstance; 
    angle = _std_radian(angle);
    
    // 生成3个前哨站装甲板，yaw间隔2*PI/3
    for (size_t i = 0; i < armors.size(); i++) {
        // 添加高斯噪声
        std::normal_distribution<float> dist_pos(0, sigma);
        std::normal_distribution<float> dist_angle(0, sigma * 10);
        
        // 计算装甲板位置（相对固定中心自转）
        float armor_r = outpost_radius;
        float armor_z = z[i];
        float armor_angle = angle + i * 2.0 * PI / 3.0;
        
        armors[i].abs_pose = {
            center.x + armor_r * cos(armor_angle) + dist_pos(rng),
            center.y + armor_r * sin(armor_angle) + dist_pos(rng),
            armor_z + dist_pos(rng)
        };
        armors[i].abs_ypd = tools::xyz2ypd(armors[i].abs_pose);
        // 计算装甲板朝向角
        armors[i].yaw = _std_radian(armor_angle + dist_angle(rng));
        armors[i].predict_yaw = _std_radian(armor_angle);
        armors[i].number = "outpost";
        armors[i].confidence = armor_r;
    }

    // 绘制无噪声三装甲板投影点（参考）
    for (size_t i = 0; i < armors.size(); ++i) {
        float ideal_angle = angle + i * 2.0 * PI / 3.0;
        cv::Point2f ideal_xy(
            center.x + outpost_radius * cos(ideal_angle),
            center.y + outpost_radius * sin(ideal_angle)
        );
        cv::circle(background,base_point + ideal_xy * scaling,4,
            cv::Scalar(0, 200, 0),-1,8);
    }
    

    if(init_flag == false){
        init_flag = true;
    }else{
        // 按距离观察点的距离排序（从近到远）
        std::sort(armors.begin(), armors.end(), [](const Armor& a, const Armor& b)
         {return a.abs_pose.norm() < b.abs_pose.norm();});
    }
    armors.resize(1);
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
 std::ofstream in("mpc_log.txt", std::ios::app); 
int Test() {
    srand(time(nullptr));
    
    // 初始化随机数生成器
    rng.seed(random_seed);
    srand(random_seed);
    
#ifdef USE_DATA
    // 加载录制的数据
    if (!loadRecordedData()) {
        std::cout << "使用模拟数据替代" << std::endl;
    } else {
        std::cout << "使用录制的数据进行仿真" << std::endl;
    }
#endif
    
    // 初始化预测器和模型
    std::unique_ptr<Aim> aim_ptr;
    std::shared_ptr<KinematicModel> m_status;
    std::unique_ptr<Planner> mpc = std::make_unique<Planner>();
    io::SerialPort serial_("/dev/ttyACM0");
    Data_Get dg;
    
    while (true) {
        // 清空背景
        background = cv::Mat(cv::Size(1024, 1024), CV_8UC3, cv::Scalar(255, 255, 255));
        
        // 获取装甲板数据
        Armors armors;
        double actual_dt = dt;  // 默认使用固定时间步长
        
#ifdef USE_DATA
        if (!recorded_data.empty() && data_index < recorded_data.size()) {
            // 使用录制的数据，获取实际的时间步长
            armors = getRecordedArmors(actual_dt);
            // 如果装甲板数据为空，跳过
            if (armors.empty()) {
                if (recorded_data.empty() || data_index >= recorded_data.size()) {
                    std::cout << "数据播放完毕" << std::endl;
                }
                rate.sleep();
                continue;
            }
        } 
        else 
#endif
        {
            // 使用模拟数据，使用固定时间步长
            armors = generateSimulatedArmors();
            actual_dt = dt;  // 对于模拟数据，使用固定的dt
        }
        
        // 如果是第一次运行，初始化预测器
        if (!aim_ptr) {
            aim_ptr = std::make_unique<Aim>(armors[0]);
            m_status = std::make_shared<OutpostModel>(aim_ptr->target_state);
        }
        
        // 预测器解算，使用实际的时间步长
        aim_ptr->Resolve(armors, actual_dt);
        m_status = std::make_shared<OutpostModel>(aim_ptr->target_state);
        // 获取当前状态估计
        Armors filter_armors = m_status->GetArmors(0);
        
        // 检查是否有有效数据
        if (!filter_armors.empty() && !armors.empty()) {
            // MPC轨迹规划
            auto target = mpc->TrajectoryPlanning(aim_ptr->target_state);
            in << target.target_yaw << " " << target.manage_yaw << "\n";
            // 发送控制数据
            dg.Vision2Control((target.manage_yaw)*1e4, target.target_pitch*1e4, 
                             target.yaw_vel*1e4, target.yaw_acc*1e4, 1, 0);
            serial_.SendData(dg);
            
            // 绘制状态估计和预测结果
            for (size_t i = 0; i < filter_armors.size(); i++) {
                // 当前状态估计量（红色）
                cv::circle(background, 
                        base_point + cv::Point2f(filter_armors[i].abs_pose.x(), filter_armors[i].abs_pose.y()) * scaling, 
                        3, cv::Scalar(0, 0, 255), -1, 8);
            }
            
        
        }
        // 在窗口上显示当前数据源和dt信息
        std::string data_source = "Data Source: ";
        std::string dt_info = "dt: " + std::to_string(actual_dt);
        
#ifdef USE_DATA
        if (!recorded_data.empty() && data_index < recorded_data.size()) {
            data_source += "RECORDED (" + std::to_string(data_index) + "/" + 
                          std::to_string(recorded_data.size()) + ")";
        } else {
            data_source += "SIMULATED";
        }
#else
        data_source += "SIMULATED";
#endif
        
        cv::putText(background, data_source, cv::Point(20, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        cv::putText(background, dt_info, cv::Point(20, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        
        // 显示结果
        cv::imshow("background", background);

        // 检查按键输入
        char c = cv::waitKey(1);
        // 控制帧率
        rate.sleep();
    }
    
    return 0;
}