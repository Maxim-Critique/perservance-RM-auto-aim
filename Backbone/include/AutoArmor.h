#include "AIM.h"
#include <algorithm> 
#include <sys/time.h>
#include "Planner.h"
class AutoArmor{
public:
    AutoArmor();
    void AutoAim(Mat &img,const Data_Save &ds,Data_Get &dg);
private:
    // 发ID
    int FoundID(string x);
    // 从多个装甲板中按照ID选择进整车观测的装甲板
    Armors ChooseProcess(Armors &armors,const Data_Save &ds);
    // 发送给电控
    void Transmit(const Gimbal_Angle &gimbal_angle,const Data_Save &ds,Data_Get &dg);
    // 读取配置文件
    YAML::Node readYAML(const std::string& filename);
    // 可视化
    void ShowArmors(const Armors &armor,Mat &img);

    void DontMove(const Data_Save &ds,Data_Get &dg);
    /////////////////////////////////变量/////////////////////////////
    double time_interval;                             // 两次时间间隔
    std::unique_ptr<Aim> aim_ptr;                     // 整车观测的指针
    std::unique_ptr<Planner> planner_ptr;                 // 轨迹规划指针
    std::unique_ptr<Slover> solve_ptr;                // 姿态解算指针
    time_point last_time;                             // 上一次时间
    double compensate_pitch,compensate_yaw;           // pitch,yaw的补偿角 
    std::unique_ptr<Tradition> find_ptr;              // 找灯条和装甲板的指针
    double m_next_shoot_time;                         // 下一次开火时间
    double show;                                      // 可视化开关
};

