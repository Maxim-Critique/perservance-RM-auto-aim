#include "find_armor.h"
#include "Angle_Calculate.h"
#include "AIM.h"
#include <algorithm> 
#include <sys/time.h>
class Auto_aim{
public:
    Auto_aim();
    int IAmFuture(Mat &img,uint64_t timestamp);
private:
    //防止地面倒影
    void ProcessArmors(Armors& armorss);
    //防甩头
    bool Protection();
    //获得ROI
    Rect MakeROI(Point2d found_armor);
    //发ID
    int FoundID(string x);
    //从多个装甲板中按照ID选择进整车观测的装甲板
    Armors ChooseProcess(Armors armors,Mat &img);
    //检查装甲板组是否为空
    bool CheckArmors();
    //没找到对象时重置
    void Reboot();
    //丢失所追踪目标时重置
    void ReEKF();
    //更新时间间隔
    void UpdateTimeInterval(uint64_t timestamp);
    //发送给电控
    void Transmit(const Armor &armor);
    //读取配置文件
    YAML::Node readYAML(const std::string& filename);
    //发射间隔
    bool ShootAble();
    //检测整车观测
    void CheckAimer(Armor armor);
    /////////////////////////////////变量/////////////////////////////
    double time_interval;                             //两次时间间隔
    std::unique_ptr<Aim> aim_ptr;                     //整车观测的指针
    int try_times;                                    //整车观测找板次数
    string aim_id;                                    //瞄准的ID
    uint64_t last_time;                               //上一次时间
    Rect roi;                                         //roi的区域
    Gimbal_Angle initial_abs;                         //相对于本车的世界系yaw角
    Gimbal_Angle gimbal_angle;                        //发送给电控的云台角
    double compensate_pitch,compensate_yaw;           //pitch,yaw的补偿角 
    std::unique_ptr<Tradition> find_ptr;              //找灯条和装甲板的指针
    double m_next_shoot_time;                         //下一次开火时间
    uint64_t frist_find;                              //第一次找到
    double supple_top;                                // 小陀螺补偿

};

