#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "find_armor.h"
#include "SerialPort.h"
#include <algorithm> 
#include "Coordinate.h"


/**
 * @brief 运动学状态
 */
class KinematicModel
{
public:
    KinematicModel();
    ~KinematicModel();

    virtual Armors GetArmors(double predict_time = 0) = 0;
    virtual Armor GetClosestArmor(double predict_time,int &last_index,int &frames_since_last_switch) = 0;
    virtual Armor GetFacingArmor(double predict_time,int &last_index,int &frames_since_last_switch,Point3d &center_pose) = 0;
    virtual void print(std::string tag) = 0;

public:
    cv::Point2d center;             // 旋转中心位置
    cv::Point2d velocity;
    std::vector<double> height;     // 装甲高度。0对应索引0、2，1对应索引1、3
    std::vector<double> radius;     // 旋转半径。0对应索引0、2，1对应索引1、3
    double phase;                   // 角度（并非定值）
    double palstance;               // 角速度
   
    int number;                     // 装甲板数量
    int index;                      // 当前目标索引
};

class StandardModel : public KinematicModel
{
public:
    StandardModel();
    StandardModel(const Eigen::Matrix<double, 10, 1> &X);
    ~StandardModel();

    /**
     * @brief 为了保留index，赋值时不继承index
     */
    StandardModel operator=(const StandardModel &status);

    /**
     * @brief 获取该运动状态下所有装甲板
     * @param predict_time 预测时间
     */
    Armors GetArmors(double predict_time = 0);

    /**
     * @brief 获取目标装甲板
     * @param predict_time 预测时间
     */
    Armor GetClosestArmor(double predict_time,int &last_index,int &frames_since_last_switch);

    /**
     * @brief 获取正对的装甲板
     */
    Armor GetFacingArmor(double predict_time,int &last_index,int &frames_since_last_switch,Point3d &center_pose);

    /**
     * @brief 打印信息
     */
    void print(std::string tag);
    
    
};
