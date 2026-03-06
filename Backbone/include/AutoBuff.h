#include "BuffDetect.h"
#include "BuffSlove.h"
#include "BuffTarget.h"
#include "BuffPlan.h"
class AutoBuff{
public: 
    AutoBuff();
    void AutoAim(cv::Mat &img,const Data_Save &ds,Data_Get &dg);

private:
    void Transmit(const Gimbal_Angle &gimbal_angle,const Data_Save &ds,Data_Get &dg,cv::Mat &img);
    void Show(cv::Mat &img);
    std::unique_ptr<auto_buff::BuffDetector> discover_;
    std::unique_ptr<auto_buff::Solver> slover_;
    std::unique_ptr<auto_buff::Target>target_;
    std::unique_ptr<auto_buff::Planner>planner_;
    int last_mode = 1;
};

