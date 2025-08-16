
#include "AIM.h"
#include "Base.h"
#include "test.h"
// ʹ��¼�����ݽ��з���
#define USE_DATA
// �۲������
cv::Point2f observation_point(0, 0);
// ���滭��
cv::Mat background;
// MPC��������
cv::Mat figure_mpc = cv::Mat(cv::Size(1024, 512), CV_8UC3, cv::Scalar(255, 255, 255));
// MPC�������������ͼ����Χ���Զ����ã�
int cnt = 0;
// ԭ�㣨�������ĵ㣩
cv::Point2f base_point = cv::Point2f(512, 512);

/**
 *@brief: ��ȡ��ǰʱ��
    *
    *@param: null
    *
    *@return: double ʱ���
    */
double now()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000;
}

Rate::Rate() : Rate(30) {}

/**
 *@brief: ֡�ʿ���
    *
    *@param: unsigned int ֡��
    *
    *@return: null
    */
Rate::Rate(unsigned int r)
:   m_LoopRate(r),
    m_start_time_with_usec(0)
{}

/**
 *@brief: �趨ʱ��˯���Դﵽ֡�ʿ���
    *
    *@param: null
    *
    *@return: void
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
 *@brief: ������΢��Ϊ��λ��ʱ���
    *
    *@param: null
    *
    *@return: void
    */
unsigned int getTimeUSecNow()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

// ����֡��
Rate rate(40);
int Test()
{
    int last_index;
    int frames_since_last_switch;
    // ¼������·��
    std::ifstream ifs("/home/zhbit/WRITE/build/adjust.txt");
    double time,x,y,z,yaw;
    Armor armor;
    ifs >> time >> x >> y >> z >> yaw;
    armor.abs_pose = Point3d(x,y,z);
    armor.yaw = yaw;
    std::unique_ptr<aim> aim_ptr; 
    std::shared_ptr<KinematicModel> m_status;     
    aim_ptr = std::make_unique<aim>(armor);
    m_status = std::make_shared<StandardModel>(aim_ptr->target_state); 
    float scaling = 100;
    while (!ifs.eof())
    {
        double time,x,y,z,yaw;
        background = cv::Mat(cv::Size(1024, 1024), CV_8UC3, cv::Scalar(255, 255, 255));
        ifs >> time >> x >> y >> z >> yaw;
        Armor armor;
        armor.abs_pose = Point3d(x,y,z);
        armor.yaw = yaw;
        cv::circle(background, base_point + cv::Point2f(armor.abs_pose.x, armor.abs_pose.y) * scaling, 3,
                    cv::Scalar(255,0 , 0), -1, 8);
        cv::line(background, base_point + cv::Point2f(armor.abs_pose.x, armor.abs_pose.y) * scaling,
                    base_point + cv::Point2f(armor.abs_pose.x, armor.abs_pose.y) * scaling + cv::Point2f(10 * cos(armor.yaw), 10 * sin(armor.yaw)),
                    cv::Scalar(255,0 , 0), 2, 8);
        aim_ptr->m_model_observer->Updatedt(time);
        // Ԥ��������
        m_status = aim_ptr->Resolve(armor);
        // ��ǰ״̬��
        Armors filter_armors = m_status->getArmors();
        double hit_time = GetDistance(Point3d(m_status->center.x,m_status->center.y,(m_status->radius[0]+m_status->radius[1]/2))) / bullet_speed + 0.14;
        // Ԥ������
        Armors predict_armors = m_status->getArmors(hit_time);
        Armor target_armor = m_status->getClosestArmor(hit_time,last_index,frames_since_last_switch);
        // Armor target_armor = m_status->getFacingArmor(hit_time,last_index,frames_since_last_switch);
        for (int i = 0; i < filter_armors.size(); i++)
        {
            // ��ǰ״̬������
            cv::circle(background, base_point + cv::Point2f(filter_armors[i].abs_pose.x, filter_armors[i].abs_pose.y) * scaling, 3,
                    cv::Scalar(0, 255, 0), -1, 8);
            // Ԥ��װ�װ�����
            cv::circle(background, base_point + cv::Point2f(predict_armors[i].abs_pose.x, predict_armors[i].abs_pose.y) * scaling, 2,
                    cv::Scalar(0, 0, 255), -1, 8);
            // Ԥ��װ�װ�߶�
            // cv::circle(background, base_point + cv::Point2f(predict_armors[i].abs_pose.x, predict_armors[i].abs_pose.y) * scaling, abs(predict_armors[i].abs_pose.z) * 50,
            //         cv::Scalar(0, 127, 255), 2, 8);
        }
        cv::imshow("background",background);
        cv::waitKey(1);
        rate.sleep();
    }
}