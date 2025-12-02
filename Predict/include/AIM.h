#ifndef AIM_H
#define AIM_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <map>
#include <vector>
#include <deque>
#include "find_armor.h"
#include "EKF.h"
#include "MPC.h"

#include "SerialPort.h"
#include "Angle_Calculate.h"
using namespace cv;
using namespace std;

class Aim {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    // 初始化
    Aim(Armor armor);
    // 主接口
    std::shared_ptr<KinematicModel> Estimate(const Armors &armors, double &time);
    // EKF的主要部分
    std::shared_ptr<KinematicModel> Resolve(const Armors &armors, double time);
    // 匹配装甲板
    std::map<int, int> Match(const Eigen::MatrixXd &score);
    // 初始化状态量
    void InitEKF(Armor armor);
    // 计算得分，匹配装甲板
    Eigen::MatrixXd GetScoreMat(const Armors &detect_armors, const Armors &standard_armors);
    // 均值滤波，过滤角速度
    double Homogeneity(double new_value);
    
    ///////////////////////////          变量               ////////////////////////////////////////////
    double m_score_tolerance;                                    // 装甲板匹配得分最大值
    std::shared_ptr<KinematicModel> m_status;                    // 当前状态。为了保留最佳目标装甲板，必须声明为类成员变量以保留其索引值
    std::shared_ptr<ExtendedKalmanFilter> m_model_observer;      // 状态观测器
    Eigen::VectorXd target_state;                                // EKF状态量
    string aim_id;                                               // 瞄准对象
    std::deque<double> buffer;
    double sum = 0;
    const size_t window_size = 40;
    int last_index = 0;
    int frames_since_last_switch = 0;

private:
    class Match {
    public:
        Match();
        ~Match();

        /**
         * @brief 获取矩阵中的一组位于互不相同的行和列的数的位置，且选取的这些数的和最小
         * @param matrix 输入的矩阵
         * @param n 匹配装甲板数
         * @return map<int, int> 行和列的位置
         */
        std::map<int, int> GetMatch(Eigen::MatrixXd matrix, double score_max, int m);

        bool debug = false;     // 是否输出生成的所有组合数

    private:
        /**
         * @brief 求出在n个数中选取k个数，所有的组合结果，即C(n,k)的所有结果
         * @param input 一组数，由用户决定
         * @param tmp_v 存储中间结果
         * @param result C(n,k)结果
         * @param start 起始位置，应指定为0
         * @param k k个数，由用户决定
         */
        void GetCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, 
                                   std::vector<std::vector<int>> &result, size_t start, size_t k);

        double min = 0, tmp = 0;                    // 最小值

        std::vector<int> col;                       // 数列中的每个数代表矩阵的每一列
        std::vector<int> row;                       // 数列中的每个数代表矩阵的每一行
        std::vector<int> tmp_v;                     // 存储C(n,k)的中间结果

        std::vector<std::vector<int>> result;       // 存储C(n,k)的结果
        std::vector<std::vector<int>> nAfour;       // 存储A(n,4)的结果
        std::vector<std::vector<int>> fourAfour;    // 存储A(4,4)的结果

        std::map<int, int> row_col;                 // 存储最终结果，row_col[i]=j表示矩阵的第i行第j列是要选取的数
        
    } m_match;
};

#endif // AIM_H