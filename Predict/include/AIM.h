#include "EKF.h"
#include "Pridict.h"
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include "SerialPort.h"
#include "Angel_Calculate.h"
#include "CA.h"
using namespace std;
class Aim
{
public:
    // 初始化
    Aim(Armor armor);
    // 主接口
    Armor Shoot(const Armors &armors,Mat &img,double time);
    // ekf的主要部分
    std::shared_ptr<KinematicModel> Resolve(const Armor &armor);
    // 匹配装甲板
    std::map<int, int> Match(const Eigen::MatrixXd &score);
    // 初始化状态量
    void InitEKF (Armor armor);
    // 计算得分，匹配装甲板
    Eigen::MatrixXd GetScoreMat(const Armor &detect_armor, const Armors &standard_armors);
    // 读配置文件
    YAML::Node readYAML(const std::string& filename);
    // 绘制三维重投影
    void DraftingSoftware(Mat &img);
    // 均值滤波，过滤角速度
    double Homogeneity(double new_value);
    // 选择瞄准模式
    void ChooseFireModel();
    // CA的主要部分 
    void SloveCA(const Armors armors,double time);
    ///////////////////////////          变量               ////////////////////////////////////////////
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW ;
    double m_score_tolerance;                                    // 装甲板匹配得分最大值
    std::shared_ptr<KinematicModel> m_status;                    // 当前状态。为了保留最佳目标装甲板，必须声明为类成员变量以保留其索引值
    std::shared_ptr<ExtendedKalmanFilter> m_model_observer;      // 状态观测器
    std::shared_ptr<CAEKF>m_model_translation;                   // 平移观测器
    Eigen::VectorXd target_state;                                // EKF状态量
    Eigen::VectorXd translation_state;                           // 平移状态量
    double ca_yaw;                                               // 平移模型中朝向角
    int ca_try_times;                                            // ca模型的换板
    Point2d ui;
    double predict_time;                                         // 预测时长
    Point3d predict_point;                                       // 预测点俯视图
    bool show;
    int control_model;
    std::deque<double> buffer;
    double sum = 0;
    const size_t window_size = 40;
    int last_index = 0;
    int frames_since_last_switch = 0;
private:
   
    class Match
    {
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
        void getCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, std::vector<std::vector<int>> &result, int start, int k);

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
