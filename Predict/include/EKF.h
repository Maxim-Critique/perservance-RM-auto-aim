#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include "find_armor.h"

class ExtendedKalmanFilter {
public:
    explicit ExtendedKalmanFilter(const Eigen::VectorXd& initial_state);
    
    // 核心接口
    Eigen::VectorXd predict(double dt);
    Eigen::VectorXd update(const Armors& armors, const std::vector<int>& match_indices);
    
    // 配置加载
    void loadConfig(const std::string& filename);
    // NIS检验，新息卡方检验
    void NIS(std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> compute);
    bool is_lost = false;
private:
    // 状态变量
    Eigen::VectorXd x_pri;  // 先验状态
    Eigen::VectorXd x_post; // 后验状态
    Eigen::MatrixXd P_pri;  // 先验协方差
    Eigen::MatrixXd P_post; // 后验协方差
    Eigen::MatrixXd K;      // 卡尔曼增益
    Eigen::MatrixXd I;      // 单位矩阵
    
    // 噪声参数
    double dd;      // 平面距离过程噪声
    double da;      // 角度过程噪声
    double dz;      // 高度过程噪声
    double dr;      // 半径过程噪声

    double dt_;     // 时间步长
    std::deque<bool> nis_history_;                  // NIS历史记录队列
    static constexpr size_t nis_window = 15;        // 历史窗口大小
    static constexpr size_t nis_threshold_count = 7;// 触发阈值次数
    static constexpr double nis_threshold = 0.711;  // 卡方检验阈值

    // 策略基类
    class UpdateStrategy {
    protected:
        static double rd;      // 位置观测噪声（继承给所有策略）
        static double rz;      // 高度观测噪声
        static double ra;      // 角度观测噪声
        static double rr;      // 半径观测噪声
        static double m_gain;  // 测量增益系数
        
    public:
        struct Data {
            const Armors& armors;
            std::vector<int> match_indices;
        };
        
        virtual ~UpdateStrategy() = default;
        virtual void configure(const Eigen::VectorXd& x_pri, const Data& data) = 0;
        virtual std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() = 0;

        // 从父类加载噪声参数
        static void LoadNoiseParams(double rd_, double rz_, double ra_, double m_gain_,double rr_) {
            rd = rd_;
            rz = rz_;
            ra = ra_;
            m_gain = m_gain_;
            rr = rr_;
        }
    };

    // 单装甲板策略
    class SingleArmorStrategy : public UpdateStrategy {
    public:
        void configure(const Eigen::VectorXd& x_pri, const Data& data) override;
        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() override;
    
    private:
        Eigen::VectorXd x;
        Armor armor;
        int index;
    };

    // 双装甲板策略 
    class DualArmorStrategy : public UpdateStrategy {
    public:
        void configure(const Eigen::VectorXd& x_pri, const Data& data) override;
        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() override;
    
    private:
        Eigen::VectorXd x;
        Armor armor1;
        Armor armor2;
        int index1;
        int index2;
    };

    // 内部方法
    Eigen::MatrixXd computeProcessNoise() const;
    std::unique_ptr<UpdateStrategy> createStrategy(const UpdateStrategy::Data& data);
    void constrainState(Eigen::VectorXd& state);
};

#endif // EKF_H