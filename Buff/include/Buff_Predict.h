#include <queue>
#include <cstdint>
#include <Eigen/Dense>
class BigNewtonGauss {
public:
    BigNewtonGauss();
    ~BigNewtonGauss() = default;

    void reset(); // 模式切换的时候需要重置

    // 返回是否能预测
    int fit(double angle,double speed,uint64_t timestamp);
    // 到 timestamp 时大风车的角度
    double getAngle(uint64_t timestamp);

private:
    std::deque<uint64_t> fit_timestamps;
    std::deque<double> fit_speeds;

    int min_fit_samples = 10; // 最快可以开始拟合的帧数
    int max_fit_samples = 100; // 拟合所需的最高帧数

    double a = 0.8;
    double omega = 1.9;
    double phase = 0; // 相位
    double b = 1.0;

    int initialized = 0;
    double direction = 0.0; // 1.0 or -1.0;

    uint64_t last_time = 0;
    double last_angle = 0;

    double a_init;      // a的初始值
    double omega_init;  // omega的初始值
    double phase_init;  // phase的初始值
    double b_init;      // b的初始值
    double lambda;      // 惩罚权重
    double lambda_a;
};

