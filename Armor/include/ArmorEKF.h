#ifndef ARMOR_EKF_H
#define ARMOR_EKF_H

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <memory>
#include "ArmorTypes.h"

/**
 * @file ArmorEKF.h
 * @brief 历史版装甲板专用十维扩展卡尔曼滤波器，不是 tools 中的通用同名滤波器。
 * 当前 CMake 已排除 ArmorEKF.cpp；现行 Armor/sentry 使用 tools/extended_kalman_filter。
 *
 * 普通目标状态：[xc, vx, yc, vy, z1, z2, r1, r2, phase, omega]。
 * 前哨站状态：  [xc, vx, yc, vy, z1, z2, z3, r,  phase, omega]。
 * 位置/半径为 m，速度为 m/s，相位为 rad，角速度为 rad/s；两个布局不能互换。
 * 板观测按 [位置方位角, 位置仰角, 距离, 板朝向 yaw] 排列，前三项来自 abs_ypd。
 */

/**
 * @brief 使用匀速平移/旋转预测和单板、双板或前哨观测策略完成 EKF 更新。
 * 典型顺序为 predict(dt) -> update(armors, match_indices)，由历史 Aim 负责关联与重建。
 * is_lost 是重建请求，既可能来自无匹配，也可能来自 NIS 异常统计，并非完整追踪状态机。
 */
class ExtendedKalmanFilter {
public:
    /**
     * @brief 以给定十维状态同时初始化先验和后验，并建立初始协方差。
     * @param initial_state 按目标类别对应的十维布局，单位见文件说明。
     * @param armor_number 分类标签；只有字符串 outpost 走前哨布局及更新分支。
     * 构造函数不执行首次预测，dt_ 和增益等工作量由后续步骤设置。
     */
    explicit ExtendedKalmanFilter(const Eigen::VectorXd& initial_state, const std::string& armor_number);

    /**
     * @brief 从 x_post/P_post 预测到 dt 秒后，更新 x_pri/P_pri 并返回先验状态。
     * F 对 xc/yc/phase 分别加入 vx*dt、vy*dt、omega*dt，高度与半径保持不变。
     * dt 应由调用方保证为合理的帧间隔；此历史实现没有独立拒绝负值或异常大值。
     */
    Eigen::VectorXd predict(double dt);
    /**
     * @brief 按匹配索引选择策略并完成观测更新，返回本次状态估计。
     * @param armors 已有 abs_pose、abs_ypd、yaw 的实测板，须与匹配顺序一致。
     * @param match_indices 对应模型板索引；普通目标有效范围 0~3，前哨为 0~2。
     * 当前策略直接读取 armors[0] 或 armors[0/1]，不自行重排观测，调用方要保证一致性。
     * 无匹配时置 is_lost 并返回 x_pri；不支持的普通匹配数量也返回 x_pri。
     * 成功更新采用 Joseph 形式更新协方差；前哨有效更新满 100 次后将 omega 固定为
     * 累计符号对应的 +/-2.51 rad/s，普通目标另执行 NIS 统计。
     */
    Eigen::VectorXd update(const Armors& armors, const std::vector<int>& match_indices);

    /**
     * @brief 将 residual^T * S^-1 * residual 与固定阈值比较，维护异常历史。
     * @param compute 观测策略返回的三元组 (residual, H, R)。
     * 实现以 S=H*P_post*H^T+R 计算统计量；最近最多 15 次中异常达到 7 次即置 is_lost。
     * 0.511 是源码固定阈值，并未根据观测维数动态计算卡方分位数。
     */
    void NIS(std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> compute);
    bool is_lost = false; ///< 失配/统计异常后的重建标记；当前实现不在正常更新时自动清回 false。
private:
    // 先验是本帧观测融合之前的预测，后验是融合之后的估计，状态布局均见文件说明。
    Eigen::VectorXd x_pri;  ///< 10x1 先验状态，predict 写入、观测策略读取。
    Eigen::VectorXd x_post; ///< 10x1 后验状态，成功 update 写入、下一次 predict 使用。
    Eigen::MatrixXd P_pri;  ///< 10x10 先验协方差，由 F*P_post*F^T+Q 得到。
    Eigen::MatrixXd P_post; ///< 10x10 后验协方差，各项量纲随状态分量而定。
    Eigen::MatrixXd K;      ///< 卡尔曼增益，单板/前哨为 10x4，双板为 10x8。
    Eigen::MatrixXd I;      ///< 10x10 单位矩阵，供后验协方差更新使用。

    std::string armor_number_; ///< 目标类别标签，决定状态布局、噪声和前哨角速度处理。
    double dd;                ///< 平面加速度噪声方差尺度；computeProcessNoise 取普通 100、前哨 1。
    double da;                ///< 角加速度噪声方差尺度；computeProcessNoise 取普通 400、前哨 10。
    double dt_;               ///< 最近一次 predict 的时间步长，s；生成 Q 时组合 dt^2/dt^3/dt^4。
    int outpost_cnt_ = 0;                  ///< 前哨成功观测更新计数，累加到 100 后停止增加。
    double outpost_vyaw_sum_ = 0.0;         ///< 前 100 次前哨更新的 omega 累计值，用于确定固定转速的符号。
    std::deque<bool> nis_history_;         ///< 普通目标每次 NIS 是否超阈值的历史，true 表示异常。
    static constexpr size_t nis_window = 15;        ///< 异常统计最多保留的更新次数，不是 15 秒。
    static constexpr size_t nis_threshold_count = 7;///< 当前窗口中达到此异常次数时请求重建。
    static constexpr double nis_threshold = 0.511; ///< 单次 NIS 超限门槛，按原代码保留。

    /**
     * @brief 把不同板数量/几何布局的观测统一成 (残差, 观测雅可比, 观测噪声)。
     * configure 只保存本帧数据，compute 根据保存的数据构造线性化更新项。
     */
    class UpdateStrategy {
    public:
        struct Data {
            const Armors& armors;           ///< 借用本帧观测，生命周期须覆盖策略 configure 调用。
            std::vector<int> match_indices; ///< 模型板索引的副本，不是像素位置或目标类别 ID。
        };
        /// 策略通过 unique_ptr<UpdateStrategy> 管理，虚析构确保派生资源正确释放。
        virtual ~UpdateStrategy() = default;
        /// 读取十维先验与本帧匹配观测；数据数量需满足所选策略的前提。
        virtual void configure(const Eigen::VectorXd& x_pri, const Data& data) = 0;
        /**
         * @return (residual, H, R)：分别为观测残差、对十维状态的雅可比、观测协方差。
         * 单板观测四维，双板八维；位置方位角与板自身朝向 yaw 是不同观测量。
         */
        virtual std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() = 0;
    };

    /// 普通四板目标只观测到一块时的策略，按板索引奇偶选半径/高度。
    class SingleArmorStrategy : public UpdateStrategy {
    public:
        /// 复制先验、armors[0] 和 match_indices[0]，因此两个列表必须非空。
        void configure(const Eigen::VectorXd& x_pri, const Data& data) override;
        /// 返回 4x1 residual、4x10 H、4x4 R；相位间隔为 pi/2，板朝向残差做周期归一化。
        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() override;

    private:
        Eigen::VectorXd x; ///< 本次计算使用的十维先验副本。
        Armor armor;       ///< 本次使用的实测板副本，读取其 abs_pose/abs_ypd/yaw。
        int index;         ///< 对应模型板索引 0~3；index%2 决定 z 与 r 分组。
    };

    /// 前哨站单板观测策略：三种高度、单半径，相邻板相位差 2*pi/3。
    class OutpostStrategy : public UpdateStrategy {
    public:
        /// 复制前哨先验及第一块观测/索引；即使 data 中有多块板，本策略也只取第一块。
        void configure(const Eigen::VectorXd& x_pri, const Data& data) override;
        /// 使用 x(7) 半径、x(4+index) 高度，返回与单板同维度的 residual/H/R。
        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() override;

    private:
        Eigen::VectorXd x; ///< 前哨状态布局的十维先验副本。
        Armor armor;       ///< 当前前哨观测副本。
        int index;         ///< 模型板索引 0~2，分别选择 z1/z2/z3。
    };

    /// 普通目标同时观测到两块板时，将两组四维观测堆叠后一次更新。
    class DualArmorStrategy : public UpdateStrategy {
    public:
        /// 分别复制 armors[0/1] 和 match_indices[0/1]，要求两者数量都至少为 2。
        void configure(const Eigen::VectorXd& x_pri, const Data& data) override;
        /// 返回 8x1 residual、8x10 H、8x8 分块噪声 R，并分别归一化两个板朝向残差。
        std::tuple<Eigen::VectorXd, Eigen::MatrixXd, Eigen::MatrixXd> compute() override;

    private:
        Eigen::VectorXd x; ///< 共享的普通目标先验，两块板都从该同一整车状态预测。
        Armor armor1;      ///< 第一块实测板副本。
        Armor armor2;      ///< 第二块实测板副本。
        int index1;        ///< 第一块观测对应的模型板索引 0~3。
        int index2;        ///< 第二块观测对应的模型板索引 0~3，正常关联应与 index1 不同。
    };

    /// 构造 10x10 Q，仅向 x/vx、y/vy、phase/omega 三个二状态块加入噪声；dd/da 在此硬编码选择。
    Eigen::MatrixXd computeProcessNoise();
    /// outpost 优先选前哨策略；普通目标按匹配数 1/2 选策略，其他数量返回 nullptr。
    std::unique_ptr<UpdateStrategy> createStrategy(const UpdateStrategy::Data& data);
    /// 历史预留状态约束声明；当前 ArmorEKF.cpp 未提供定义，也没有调用，不应认为已施加约束。
    void constrainState(Eigen::VectorXd& state);
};

#endif // ARMOR_EKF_H
