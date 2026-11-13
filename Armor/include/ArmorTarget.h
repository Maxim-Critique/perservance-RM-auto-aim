#ifndef ARMOR_TARGET_H
#define ARMOR_TARGET_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <map>
#include <vector>
#include <deque>
#include "ArmorTypes.h"
#include "ArmorEKF.h"
#include "ArmorPredictor.h"
#include "ArmorSolver.h"
using namespace cv;
using namespace std;

/**
 * @file ArmorTarget.h
 * @brief 历史版单目标整车观测封装，串联模型预测、装甲板关联和专用 EKF。
 * 当前 CMake 已排除 ArmorTarget.cpp；现行 AutoArmor 使用 Armor/sentry/tracker 与 target。
 * 本文件 Aim/Match 的十维状态布局与 sentry 模块不同，使用前须区分代码版本。
 */

/**
 * @brief 维护同一类别目标的十维运动状态，而不是完整敌我选择或串口控制器。
 * 构造用一块已解算的 Armor 建立初态；后续输入需在同一世界参考系下，长度为 m、角度为 rad。
 * aim_id 是识别标签，不是能跨同类多车唯一识别的跟踪 UUID。
 */
class Aim {
public:
    /// 为含 Eigen 定长对齐成员的场景保留 Eigen 分配宏，不参与状态估计计算。
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /// 保存 armor.number，按实测 abs_pose/yaw 建立初始状态并构造专用 EKF。
    Aim(Armor armor);
    /**
     * @brief 调用 Resolve 更新整车状态，再生成可供历史 Planner 使用的运动学模型。
     * @param armors 当前目标的已解算观测；不在此函数里做图像检测或 PnP。
     * @param time 帧间隔，s；签名为非常量引用，但当前实现只读取、不修改它。
     * @return 从 target_state 创建的 StandardModel 或 OutpostModel 共享指针。
     * 当前实现已注释掉 visualizeState 调用，不会自动显示俯视窗口。
     */
    std::shared_ptr<KinematicModel> Estimate(const Armors &armors, double &time);
    /**
     * @brief 必要时重建 EKF，再执行预测 -> 模型展开 -> 匹配 -> 观测更新。
     * @param armors 实测板列表；is_lost 分支会直接访问 armors[0]，调用方须保证非空。
     * @param time 从上一状态到本次观测的时间，s，调用方负责时间有效性。
     * 匹配得分上限在当前实现中直接传入 1，并不使用 m_score_tolerance 成员。
     */
    void Resolve(const Armors &armors, double time);
    /**
     * @brief 简单逐行最小值匹配辅助函数，与下方嵌套 Match 类是两种不同接口。
     * @param score 行为观测、列为模型板的代价矩阵；列数应大于 0。
     * @return 每行的最小值列索引；不排除多行选同一列，也不执行阈值剔除。
     * 当前 Resolve 实际调用 m_match.GetMatch，而不是这个逐行版本。
     */
    std::map<int, int> Match(const Eigen::MatrixXd &score);
    /**
     * @brief 按 aim_id 重写 target_state；不在函数内替换 m_model_observer。
     * 普通初始半径 0.25 m；前哨 0.225 m；中心由板位置减去径向偏移，平移/角速度初值为 0。
     * 普通布局 [xc,vx,yc,vy,z1,z2,r1,r2,phase,omega]；前哨第 6 项变 z3、第 7 项为唯一半径。
     */
    void InitEKF(Armor armor);
    /**
     * @brief 计算观测板与预测板之间的匹配代价，使用三维距离及周期归一化的朝向差。
     * @param detect_armors 实测板，主要读取 abs_pose/yaw。
     * @param standard_armors 模型展开的候选板，主要读取 abs_pose/predict_yaw。
     * @return m 行 n 列代价矩阵；空输入时返回相应尺寸的零矩阵，使用方按较小代价匹配。
     * @note 当前 cpp 的三列指标中仅写了第 0、2 列，第 1 列没有赋值却参与熵权计算；
     * 本注释记录现状，不代表该历史评分实现已经过正确性修复。
     */
    Eigen::MatrixXd GetScoreMat(const Armors &detect_armors, const Armors &standard_armors);

    /// 绘制 800x800 的世界系 x/y 俯视观测与模型板，显示 dt（s 转 ms）并调用 imshow/waitKey。
    void visualizeState(const Armors &detected_armors, double dt);

    /// 根据 aim_id 是否等于 outpost 选择三板/四板模型；state 必须匹配相应十维布局。
    std::shared_ptr<KinematicModel> createModel(const Eigen::VectorXd &state) const;

    double m_score_tolerance;                               ///< 名义上的匹配容差；当前 cpp 未初始化或使用，实际 Resolve 使用常数 1。
    std::shared_ptr<ExtendedKalmanFilter> m_model_observer; ///< 专用十维 EKF 的共享指针；失配后由 Resolve 重建。
    Eigen::VectorXd target_state;                           ///< 最近估计的十维整车状态，不是某一块板的 xyz/yaw。
    string aim_id;                                          ///< 目标识别类别字符串，用于模型选择和外层同类别筛选。
private:
    /**
     * @brief 通过枚举行列组合与排列寻找低总代价的非重复匹配，再逐项做阈值过滤。
     * 名字中的 four 为历史命名；实际可处理 m 个模型板，不专指四板或四个观测。
     */
    class Match {
    public:
        /// 当前构造为空，工作容器由 GetMatch 准备，标量依成员初值初始化。
        Match();
        /// 当前析构为空，容器自行释放。
        ~Match();

        /**
         * @brief 枚举模型索引与观测行的组合，在不重复使用列的条件下比较总代价。
         * @param matrix m_obs x n_model 的代价矩阵，按值接收；索引访问要求模型列至少有 m 列。
         * @param score_max 单个最终匹配允许的最大代价，不是匹配数量或总代价上限。
         * @param m 模型装甲板数量（通常 3 或 4），不是实测装甲板数量。
         * @return 观测行 -> 模型板列；未匹配或分数超限的项不在返回 map 中。
         * 实现以对角元素累计生成初始候选，min 没有在每次调用开始清零，
         * 因而此接口说明只描述现有枚举流程，不额外保证跨帧最优匹配性质。
         */
        std::map<int, int> GetMatch(Eigen::MatrixXd matrix, double score_max, int m);

        bool debug = false; ///< 是否向标准输出打印组合、排列、代价和匹配表，打开后输出量较大。

    private:
        /**
         * @brief 回溯枚举 input 中 k 个元素的所有不重复组合，追加进 result。
         * @param input 待选的行索引序列。
         * @param tmp_v 当前递归分支已选元素，调用前通常为空，递归退出时恢复。
         * @param result 组合结果容器；本函数只追加，不负责清空。
         * @param start 本层开始选择的位置，首次调用为 0。
         * @param k 需选的观测行数量，调用方按可用行数与模型板数决定。
         */
        void GetCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v,
                                   std::vector<std::vector<int>> &result, size_t start, size_t k);

        double min = 0, tmp = 0; ///< min 保存当前比较基准/最佳总代价；tmp 是本次候选累加器，在枚举中清零重算。

        std::vector<int> col;    ///< 模型列索引 0~m-1，供列排列枚举。
        std::vector<int> row;    ///< 输入矩阵所有观测行的索引。
        std::vector<int> tmp_v;  ///< 生成行组合时使用的递归临时栈。

        std::vector<std::vector<int>> result;    ///< 观测行的组合 C(n,k)，未包含顺序变化。
        std::vector<std::vector<int>> nAfour;    ///< 每个行组合继续排列得到的有序行候选，不固定为四个。
        std::vector<std::vector<int>> fourAfour; ///< 模型列的全排列，即 m! 个列顺序候选。

        std::map<int, int> row_col; ///< 当前结果表：key 是观测行，value 是模型板列；最终输出前删除 -1 与超阈值项。

    } m_match; ///< 当前 Aim 独占的非重复关联枚举器；不是上方公有 Match() 逐行辅助函数。
};

#endif // ARMOR_TARGET_H
