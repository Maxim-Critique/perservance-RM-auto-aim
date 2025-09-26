#include "AIM.h"

Aim::Aim(Armor armor){
    try {
        YAML::Node config = YAML::LoadFile(YamlPath);
        predict_time = config["AIM"]["predict_time"].as<double>();
        show = config["CAMERA"]["show"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()) + "Aim");
    }
    target_state = Eigen::VectorXd::Zero(10);
    translation_state = Eigen::VectorXd::Zero(7);
    InitEKF(armor);
    m_model_observer = std::make_shared<ExtendedKalmanFilter>(target_state);
    m_model_translation = std::make_shared<CAEKF>(translation_state);
    mpc = std::make_unique<MPC>();
    ca_yaw = armor.yaw;
    control_model = 1;
    ca_try_times = 0;
}

Gimbal_Angle Aim::Shoot(const Armors &armors, Mat &img, double &time){
    
    m_status = std::make_shared<StandardModel>(target_state);         
    // 反小陀螺模型
    m_status = Resolve(armors, time);
    ChooseFireModel();
    double hit_time = cv::norm(Point3d(m_status->center.x, m_status->center.y, (m_status->height[0] + m_status->height[1]) / 2)) / (bullet_speed) + predict_time;
    Armor armor;
    switch (control_model) {
        case 1:  // 低速模式
            // 平移模型
            SloveCA(armors, time);
            armor.abs_pose = m_model_translation->GetArmor(hit_time);
            break;
            
        case 2:  // 不锁中心模式
            armor = m_status->GetClosestArmor(hit_time, this->last_index, this->frames_since_last_switch);
            break;
            
        case 3:  // 锁中心模式
            armor = m_status->GetFacingArmor(hit_time, this->last_index, 
                                            this->frames_since_last_switch, this->predict_point);
            break;
            
        default:
            cout << "瞄准模式选择ERROR" << endl;
            break;
    }
    armor.cam_pose = C_C.abs2cam(armor.abs_pose);
    // 简单方法
    // Gimbal_Angle gimbal_angle = C_C.GetAngle(armor.cam_pose);
    // MPC方法
    Gimbal_Angle gimbal_angle = mpc->TrajectoryPlanning(*m_status);
    return gimbal_angle;
} 

void Aim::SloveCA(const Armors armors, double time){
    Armor ca_armor;
    for(auto armor : armors){
        if(abs(armor.yaw - ca_yaw) < D2R(30)){
            ca_yaw = armor.yaw;
            ca_armor = armor;
        }
    } 
    if(ca_armor.points.empty()){
        ca_try_times++;
    } else {
        m_model_translation->predict(time);
        m_model_translation->update(ca_armor.abs_pose);
    }
    if(ca_try_times > 5){
        InitEKF(armors[0]);
        m_model_translation = std::make_shared<CAEKF>(translation_state);
    }
}

double Aim::Homogeneity(double new_value) {
    if (buffer.size() >= window_size) {
        sum -= buffer.front();
        buffer.pop_front();
    }
    buffer.push_back(new_value);
    sum += new_value;
    return sum / buffer.size();
}

void Aim::DraftingSoftware(Mat &img) {
    // 实现三维重投影绘制
    // TODO: 根据实际需求实现
}
int PAJDS = 0;
std::shared_ptr<KinematicModel> Aim::Resolve(const Armors &armors, double time)
{   
    if(m_model_observer->is_lost){
        cout<<"   触发nis重置   "<<" 次数 "<<PAJDS<<endl;
        InitEKF(armors[0]);
        m_model_observer = std::make_shared<ExtendedKalmanFilter>(target_state);
        PAJDS++;
    }
    target_state = m_model_observer->predict(time);
    std::shared_ptr<KinematicModel> prior_status = std::make_shared<StandardModel>(target_state);
    Eigen::MatrixXd score = GetScoreMat(armors, prior_status->GetArmors(0));
    std::map<int, int> armor_match = m_match.GetMatch(score, 1, 4);
    vector<int> match;
    for (const auto& [key, value] : armor_match) { 
        match.push_back(value);
    }
    target_state = m_model_observer->update(armors, match);
    Eigen::VectorXd after_change = target_state;  
    after_change[9] = Homogeneity(after_change[9]);
    std::shared_ptr<KinematicModel> posterior_status = std::make_shared<StandardModel>(after_change);
    return posterior_status;
}

void Aim::InitEKF(Armor armor){
    // 初始半径设置为0.25
    target_state << armor.abs_pose.x - 0.25 * cos(armor.yaw),   // xc
                   0,                                           // vx
                   armor.abs_pose.y - 0.25 * sin(armor.yaw),    // yc
                   0,                                           // vy
                   armor.abs_pose.z,                            // z1
                   armor.abs_pose.z,                            // z2        
                   0.25,                                        // r1
                   0.25,                                        // r2
                   armor.yaw,                                   // yaw
                   0;                                           // vyaw
    translation_state << armor.abs_pose.x,
                         0,
                         0,
                         armor.abs_pose.y,
                         0,
                         0,
                         armor.abs_pose.z;
}

void Aim::ChooseFireModel() {
    double palstance = abs(m_status->palstance);
    switch (control_model) {
        case 1:  // 低速模式 → 切换到不锁中心模式的条件
            if (palstance > PI / 3) {
                control_model = 2;
            }
            break;
            
        case 2:  // 不锁中心模式 → 切换条件
            if (palstance < PI / 6) {
                control_model = 1;  // 切回低速模式
            } else if (palstance > 2.5 * PI) {
                control_model = 3;  // 切换到锁中心模式
            }
            break;
            
        case 3:  // 锁中心模式 → 切回不锁中心模式的条件
            if (palstance < 1.5 * PI) {
                control_model = 2;
            }
            break;
            
        default:
            // 可选：添加错误处理或日志记录
            break;
    }
}

Eigen::MatrixXd Aim::GetScoreMat(const Armors &detect_armors, const Armors &standard_armors)
{
    const Eigen::Index m = detect_armors.size();
    const Eigen::Index n = standard_armors.size();
    if (m == 0 || n == 0)
        return Eigen::MatrixXd::Zero(m, n);

    // 1. 计算负向指标矩阵 (m×n × 3)
    Eigen::Matrix<double, Eigen::Dynamic, 3> negative_score(m * n, 3);
    for (Eigen::Index i = 0; i < m; ++i) {
        for (Eigen::Index j = 0; j < n; ++j) {
            const auto &detect = detect_armors[i].abs_pose;
            const auto &standard = standard_armors[j].abs_pose;
            // xy平面距离
            negative_score(i * n + j, 0) = std::hypot(detect.x - standard.x, detect.y - standard.y);
            // z轴距离
            negative_score(i * n + j, 1) = std::abs(detect.z - standard.z);
            // 角度差
            negative_score(i * n + j, 2) = std::abs(_std_radian(detect_armors[i].yaw - standard_armors[j].predict_yaw));
        }
    }

    // 2. 标准化 (利用广播和逐元素运算)
    const auto max_vals = negative_score.colwise().maxCoeff();
    const auto min_vals = negative_score.colwise().minCoeff();
    const auto denoms = (max_vals - min_vals).cwiseMax(1e-10);  // 防止除以零
    const auto regular_score = (max_vals.matrix().replicate(m * n, 1) - negative_score).array().rowwise() / denoms.array();

    // 3. 计算权重 (熵权法)
    const auto col_sums = regular_score.colwise().sum().cwiseMax(1e-10);  // 防止除以零
    const auto score_weight = regular_score.array().rowwise() / col_sums.array();

    Eigen::Vector3d entropy = Eigen::Vector3d::Zero();
    for (Eigen::Index i = 0; i < score_weight.rows(); ++i) {
        for (Eigen::Index j = 0; j < 3; ++j) {
            const double val = score_weight(i, j);
            if (val > 0)
                entropy(j) -= val * std::log(val);
        }
    }
    entropy /= std::log(score_weight.rows());

    // 4. 计算最终权重
    const double denom = std::max(3.0 - entropy.sum(), 1e-10);  // 防止除以零
    const Eigen::Vector3d weight = (Eigen::Vector3d::Ones() - entropy) / denom;

    // 5. 计算匹配得分矩阵 (m×n)
    Eigen::MatrixXd score(m, n);
    for (Eigen::Index i = 0; i < m; ++i) {
        for (Eigen::Index j = 0; j < n; ++j) {
            score(i, j) = negative_score.row(i * n + j).dot(weight);
        }
    }

    return score;
}

std::map<int, int> Aim::Match(const Eigen::MatrixXd &score)
{
    std::map<int, int> match;
    for (Eigen::Index i = 0; i < score.rows(); i++)
    {
        Eigen::Index index;
        score.row(i).minCoeff(&index);
        match[i] = index;
    }
    return match;
}

Aim::Match::Match() {}
Aim::Match::~Match() {}

std::map<int, int> Aim::Match::GetMatch(Eigen::MatrixXd matrix, double score_max, int m)
{
    /**
        * @brief 初始化各个变量
        *
        */
    /// 初始化最终结果
    this->row_col.clear();
    for (Eigen::Index i = 0; i < matrix.rows(); i++)
    {
        // 值为-1表示不选择该行中的任何数
        this->row_col[i] = -1;
    }
    /// 初始化min
    for (Eigen::Index i = 0; i < matrix.rows() && i < m; i++)
    {
        this->min += matrix(i, i);
        this->row_col[i] = i;
    }
    /// 初始化行数列
    this->row.clear();
    for (Eigen::Index i = 0; i < matrix.rows(); i++)
    {
        // 有多少行，行数列就有多少个数
        this->row.emplace_back(i);
    }
    /// 初始化列数列，n块装甲板，因此列为n
    this->col.clear();
    for (int i = 0; i < m; i++)
    {
        this->col.emplace_back(i);
    }

    this->tmp_v.clear();
    this->result.clear();
    this->nAfour.clear();
    this->fourAfour.clear();

    /**
        * @brief 用A(n,m)求出可能选择的所有行的组合
        * 因为C(n,m) * A(m,m) = A(n,m)，所以先计算C(n,m)，再计算A(m,m)，最后将结果存到result中
        *
        */
    /// 计算C(n,m)
    if (matrix.rows() <= m)
        this->GetCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
    else
        this->GetCombinationsNumbers(this->row, this->tmp_v, this->result, 0, m);
    
    if (this->debug) {
        // 输出C(n,m)计算结果
        std::cout << "C(n,m)生成结果" << std::endl;
        for (auto it = this->result.begin(); it != this->result.end(); it++) {
            for (auto it2 = it->begin(); it2 != it->end(); it2++) {
                std::cout << *it2 << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
    }
    
    /// 使用上一步的结果计算A(m,m)，最终得到A(n,m)
    for (auto it = this->result.begin(); it != this->result.end(); it++) {
        do {
            this->nAfour.emplace_back(*it);
        } while (next_permutation(it->begin(), it->end())); // stl自带全排列函数
    }
    
    if (this->debug) {
        // 输出A(n,m)计算结果
        std::cout << "A(n,m)生成结果" << std::endl;
        for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++) {
            for (auto it2 = it->begin(); it2 != it->end(); it2++) {
                std::cout << *it2 << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
    }

    /**
        * @brief 用A(m,m)求出可能选择的所有列的组合
        *
        */
    /// 计算A(m,m)
    do {
        fourAfour.push_back(col);
    } while (next_permutation(col.begin(), col.end())); // stl自带全排列函数
    
    if (this->debug) {
        // 输出A(m,m)计算结果
        std::cout << "A(m,m)生成结果" << std::endl;
        for (auto it = this->fourAfour.begin(); it != this->fourAfour.end(); it++) {
            for (auto it2 = it->begin(); it2 != it->end(); it2++) {
                std::cout << *it2 << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
    }

    /**
        * @brief 用A(n,m)和A(m,m)求出可能选择的所有行列的组合的结果
        *
        */
    for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++) {
        for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++) {
            // 对于每一种组合，都求一遍它们的值，与min比较，如果小于min，则更新min
            this->tmp = 0;
            for (size_t i = 0; i < it->size(); i++) {
                this->tmp += matrix((*it)[i], (*it2)[i]);
            }
            if (this->tmp < this->min) {
                this->min = this->tmp;
                // 重新初始化map
                this->row_col.clear();
                // 每选择矩阵中的一个数时，就将该位置记录下来存到map中
                for (size_t i = 0; i < it->size(); i++) {
                    this->row_col[it->at(i)] = it2->at(i);
                }
            }
        }
    }
    
    if (this->debug) {
        // 输出删除前的map
        std::cout << "删除-1前的map结果" << std::endl;
        for (auto it = this->row_col.begin(); it != this->row_col.end(); it++) {
            std::cout << it->first << " " << it->second << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
    }
    
    /// 删除-1的键值对
    for (auto it = this->row_col.begin(); it != this->row_col.end();) {
        if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max)) {
            it = this->row_col.erase(it);
        } else {
            ++it;
        }
    }
    
    if (this->debug) {
        // 输出min的值和删除后的map
        std::cout << "min和删除-1后的map结果" << std::endl;
        std::cout << "min = " << min << std::endl;
        for (auto it = this->row_col.begin(); it != this->row_col.end(); ++it) {
            std::cout << it->first << " " << it->second << " " << matrix(it->first, it->second) << std::endl;
        }
        std::cout << "-------------------------------" << std::endl;
    }
    
    /// 输出矩阵
    if (this->debug) {
        std::cout << "matrix:" << std::endl;
        for (Eigen::Index i = 0; i < matrix.rows(); i++) {
            for (Eigen::Index j = 0; j < matrix.cols(); j++) {
                std::cout << matrix(i, j) << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    
    return this->row_col;
}

void Aim::Match::GetCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, 
                                       std::vector<std::vector<int>> &result, size_t start, size_t k)
{
    for (size_t i = start; i < input.size(); i++) {
        tmp_v.emplace_back(input[i]);
        if (tmp_v.size() == k) {
            result.emplace_back(tmp_v);
            tmp_v.pop_back();
            continue;
        }
        // 使用递归计算
        GetCombinationsNumbers(input, tmp_v, result, i + 1, k);
        tmp_v.pop_back();
    }
}