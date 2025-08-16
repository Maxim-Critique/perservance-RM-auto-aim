#include "AIM.h"

aim::aim(Armor armor){
    target_state = Eigen::VectorXd::Zero(10);
    translation_state = Eigen::VectorXd::Zero(7);
    InitEKF(armor);
    m_model_observer = std::make_shared<ExtendedKalmanFilter>(target_state);
    m_model_translation = std::make_shared<CAEKF>(translation_state);
    readYAML(Yaml);
    center_tracked = false;
}

void aim::Shoot(Armor &armor ,Mat &img ,double time){
    m_status = std::make_shared<StandardModel>(target_state);         
    m_model_observer->Updatedt(time);
    m_status = Resolve(armor);

    Armor ui_armor = m_status->getArmors(0)[get_id];
    armor.cam_pose = C_C.abs2cam(ui_armor.abs_pose);
    
    //绘制图像
    if(abs(m_status->palstance)<1.8){
        control_model = 1; 
        m_model_translation->predict(time);
        translation_state = m_model_translation->update(armor.abs_pose);
        double hit_time = GetDistance(Point3d(m_status->center.x,m_status->center.y,(m_status->height[0]+m_status->height[1])/2))/ (bullet_speed) + predict_time;
        armor.abs_pose = m_model_translation->GetArmor(hit_time);
    }else{
        // 判断是否锁中心
        if(center_tracked == false){
            
            if ((abs(m_status->palstance) - switch_threshold) > palstance_threshold)
            {
                center_tracked = true;
            }
        }  
        else
        {
            if (abs(abs(m_status->palstance) + switch_threshold) < palstance_threshold){
                center_tracked = false;
            }     
        }
        if(center_tracked){
            // 锁中心，附近是否有版
            double center_hit_time = GetDistance(Point3d(m_status->center.x,m_status->center.y,(m_status->height[0]+m_status->height[1])/2))/(bullet_speed) + predict_time;           
            armor = m_status->getFacingArmor(center_hit_time,this->last_index,this->frames_since_last_switch,this->predict_point);    
            control_model = 3;
        }
        else{       
            // 不锁中心
            double hit_time = GetDistance(Point3d(m_status->center.x,m_status->center.y,(m_status->height[0]+m_status->height[1])/2))/ (bullet_speed) + predict_time;
            armor = m_status->getClosestArmor(hit_time,this->last_index,this->frames_since_last_switch);
            control_model = 2;
        }
        
    }
    armor.cam_pose = C_C.abs2cam(armor.abs_pose);
    ui = pnp_armor.ProjectTo2D(Point3d(-armor.cam_pose.y,-armor.cam_pose.z,armor.cam_pose.x));
    if(show){
        DraftingSoftware(img);
    }
    
} 

double aim::Homogeneity(double new_value) {
    this->sum -= this->buffer[this->index];      // 减去即将被覆盖的值
    this->sum += new_value;          // 加上新值
    this->buffer[this->index] = new_value; // 存储新值
    this->index = (this->index + 1) % 40;   // 环形索引
    return this->sum / 40;            // 直接返回平均值
}

std::shared_ptr<KinematicModel> aim::Resolve(const Armor &armor)
{   

    target_state = m_model_observer->predict(target_state);
    std::shared_ptr<KinematicModel> prior_status = std::make_shared<StandardModel>(target_state);
    Eigen::MatrixXd score = GetScoreMat(armor, prior_status->getArmors(0));
    std::map<int, int> armor_match = m_match.GetMatch(score,1, 4);

    auto first_element = armor_match.begin();
    int num = armor_match.begin()->first;
    VectorXd z = Eigen::VectorXd::Zero(4, 1);
    UpdatePrepare(z,armor);
    target_state = m_model_observer->update(z,armor_match[num]);
    
    Eigen::VectorXd after_change = target_state;  
    after_change[9] = Homogeneity(after_change[9]);
    std::shared_ptr<KinematicModel> posterior_status = std::make_shared<StandardModel>(after_change);
    get_id = armor_match[num];
    return posterior_status;

}

void aim::InitEKF (Armor armor){
    //出是半径设置为0.25
    target_state << armor.abs_pose.x - 0.25 * cos(armor.yaw) ,   //xc
            0 ,                                     //vx
            armor.abs_pose.y - 0.25 * sin(armor.yaw) ,   //yc
            0 ,                                     //vy
            armor.abs_pose.z ,                           //z1
            armor.abs_pose.z ,                           //z2        
            0.25 ,                                  //r1
            0.25 ,                                  //r2
            armor.yaw ,                             //yaw
            0 ;                                     //vyaw
    translation_state << armor.abs_pose.x,
                         0,
                         0,
                         armor.abs_pose.y,
                         0,
                         0,
                         armor.abs_pose.z;
}

void aim::UpdatePrepare (VectorXd &X_,Armor armor){
    X_ <<  armor.abs_pose.x,
           armor.abs_pose.y,
           armor.abs_pose.z,
           _std_radian(armor.yaw);
}





Eigen::MatrixXd aim::GetScoreMat(const Armor &detect_armor, const Armors &standard_armors)//计算分数匹配装甲板
{
    if (standard_armors.empty()) {
        return Eigen::MatrixXd::Zero(0,0);
    }
    int m = 1;
    int n = standard_armors.size();
    // 计算两组装甲板之间的坐标差和角度差两个负向指标
    Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score;
    negative_score.resize(m * n, 2);
    
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            negative_score(i * n + j, 0) = GetDistance(detect_armor.abs_pose - standard_armors[j].abs_pose);
            negative_score(i * n + j, 1) = abs(_std_radian(detect_armor.yaw - standard_armors[j].pridict_yaw));
        }
    }

    // 数据标准化
    Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score;
    regular_score.resize(m * n, 2);
    for (int i = 0; i < regular_score.rows(); i++)
    {
        regular_score(i, 0) = (negative_score.col(0).maxCoeff() - negative_score(i, 0)) / (negative_score.col(0).maxCoeff() - negative_score.col(0).minCoeff());
        regular_score(i, 1) = (negative_score.col(1).maxCoeff() - negative_score(i, 1)) / (negative_score.col(1).maxCoeff() - negative_score.col(1).minCoeff());
    }

    // 计算样本值占指标的比重
    Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight;
    score_weight.resize(m * n, 2);
    Eigen::VectorXd col_sum = regular_score.colwise().sum();
    for (int i = 0; i < score_weight.rows(); i++)
    {
        score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
        score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
    }

    // 计算每项指标的熵值
    Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
    for (int i = 0; i < score_weight.rows(); i++)
    {
        if (score_weight(i, 0) != 0)
            entropy(0) -= score_weight(i, 0) * log(score_weight(i, 0));
        if (score_weight(i, 1) != 0)
            entropy(1) -= score_weight(i, 1) * log(score_weight(i, 1));
    }
    entropy /= log(score_weight.rows());

    // 计算权重
    Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy) / (2 - entropy.sum());

    // 计算匹配得分矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> score;
    score.resize(m, n);
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (i < 1 && j < standard_armors.size())
            {
                score(i, j) = negative_score.row(i * standard_armors.size() + j) * weight;
            }
        }
    }

    return score;
}

std::map<int, int> aim::Match(const Eigen::MatrixXd &score)
{
    std::map<int, int> match;
    for (int i = 0; i < score.rows(); i++)
    {
        int index;
        score.row(i).minCoeff(&index);
        match[i] = index;
        // std::cout << "(" << i << ", " << match[i] << ") ";
    }
    // std::cout << std::endl;
    return match;
}

aim::Match::Match(){}
aim::Match::~Match(){}

std::map<int, int> aim::Match::GetMatch(Eigen::MatrixXd matrix, double score_max, int m)
{
    /**
     * @brief 初始化各个变量
     *
     */
    /// 初始化最终结果
    this->row_col.clear();
    for (int i = 0; i < matrix.rows(); i++)
    {
        // 值为-1表示不选择该行中的任何数
        this->row_col[i] = -1;
    }
    /// 初始化min
    for (int i = 0; i < matrix.rows() && i < m; i++)
    {
        this->min += matrix(i, i);
        this->row_col[i] = i;
    }
    /// 初始化行数列
    this->row.clear();
    for (int i = 0; i < matrix.rows(); i++)
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
        this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
    else
        this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, m);

    #ifdef match_debug
    // 输出C(n,m)计算结果
    // 红色输出
    std::cout << _red("C(n,m)生成结果") << std::endl;
    for (auto it = this->result.begin(); it != this->result.end(); it++)
    {
        for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
        {
            std::cout << *it2 << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
    #endif

    /// 使用上一步的结果计算A(m,m)，最终得到A(n,m)
    for (auto it = this->result.begin(); it != this->result.end(); it++)
    {
        do
        {
            this->nAfour.emplace_back(*it);
        } while (next_permutation(it->begin(), it->end())); // stl自带全排列函数
    }

    #ifdef match_debug
    // 输出A(n,m)计算结果
    // 绿色输出
    std::cout << _green("A(n,m)生成结果") << std::endl;
    for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
    {
        for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
        {
            std::cout << *it2 << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
    #endif

    /**
     * @brief 用A(m,m)求出可能选择的所有列的组合
     *
     */
    /// 计算A(m,m)
    do
    {
        fourAfour.push_back(col);
    } while (next_permutation(col.begin(), col.end())); // stl自带全排列函数

    #ifdef match_debug
    // 输出A(m,m)计算结果
    // 蓝色输出
    std::cout << _blue("A(m,m)生成结果") << std::endl;
    for (auto it = this->fourAfour.begin(); it != this->fourAfour.end(); it++)
    {
        for (std::vector<int>::iterator it2 = it->begin(); it2 != it->end(); it2++)
        {
            std::cout << *it2 << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
    #endif

    /**
     * @brief 用A(n,m)和A(m,m)求出可能选择的所有行列的组合的结果
     *
     */
    for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
    {
        for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++)
        {
            // 对于每一种组合，都求一遍它们的值，与min比较，如果小于min，则更新min
            this->tmp = 0;
            for (int i = 0; i < it->size(); i++)
                this->tmp += matrix((*it)[i], (*it2)[i]);
            if (this->tmp < this->min)
            {
                this->min = this->tmp;
                // 重新初始化map
                this->row_col.clear();
                // 每选择矩阵中的一个数时，就将该位置记录下来存到map中
                for (int i = 0; i < it->size(); i++)
                {
                    this->row_col[it->at(i)] = it2->at(i);
                }
            }
        }
    }

    #ifdef match_debug
    // 输出删除前的map
    // 紫色输出
    std::cout << _purple("删除-1前的map结果") << std::endl;
    for (auto it = this->row_col.begin(); it != this->row_col.end(); it++)
    {
        std::cout << it->first << " " << it->second << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
    #endif

    /// 删除-1的键值对
    for (auto it = this->row_col.begin(); it != this->row_col.end();)
    {
        if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max))
        {
            it = this->row_col.erase(it);
        }
        else
            ++it;
    }

    #ifdef match_debug
    // 输出min的值和删除后的map
    // 白色输出
    std::cout << _white("min和删除-1后的map结果") << std::endl;
    std::cout << "min = " << min << std::endl;
    for (auto it = this->row_col.begin(); it != this->row_col.end(); ++it)
    {
        std::cout << it->first << " " << it->second << " " << matrix(it->first, it->second) << std::endl;
    }
    std::cout << "-------------------------------" << std::endl;
    #endif

    /// 输出矩阵
    #ifdef match_debug
    std::cout << "matrix:" << std::endl;
    for (int i = 0; i < matrix.rows(); i++)
    {
        for (int j = 0; j < matrix.cols(); j++)
        {
            std::cout << matrix(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    #endif

    return this->row_col;
}

void aim::Match::getCombinationsNumbers(std::vector<int> &input, std::vector<int> &tmp_v, std::vector<std::vector<int>> &result, int start, int k)
{
    for (int i = start; i < input.size(); i++)
    {
        tmp_v.emplace_back(input[i]);
        if (tmp_v.size() == k)
        {
            result.emplace_back(tmp_v);
            tmp_v.pop_back();
            continue;
        }
        // 使用递归计算
        getCombinationsNumbers(input, tmp_v, result, i + 1, k);
        tmp_v.pop_back();
    }
}

void aim::DraftingSoftware(Mat &img){

    // Point3d pose = C_C.abs2cam(Point3d(m_status->center.x,m_status->center.y,(m_status->height[0]+m_status->height[1])/2));
    // Point2d zi = pnp_armor.ProjectTo2D(Point3d(-pose.y,-pose.z,pose.x + error_x));
    // cv::circle(img, zi, 2, cv::Scalar(0, 0, 255), -1);  // 红色：BGR(0, 0, 255)
    cv::circle(img, ui, 2, cv::Scalar(0, 0, 255), -1);  // 红色：BGR(0, 0, 255)

}

YAML::Node aim::readYAML(const std::string& filename){
    try {
        YAML::Node config = YAML::LoadFile(filename);
        predict_time = config["AIM"]["predict_time"].as<double>();
        palstance_threshold = config["AIM"]["palstance_threshold"].as<double>();
        switch_threshold = config["AIM"]["switch_threshold"].as<double>();
        show = config["CAMERA"]["show"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Error: " + std::string(e.what()));
    }
    return YAML::LoadFile(filename);
}