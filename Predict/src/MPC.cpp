#include "MPC.hpp"

MPC::MPC()
{
    m_process_error_weight << 10000 ,10000;
    m_target_error_weight  << 10000 ,10000;
    m_process_control_weight << 10  ,   20;
    m_dt = 0.01;
    m_step = 5 ;
    setMPCMatrix();
    m_debug = 0;
    
}
MPC::~MPC()
{}

void MPC::setMPCMatrix()
{
    // 初始化系统参数
    m_A.resize(_D_STATE_, _D_STATE_);
    m_B.resize(_D_STATE_, _D_CTRL_);
    m_Q.resize(_D_STATE_, _D_STATE_);
    m_R.resize(_D_CTRL_, _D_CTRL_);
    m_F.resize(_D_STATE_, _D_STATE_);

    m_A <<  1, 0,
            0, 1;
    m_B << m_dt,     0,
              0,  m_dt;

    m_Q = m_process_error_weight.asDiagonal();
    m_R = m_process_control_weight.asDiagonal();
    m_F = m_target_error_weight.asDiagonal();


    // 初始化中间量
    Eigen::MatrixXd M;
    Eigen::MatrixXd C;
    M.resize((m_step + 1) * _D_STATE_, _D_STATE_);
    C.resize((m_step + 1) * _D_STATE_, m_step * _D_STATE_);
    M.setZero();
    C.setZero();

    Eigen::MatrixXd tmp = Eigen::Matrix<double, _D_STATE_, _D_STATE_>::Identity();
    M.block<_D_STATE_, _D_STATE_>(0, 0) = Eigen::Matrix<double, _D_STATE_, _D_STATE_>::Identity();
    for (int i = 1; i <= m_step; i++)
    {
        C.block<_D_STATE_, _D_CTRL_>(i * _D_STATE_, 0) = tmp * m_B;
        for (int j = 1; j < m_step; j++)
        {
            C.block<_D_STATE_, _D_CTRL_>(i * _D_STATE_, j * _D_CTRL_) = C.block<_D_STATE_, _D_CTRL_>((i - 1) * _D_STATE_, (j - 1) * _D_CTRL_);
        }
        tmp *= m_A;
        M.block<_D_STATE_, _D_STATE_>(i * _D_STATE_, 0) = tmp;
    }

    
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Q_.resize((m_step + 1) *_D_STATE_, (m_step + 1) * _D_STATE_);
    R_.resize(m_step *_D_CTRL_, m_step * _D_CTRL_);
    Q_.setZero();
    R_.setZero();

    for (int i = 0; i < m_step; i++)
    {
        Q_.block<_D_STATE_, _D_STATE_>(i * _D_STATE_, i * _D_STATE_) = m_Q;
        R_.block<_D_CTRL_, _D_CTRL_>(i * _D_CTRL_, i * _D_CTRL_) = m_R;
    }
    Q_.block<_D_STATE_, _D_STATE_>(m_step * _D_STATE_, m_step * _D_STATE_) = m_F;

    

    // 计算损失函数系数
    m_H = C.transpose() * Q_ * C + R_;
    m_E = C.transpose() * Q_ * M;
    m_Ed = C.transpose() * Q_;

       
    // std::cout << "[MPC] H: " << std::endl << m_H << std::endl;
    // std::cout << "[MPC] E: " << std::endl << m_E << std::endl;
    // std::cout << "[MPC] Ed: " << std::endl << m_Ed << std::endl;
        
}

Point2d MPC::getGimbalSpeed(vector<Point2d>P_Y)
{
    // std::cout<<P_Y[0]*180/PI<<std::endl;
    // 根据运动模型预测控制目标
    Eigen::MatrixXd predict_ctrl_target = getPredictControlTarget(P_Y);
    // predict_ctrl_target.conservativeResize(predict_ctrl_target.rows() * predict_ctrl_target.cols(), 1);
    predict_ctrl_target.resize(predict_ctrl_target.rows() * predict_ctrl_target.cols(), 1);
    // std::cout << "[MPC] Predict_Ctrl_Target: " << std::endl << predict_ctrl_target << std::endl;

    // 求解最优控制
    Eigen::Matrix<double, 2, 1> cur_state;
    cur_state << 0,0;
    // cur_state << ds.camera_pit_angle, ds.camera_yaw_angle;
    Eigen::MatrixXd U = solveQuadProg(m_H, m_E * cur_state - m_Ed * predict_ctrl_target);
    Point2d target_ctrl;
    target_ctrl.x=U(0, 0),
    target_ctrl.y=U(1, 0);

    return target_ctrl;
}

Eigen::Matrix<double, _D_STATE_,6> MPC::getPredictControlTarget(vector<Point2d>P_Y)
{
    // 控制目标
    Eigen::MatrixXd predict_ctrl_target;
    predict_ctrl_target.resize(_D_STATE_, m_step + 1);
    predict_ctrl_target.setZero();

    for (int i = 0; i <= m_step; i++)
    {
        predict_ctrl_target(0, i) = P_Y[i].x;
        predict_ctrl_target(1, i) = P_Y[i].y;
    }
    return predict_ctrl_target;
}

Eigen::MatrixXd MPC::solveQuadProg(Eigen::MatrixXd H, Eigen::MatrixXd f)
{
    // 迭代中间量，在循环中可以看作是上一次迭代的结果
    Eigen::MatrixXd x0;
    Eigen::MatrixXd r0;
    Eigen::MatrixXd p0;
    x0 = f;
    x0.setZero();
    r0 = H * x0 + f;
    p0 = r0;

    // 迭代更新量
    Eigen::MatrixXd x;
    Eigen::MatrixXd r;
    Eigen::MatrixXd p;
    Eigen::MatrixXd alpha;
    Eigen::MatrixXd beta;

    for (int i = 0; i < 1000; i++)
    {
        alpha = (r0.transpose() * r0) * (p0.transpose() * H * p0).inverse();
        x = x0 - alpha(0, 0) * p0;
        r = r0 - alpha(0, 0) * H * p0;
        beta = (r.transpose() * r) * (r0.transpose() * r0).inverse();
        p = r + beta(0, 0) * p0;

        if ((x - x0).norm() < 1e-8)
        {
            return x;
        }

        x0 = x;
        r0 = r;
        p0 = p;
        
    }
    std::cout<<"MPC拟合失败"<<std::endl;
    return x;
}



