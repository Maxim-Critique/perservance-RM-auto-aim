// BA_Optimizer.cpp
#include "BA_Optimizer.h"

BA_Optimizer::BA_Optimizer(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
    : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {}

double BA_Optimizer::optimizeYaw(const std::vector<cv::Point3f>& object_points,
                                const std::vector<cv::Point2f>& image_points,
                                const cv::Point3d& initial_pose,
                                double initial_yaw) {
    // 初始化参数
    double pose[3] = {initial_pose.x, initial_pose.y, initial_pose.z};
    double yaw[1] = {initial_yaw};
    
    // 构建优化问题
    ceres::Problem problem;
    
    // 添加残差块
    for (size_t i = 0; i < object_points.size(); ++i) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<BA_CostFunction, 2, 3, 1>(
                new BA_CostFunction(object_points[i], image_points[i], 
                                  camera_matrix_, dist_coeffs_));
        problem.AddResidualBlock(cost_function, nullptr, pose, yaw);
    }
    
    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 40;
    options.function_tolerance = 1e-6;
    
    // 运行优化
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    // 返回优化后的yaw角
    return yaw[0];
}