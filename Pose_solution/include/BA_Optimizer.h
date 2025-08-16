// BA_Optimizer.h
#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "Base.h"

class BA_Optimizer {
public:
    BA_Optimizer(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);
    
    // 优化函数
    double optimizeYaw(const std::vector<cv::Point3f>& object_points,
                      const std::vector<cv::Point2f>& image_points,
                      const cv::Point3d& initial_pose,
                      double initial_yaw);
    
private:
    // Ceres代价函数
    struct BA_CostFunction {
        BA_CostFunction(const cv::Point3f& object_point, 
                       const cv::Point2f& image_point,
                       const cv::Mat& camera_matrix,
                       const cv::Mat& dist_coeffs)
            : object_point_(object_point), 
              image_point_(image_point),
              camera_matrix_(camera_matrix),
              dist_coeffs_(dist_coeffs) {}
        
        template <typename T>
        bool operator()(const T* const pose, const T* const yaw, T* residuals) const {
            // 1. 将3D点从装甲板坐标系转换到相机坐标系
            T point[3];
            point[0] = T(object_point_.x);
            point[1] = T(object_point_.y);
            point[2] = T(object_point_.z);
            
            // 2. 应用Yaw旋转
            T cos_yaw = ceres::cos(yaw[0]);
            T sin_yaw = ceres::sin(yaw[0]);
            
            T rotated_point[3];
            rotated_point[0] = cos_yaw * point[0] - sin_yaw * point[1];
            rotated_point[1] = sin_yaw * point[0] + cos_yaw * point[1];
            rotated_point[2] = point[2];
            
            // 3. 加上平移
            T camera_point[3];
            camera_point[0] = rotated_point[0] + T(pose[0]);
            camera_point[1] = rotated_point[1] + T(pose[1]);
            camera_point[2] = rotated_point[2] + T(pose[2]);
            
            // 4. 投影到图像平面
            T fx = T(camera_matrix_.at<double>(0, 0));
            T fy = T(camera_matrix_.at<double>(1, 1));
            T cx = T(camera_matrix_.at<double>(0, 2));
            T cy = T(camera_matrix_.at<double>(1, 2));
            
            T xp = camera_point[0] / camera_point[2];
            T yp = camera_point[1] / camera_point[2];
            
            // 5. 计算残差
            residuals[0] = T(image_point_.x) - (fx * xp + cx);
            residuals[1] = T(image_point_.y) - (fy * yp + cy);
            
            return true;
        }
        
    private:
        cv::Point3f object_point_;
        cv::Point2f image_point_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
    };
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};