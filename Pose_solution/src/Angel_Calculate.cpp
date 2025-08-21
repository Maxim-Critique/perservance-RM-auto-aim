#include "Angel_Calculate.h"
#include "Base.h"
#include <numeric>
#include <string>
#include "camera_to_imu.h"

PNP::PNP()
{
    
    Intrinsic_Matrix = Mat::zeros(3, 3, CV_32F);
    float tempMatrix[3][3] = 
    {{1730.7    ,       0.4,               731.1},
     {0,                1730.1 ,           559.6}, 
     {0,                0,                 1}};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Intrinsic_Matrix.at<float>(i, j) = tempMatrix[i][j];
        }
    }
    Distortion = Mat::zeros(5, 1, CV_64F);
    Distortion.at<double>(0, 0) = -0.0681 ;
    Distortion.at<double>(1, 0) =  0.1248;
    Distortion.at<double>(2, 0) =  -0.0000634;
    Distortion.at<double>(3, 0) =  0.0005893;
    Distortion.at<double>(4, 0) =  -0.0815;

    rvec = Mat::zeros(3, 1, CV_64F);
    tvec = Mat::zeros(3, 1, CV_64F);
}







Point2d PNP::ProjectTo2D(Point3d point3D) {
    // 创建齐次坐标，确保为 CV_64F 类型
    Mat point3D_homogeneous = (Mat_<double>(4, 1) << point3D.x, point3D.y, point3D.z, 1);

    // 确保 Intrinsic_Matrix 为 CV_64F 类型
    Mat intrinsic;
    Intrinsic_Matrix.convertTo(intrinsic, CV_64F);

    // 进行投影
    Mat projected = intrinsic * point3D_homogeneous.rowRange(0, 3);

    // 齐次坐标归一化
    double w = projected.at<double>(2, 0);
    double x_2D = projected.at<double>(0, 0) / w;
    double y_2D = projected.at<double>(1, 0) / w;

    // 返回二维坐标
    return Point2d(static_cast<float>(x_2D), static_cast<float>(y_2D));
}


void PNP::readConfig(Armor armor) {
    
    double width , height;
    if(armor.type == ArmorType::SMALL){
        width = 0.0725;
        height = 0.0325;
    }
    else{
        width = 0.1175;
        height = 0.034;
    }
    world_points.clear();
    world_points.push_back(Point3f(-width , -height , 0));
    world_points.push_back(Point3f(-width ,  height , 0));
    world_points.push_back(Point3f( width ,  height , 0));
    world_points.push_back(Point3f( width , -height , 0));    
}

void PNP::BuffPnP(vector<Point2f> Buff_points, Point3d &pose, Point3d &Rpose, Point3d &Aimpose, double &roll, double &yaw)
{
    // 0-----------1
    // |           |
    // 3-----------2

    vector<Point3f> obj;
    double width = 0.165;
    double height = 0.03;

    obj.push_back(Point3f(width, -height, 0));
    obj.push_back(Point3f(-width, -height, 0));
    obj.push_back(Point3f(-width, height, 0));
    obj.push_back(Point3f(width, height, 0));

    cv::Mat rvec, tvec;
    cv::solvePnP(obj, Buff_points, Intrinsic_Matrix, Distortion, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 将旋转向量转换为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 10点滑动均值滤波
    static std::deque<double> distance_history;               // 静态队列保存历史数据
    double current_distance = *tvec.ptr<double>(2) - error_x; // 当前测量值

    // 更新历史数据队列
    distance_history.push_back(current_distance);
    if (distance_history.size() > 5)
    {
        distance_history.pop_front(); // 保持队列长度不超过10
    }

    // 计算窗口内平均值
    double sum = 0.0;
    for (double d : distance_history)
    {
        sum += d;
    }
    distance_x = sum / distance_history.size(); // 均值滤波结果

    // 计算输出参数
    pose = {distance_x, -*tvec.ptr<double>(0), -*tvec.ptr<double>(1)};
    pose = C_C.cam2abs(pose);
    roll = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)) + ds.camera_roll_angle;
    yaw = PI / 2 + ds.camera_yaw_angle;

    Rpose = Point3d(
        pose.x - 0.344 * cos(roll) * cos(yaw),
        pose.y - 0.344 * cos(roll) * sin(yaw),
        pose.z - 0.344 * sin(roll)  // 修正：使用roll而不是sl.roll
    );
    
    Aimpose = Point3d(
        pose.x + 0.356 * cos(roll) * cos(yaw),
        pose.y + 0.356 * cos(roll) * sin(yaw),
        pose.z + 0.356 * sin(roll)  // 修正：使用roll而不是sl.roll
    );
}
