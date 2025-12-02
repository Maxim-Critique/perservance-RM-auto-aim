#include "Angle_Calculate.h"
#include "Base.h"
#include <numeric>
#include <string>
#include "Coordinate.h"



const Mat PNP::Intrinsic_Matrix = []() {
    Mat matrix = Mat::zeros(3, 3, CV_32F);
    float tempMatrix[3][3] = {
        {1730.7f, 0.4f, 731.1f},
        {0, 1730.1f, 559.6f},
        {0, 0, 1}
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            matrix.at<float>(i, j) = tempMatrix[i][j];
        }
    }
    return matrix;
}();

const Mat PNP::Distortion = []() {
    Mat distortion = Mat::zeros(5, 1, CV_64F);
    distortion.at<double>(0, 0) = -0.0681;
    distortion.at<double>(1, 0) = 0.1248;
    distortion.at<double>(2, 0) = -0.0000634;
    distortion.at<double>(3, 0) = 0.0005893;
    distortion.at<double>(4, 0) = -0.0815;
    return distortion;
}();

const vector<Point3f> PNP::world_points_big = [] {
    const double width = 0.115;
    const double height = 0.025;
    return vector<Point3f> {
        Point3f(-width, -height, 0),
        Point3f(-width,  height, 0),
        Point3f( width,  height, 0),
        Point3f( width, -height, 0)
    };
}();

const vector<Point3f> PNP::world_points_small = [] {
    const double width = 0.065;
    const double height = 0.025;
    return vector<Point3f> {
        Point3f(-width, -height, 0),
        Point3f(-width,  height, 0),
        Point3f( width,  height, 0),
        Point3f( width, -height, 0)
    };
}();

const Eigen::Vector3d PNP::camera_gimbal(0.05, 0, 0.10);

PNP::PNP(Data_Save ds)
{
    C_C.GetTFMat(ds.camera_roll_angle, -ds.camera_pitch_angle, ds.camera_yaw_angle);
    sys_yaw = ds.camera_yaw_angle;
    double cosx = cos(ds.camera_pitch_angle);
    double cosy = cos(-ds.camera_yaw_angle);
    double cosz = cos(ds.camera_roll_angle);
    double sinx = sin(ds.camera_pitch_angle);
    double siny = sin(-ds.camera_yaw_angle);
    double sinz = sin(ds.camera_roll_angle);
    R_armor2world << cosz*cosy - sinz*sinx*siny, -sinz*cosx, cosz*siny + sinz*sinx*cosy,
                     sinz*cosy + cosz*sinx*siny,  cosz*cosx, sinz*siny - cosz*sinx*cosy,
                                     -cosx*siny,       sinx,                  cosx*cosy;
}

void PNP::Solve(Armor &armor){
    if(armor.type == ArmorType::LARGE){
        world_points = world_points_big;
    }else{
        world_points = world_points_small;
    }
    cv::solvePnP(world_points, armor.points, Intrinsic_Matrix, Distortion, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    armor.cam_pose = {*tvec.ptr<double>(2), -*tvec.ptr<double>(0), -*tvec.ptr<double>(1)};
    armor.abs_pose = C_C.cam2abs(armor.cam_pose);
    armor.abs_ypd = C_C.xyz2ypd(armor.abs_pose);
    // 装甲板在相机系下坐标，相机在云台系下坐标,装甲板在世界系下坐标
    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    t_armor2world = R_armor2world * xyz_in_camera + camera_gimbal;
    double min_error = 1e10;
    double best_yaw = 0;
    for(int i=-70;i<70;i+=2){
        double yaw = sys_yaw + D2R(i);
        auto image_points = Reprojection(yaw);
        double error = 0.0;
        for (int j = 0; j < 4; j++) {
            error += sqrt(pow(armor.points[j].x - image_points[j].x, 2) + pow(armor.points[j].y - image_points[j].y, 2));
        }
        if(error<min_error){
            min_error = error;
            best_yaw = yaw;
        }  
    }
    armor.yaw = best_yaw;
    armor.re_points = Reprojection(best_yaw);
}

void PNP::BuffPnP(vector<Point2f> Buff_points, Point3d &pose, Point3d &Rpose, Point3d &Aimpose, double &roll, double &yaw)
{
    vector<Point3f> obj;
    double width = 0.165;
    double height = 0.03;

    obj.push_back(Point3f(width, -height, 0));
    obj.push_back(Point3f(-width, -height, 0));
    obj.push_back(Point3f(-width, height, 0));
    obj.push_back(Point3f(width, height, 0));

    cv::Mat rvec, tvec;
    cv::solvePnP(obj, Buff_points, Intrinsic_Matrix, Distortion, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    cv::Mat R;
    cv::Rodrigues(rvec, R);

    static std::deque<double> distance_history;
    double current_distance = *tvec.ptr<double>(2);

    distance_history.push_back(current_distance);
    if (distance_history.size() > 5)
    {
        distance_history.pop_front();
    }

    double sum = 0.0;
    for (double d : distance_history)
    {
        sum += d;
    }
    distance_x = sum / distance_history.size();

    pose = {distance_x, -*tvec.ptr<double>(0), -*tvec.ptr<double>(1)};
    // pose = C_C.cam2abs(pose);
    // roll = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)) + ds.camera_roll_angle;
    // yaw = PI / 2 + ds.camera_yaw_angle;

    Rpose = Point3d(
        pose.x - 0.344 * cos(roll) * cos(yaw),
        pose.y - 0.344 * cos(roll) * sin(yaw),
        pose.z - 0.344 * sin(roll)
    );
    
    Aimpose = Point3d(
        pose.x + 0.356 * cos(roll) * cos(yaw),
        pose.y + 0.356 * cos(roll) * sin(yaw),
        pose.z + 0.356 * sin(roll)
    );
}

vector<Point2f> PNP::Reprojection(double yaw){
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);
    auto sin_pitch = std::sin(13*PI/12);
    auto cos_pitch = std::cos(13*PI/12);
    // Matrix3d MatY = Eigen::Matrix<double, 3, 3>::Zero() ;
    // MatY << cos_yaw, 0, -sin_yaw, 
    //               0, 1,        0, 
    //         sin_yaw, 0,  cos_yaw;
    // Matrix3d MatX = Eigen::Matrix<double, 3, 3>::Zero() ;
    // MatX << 1,          0,          0, 
    //         0,  cos_pitch,  sin_pitch, 
    //         0, -sin_pitch,  cos_pitch;
    // Eigen::Matrix3d _R_armor2world = MatY * MatX;
    Eigen::Matrix3d _R_armor2world;
    _R_armor2world <<   cos_yaw,   sin_yaw * sin_pitch,   -sin_yaw * cos_pitch,
                              0,             cos_pitch,              sin_pitch,
                        sin_yaw,  -cos_yaw * sin_pitch,    cos_yaw * cos_pitch;
    
    Eigen::Matrix3d _R_armor2camera = R_armor2world.transpose() * _R_armor2world;
    Eigen::Vector3d t_armor2camera = R_armor2world.transpose() * t_armor2world - camera_gimbal; 
    cv::Vec3d _rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, _rvec);
    cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(world_points, _rvec, _tvec, Intrinsic_Matrix,  Distortion, image_points);
    return image_points;
}