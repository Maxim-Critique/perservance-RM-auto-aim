#include "BuffDetect.h"
#include "SerialPort.h"
#include "math_tools.hpp"
#include "Base.h"
cv::dnn::Net net_bh;
std::mutex mutex_bh;


namespace auto_buff{

BuffDetector::BuffDetector(){
    net_ = cv::dnn::readNetFromONNX("../model/mlp_bh.onnx");
}

void BuffDetector::HandleImg(){
    // 彩色图转灰度图
    cv::cvtColor(img_, gray_img_, cv::COLOR_BGR2GRAY);  
    // cv::imshow("gray", gray_img_); 
    // 进行二值化 
    cv::threshold(gray_img_, gray_img_, 80, 255, cv::THRESH_BINARY);
    // cv::imshow("binary", gray_img_);
    // 3x3闭运算
    cv::morphologyEx(gray_img_, gray_img_, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    // 找轮廓
    cv::findContours(gray_img_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto it = std::remove_if(contours.begin(), contours.end(),
        [](const std::vector<cv::Point>& c) { return c.size() < 20; });
    contours.erase(it, contours.end());
}

void BuffDetector::Lose()
{
    lose_++;
    if (lose_ >= 20) {
        status_ = LOSE;
        last_powerrune_ = std::nullopt;
    }
    status_ = TEM_LOSE;
}

std::optional<PowerRune> BuffDetector::Detect(cv::Mat & img){
    // 存放扇叶
    vector<FanBlade> fanblades;                              
    std::vector<std::pair<float,std::vector<cv::Point2f>>>rects;
    img_ = img;
    // 预处理
    HandleImg();         
    // 进行圆拟合，获得待机击打区域
    DetectCircles(fanblades);
    if(fanblades.empty()||fanblades.size()>2){
        Lose();
        return std::nullopt;
    }
     
    // 进行矩形拟合
    DetectRect(rects);
    if(rects.empty()){
        Lose();
        return std::nullopt;
    }
    // 获得流水灯
    if (!GetLightBars(fanblades, rects)) {
        Lose();
        return std::nullopt;
    }
    /// 生成PowerRune
    auto r_center = GetR(fanblades);    
    PowerRune powerrune(fanblades, r_center, last_powerrune_);
    if (powerrune.is_unsolve()) {
        Lose();
        return std::nullopt;
    }
    status_ = TRACK;
    lose_ = 0;

    if(show_)Show(powerrune);
    std::optional<PowerRune> P;
    P.emplace(std::move(powerrune));
    last_powerrune_ = P;
    return P;
    
}

void BuffDetector::Show(PowerRune &P){
    // 绘制待机击打区域的外接圆
    for(auto &fan : P.fanblades){
        for(int i=0;i<6;i++){
            circle(img_,fan.points[i],2,Scalar(0,200,0),-1);
            putText(img_, std::to_string(i), fan.points[i], FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,200,0), 0.2);
        }
    }    
    circle(img_,P.r_center,2,Scalar(0,200,0),-1);
    putText(img_, std::to_string(6), P.r_center, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,200,0), 0.2);    
    
}

cv::Point2f BuffDetector::GetR(std::vector<FanBlade> & fanblades)
{

    /// 算出大概位置
    cv::Point2f r_center_t = {0, 0};
    for (auto & fanblade : fanblades) {
        r_center_t += (fanblade.points[5]-fanblade.points[4])+fanblade.points[5];  // TODO
    }
    r_center_t /= float(fanblades.size());
    return r_center_t;
    /// 处理图片,mask选出大概范围
    double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
    cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8U);  // mask
    circle(mask, r_center_t, radius, cv::Scalar(255), -1);
    bitwise_and(gray_img_, mask, gray_img_);                 // 将遮罩应用于二值化图像
    // cv::imshow("Dilated Image", gray_img_);               // 调试用

    std::vector<std::vector<cv::Point>> contours;
    auto r_center = r_center_t;
    cv::findContours(gray_img_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);  // external找外部区域
    double ratio_1 = INF;
    for (auto & it : contours) {
            auto rotated_rect = cv::minAreaRect(it);
            double ratio = rotated_rect.size.height > rotated_rect.size.width
                            ? rotated_rect.size.height / rotated_rect.size.width
                            : rotated_rect.size.width / rotated_rect.size.height;
            ratio += cv::norm(rotated_rect.center - r_center_t) / (radius / 3);
            if (ratio < ratio_1) {
            ratio_1 = ratio;
            r_center = rotated_rect.center;
        }
    }
    return r_center;
}

void SaveImg(Mat &input_image){
    if (input_image.empty()) cout << "warpPerspective失败，图像为空!" << endl;
    static int number = 1;
    number++;
    string filename = "../train/train_bh/datasets/photos/" + to_string(number) + ".png";
    imwrite(filename, input_image); 
}

bool BuffDetector::Verify(cv::Point2f points[4]) {
    cv::Point2f target_vertices[4] = {cv::Point2f(0, 0), cv::Point2f(0, 27), cv::Point2f(27, 27), cv::Point2f(27, 0)};
    auto rotation_matrix = cv::getPerspectiveTransform(points, target_vertices);
    cv::Mat input_image;
    cv::warpPerspective(gray_img_, input_image, rotation_matrix, cv::Size(28, 28));
    cv::Mat job_image = input_image.clone();
    // 转换数据类型为 float32 并归一化到 [0, 1]
    input_image.convertTo(input_image, CV_32F, 1.0 / 255.0);
    
    // blobFromImage: 创建 NCHW 格式的 blob，匹配训练时的格式
    // scalefactor=1.0, size=(28,28), mean=0, swapRB=false, crop=false, ddepth=CV_32F
    cv::Mat blob = cv::dnn::blobFromImage(input_image, 1.0, cv::Size(28, 28), 
                                          cv::Scalar(0), false, false, CV_32F);
    
    mutex_bh.lock();
    net_.setInput(blob);
    cv::Mat outputs = net_.forward().clone();
    mutex_bh.unlock();
    
    // 获取类别1的 logit 值并应用 sigmoid 激活函数
    float probability = outputs.at<float>(0, 0);
    float sigmoid_prob = 1.0f / (1.0f + std::exp(-probability));
    return sigmoid_prob < 0.5f;
}

void BuffDetector::DetectCircles(vector<FanBlade>& fanblades){
    double min_ratio = 0.1;
    double max_ratio = 1;
    int min_size = 5000;
    int max_size = 9000;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        cv::Point2f center;float radius;
        cv::minEnclosingCircle(contour, center, radius);
        auto size = CV_PI * pow(radius,2);
        double area_ratio = area / size;
        if(this->debug_){
            circle(img_,center,radius,Scalar(200,0,200),2);
            putText(img_,"Size: "+to_string(size)+"Ratio :"+to_string(area_ratio),center, cv::FONT_HERSHEY_COMPLEX, 0.6, Scalar(205, 100, 225), 1);
        }
        
        if ((size>min_size) && (size<max_size)  ){
            
            cv::RotatedRect rect = cv::minAreaRect(contour);
            cv::Point2f vertices[4];
            rect.points(vertices);
            if(Verify(vertices))fanblades.emplace_back(center,radius,_light);
        }
    }
}

void BuffDetector::DetectRect(std::vector<std::pair<float,std::vector<cv::Point2f>>> &rects){
    float min_area = 1300;
    float max_area = 4500;
    float min_aspect = 0.15f;
    float max_aspect = 0.7f;
    for (const auto& contour : contours) {
        cv::RotatedRect rect = cv::minAreaRect(contour);
        cv::Point2f vertices[4];
        rect.points(vertices);
        float area = rect.size.area();
        bool w_h = (rect.size.width < rect.size.height);
        float aspect_ratio = w_h ? (rect.size.width / rect.size.height) : (rect.size.height / rect.size.width);
        if(debug_){
            for (int i = 0; i < 4; ++i) cv::line(img_, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA); 
            putText(img_,"Size: "+to_string(area)+"Bili :"+to_string(aspect_ratio),vertices[1], cv::FONT_HERSHEY_COMPLEX,0.6, Scalar(0, 255, 0), 1);
        }
        
        if ((area>min_area) && (area<max_area) && (aspect_ratio>min_aspect) && (aspect_ratio<max_aspect)) {
            if(w_h)rect.angle+=90;
            std::vector<cv::Point2f> points(vertices, vertices + 4);
            points.emplace_back(rect.center);
            rects.emplace_back(rect.angle,points);
        }
    }
}

std::vector<cv::Point2f> GetPoints(
    const cv::Point2f& center, float radius, const cv::Point2f& ref_point) 
{
    std::vector<cv::Point2f> all_points;

    cv::Vec2f AB = cv::Vec2f(ref_point - center);
    cv::Vec2f dir_L1 = normalize(AB);
    cv::Vec2f dir_L2(-dir_L1[1], dir_L1[0]);  // 垂直向量

    // L1: A ± dir_L1 * r,L2: A ± dir_L2 * r
    all_points.push_back(center-cv::Point2f(dir_L1[0]*radius, dir_L1[1]*radius));
    all_points.push_back(center+cv::Point2f(dir_L2[0]*radius, dir_L2[1]*radius));
    all_points.push_back(center+cv::Point2f(dir_L1[0]*radius, dir_L1[1]*radius));
    all_points.push_back(center-cv::Point2f(dir_L2[0]*radius, dir_L2[1]*radius));
    all_points.push_back(center);
    all_points.push_back(ref_point);
    return all_points;
}

bool BuffDetector::GetLightBars(vector<FanBlade>& fanblades, std::vector<std::pair<float, std::vector<cv::Point2f>>>& rects) {
    for (auto& fan : fanblades) {

        float min_distance = std::numeric_limits<float>::max();
        const std::vector<cv::Point2f>* best_rect_points = nullptr;

        for (const auto& rect : rects) {
            const cv::Point2f& ref_point = rect.second.back();
            float ref_angle = tools::R2D(std::atan2(ref_point.y - fan.center.y, ref_point.x - fan.center.x));
            float angle_diff = std::fabs(ref_angle - rect.first);
            if (angle_diff > 180) angle_diff = 360 - angle_diff;
            float dist = cv::norm(ref_point - fan.center);
            
            bool is_valid = (angle_diff < 20 || angle_diff > 160) && dist > fan.radius;
            
            if (is_valid && dist < min_distance) {
                min_distance = dist;
                best_rect_points = &(rect.second);
            }
        }

        if (best_rect_points) {
            cv::Point2f ref_point = best_rect_points->back(); 
            float r = fan.radius - 4;
            std::vector<cv::Point2f> all_points = GetPoints(fan.center, r, ref_point);
            fan.points = all_points;
        }
    }
    return std::all_of(fanblades.begin(), fanblades.end(), [](const FanBlade& f) { return !f.points.empty(); });
}
}// namespace


