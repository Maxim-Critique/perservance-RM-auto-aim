#include "BuffDetect.h"
#include "SerialPort.h"
cv::dnn::Net net_bh;
std::mutex mutex_bh;


namespace auto_buff{

BuffDetector::BuffDetector(){
    net_ = cv::dnn::readNetFromONNX("/home/nav4/auto-aim/model/mlp_bh.onnx");
}

void BuffDetector::HandleImg(){
    // 彩色图转灰度图
    cv::cvtColor(img_, gray_img_, cv::COLOR_BGR2GRAY);  
    // cv::imshow("gray", gray_img_); 
    // 进行二值化 
    cv::threshold(gray_img_, gray_img_, 50, 255, cv::THRESH_BINARY);
    // cv::imshow("gray", gray_img_);
    // 找轮廓
    cv::findContours(gray_img_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
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
    
    vector<FanBlade> fanblades;                               // 存放扇叶
    std::vector<std::pair<float,std::vector<cv::Point2f>>>rects;
    img_ = img;
    // 预处理
    HandleImg();         
    // 进行圆拟合，获得待机击打区域
    DetectCircles(fanblades);
    if(fanblades.empty()){
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
    GetLightBars(fanblades,rects);
    /// 生成PowerRune
    auto r_center = GetR(fanblades);
    PowerRune powerrune(fanblades, r_center, last_powerrune_);
    // circle(img_,r_center,2,Scalar(0,200,0),-1);
    // putText(img_, std::to_string(6), r_center, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,200,0), 0.2);
    /// handle error
    if (powerrune.is_unsolve()) {
        Lose();
        return std::nullopt;
    }
    status_ = TRACK;
    lose_ = 0;
    std::optional<PowerRune> P;
    P.emplace(std::move(powerrune));
    last_powerrune_ = P;
    
    return P;
    
}

cv::Point2f BuffDetector::GetR(std::vector<FanBlade> & fanblades)
{

    /// 算出大概位置
    // 从点5（扇叶中心）出发，向点6方向延伸1.4倍距离
    cv::Point2f r_center_t = {0, 0};
    for (auto & fanblade : fanblades) {
        r_center_t += (fanblade.points[5]-fanblade.points[4])+fanblade.points[5];  // TODO
    }
    r_center_t /= float(fanblades.size());

    /// 处理图片,mask选出大概范围
    double radius = cv::norm(fanblades[0].points[2] - fanblades[0].center) * 0.8;
    cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8U);  // mask
    circle(mask, r_center_t, radius, cv::Scalar(255), -1);
    bitwise_and(gray_img_, mask, gray_img_);               // 将遮罩应用于二值化图像
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

bool BuffDetector::Verify(cv::Point2f points[4]) {
    cv::Point2f target_vertices[4] = {cv::Point2f(0, 0), cv::Point2f(0, 27), cv::Point2f(27, 27), cv::Point2f(27, 0)};
    auto rotation_matrix = cv::getPerspectiveTransform(points, target_vertices);
    cv::Mat input_image;
    cv::warpPerspective(gray_img_, input_image, rotation_matrix, cv::Size(28, 28));
    input_image /= 255;
    cv::Mat blob;
    cv::dnn::blobFromImage(input_image, blob);
    mutex_bh.lock();
    net_.setInput(blob);
    cv::Mat outputs = net_.forward().clone();
    mutex_bh.unlock();
    // 获取类别1的概率
    float probability = outputs.at<float>(0, 0);  
    float sigmoid_prob = 1.0f / (1.0f + exp(-probability));  
    return sigmoid_prob > 0.7f;
}

void BuffDetector::DetectCircles(vector<FanBlade>& fanblades){
    double min_ratio = 0.65;
    double max_ratio = 0.95;
    int min_area = 1500;
    int max_area = 6000;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        cv::Point2f center;float radius;
        cv::minEnclosingCircle(contour, center, radius);
        double area_ratio = area /(CV_PI * pow(radius,2));
        if(this->debug_){
            circle(img_,center,radius,Scalar(200,0,200),2);
            putText(img_,"Size: "+to_string(area)+"Ratio :"+to_string(area_ratio),center, cv::FONT_HERSHEY_COMPLEX, 0.6, Scalar(205, 100, 225), 1);
        }
        if ((area>min_area) && (area<max_area) && (area_ratio>min_ratio) && (area_ratio<max_ratio) ){
            cv::RotatedRect rect = cv::minAreaRect(contour);
            cv::Point2f vertices[4];
            rect.points(vertices);
            if(Verify(vertices))fanblades.emplace_back(center,radius,_light);
        }
    }
}

void BuffDetector::DetectRect(std::vector<std::pair<float,std::vector<cv::Point2f>>> &rects){
    float min_area = 500;
    float max_area = 3000;
    float min_aspect = 0.05f;
    float max_aspect = 0.3f;
    for (const auto& contour : contours) {
        cv::RotatedRect rect = cv::minAreaRect(contour);
        cv::Point2f vertices[4];
        rect.points(vertices);
        float area = rect.size.area();
        bool w_h = (rect.size.width < rect.size.height);
        float aspect_ratio = w_h ? (rect.size.width / rect.size.height) : (rect.size.height / rect.size.width);
        if(debug_){
            for (int i = 0; i < 4; ++i) {cv::line(img_, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA); }
            putText(img_,"Size: "+to_string(area)+"Bili :"+to_string(aspect_ratio),vertices[0], cv::FONT_HERSHEY_COMPLEX,0.6, Scalar(0, 255, 0), 1);
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

void BuffDetector::GetLightBars(vector<FanBlade>& fanblades, std::vector<std::pair<float, std::vector<cv::Point2f>>>& rects) {
    for (auto& fan : fanblades) {
        if (show_) {
            circle(img_, fan.center, fan.radius, Scalar(200, 0, 200), 2);
        }

        float min_distance = std::numeric_limits<float>::max();
        const std::vector<cv::Point2f>* best_rect_points = nullptr;

        for (const auto& rect : rects) {
            const cv::Point2f& ref_point = rect.second.back(); 
            float dist = cv::norm(ref_point - fan.center);
            if (dist < min_distance) {
                min_distance = dist;
                best_rect_points = &(rect.second);
            }
        }

        if (min_distance != std::numeric_limits<float>::max() && best_rect_points) {
            cv::Point2f ref_point = best_rect_points->back(); 
            float r = fan.radius - 2;
            std::vector<cv::Point2f> all_points = GetPoints(fan.center, r, ref_point);
            fan.points = all_points;
            for(int i=0;i<6;i++){
                circle(img_,fan.points[i],2,Scalar(0,200,0),-1);
                putText(img_, std::to_string(i), fan.points[i], FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0,200,0), 0.2);
            }
        }
    }
}
}// namespace


