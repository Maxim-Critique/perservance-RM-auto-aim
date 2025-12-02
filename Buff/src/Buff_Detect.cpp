// #include "Buff_Detect.h"
// #include "SerialPort.h"
// cv::dnn::Net net_bh;
// std::mutex mutex_bh;

// double Distance(Point2f m, Point2f n)
// {
//     double distance;
//     distance = static_cast<double>(powf(m.x - n.x, 2) + powf(m.y - n.y, 2));
//     distance = static_cast<double>(sqrt(distance));
//     return distance;
// }


// Mat Buff_Discover::Buff_img_deal(Mat src) 
// {
//     Mat mask;
//     Mat gray;
//     Mat Light_color;
//     Mat hsv_img;
//     cvtColor(src, hsv_img, COLOR_BGR2HSV);
//     Mat hsv_end;
//     vector<Mat> channels;
//     split(hsv_img, channels);
//     static int h_value = 100; 
//     static int s_value = 100;
//     static int v_value = 100;
//     h_value = std::max(1, h_value);   // 防止除以0
//     s_value = std::max(1, s_value);
//     v_value = std::max(1, v_value);

//     channels.at(0) = channels.at(0) * 100 / h_value;
//     channels.at(1) = channels.at(1) * 100 / s_value;
//     channels.at(2) = channels.at(2) * 100 / v_value;
//     merge(channels, hsv_end);

//     Mat bgr_end;
//     cvtColor(hsv_end, bgr_end, COLOR_HSV2BGR);

//     vector<Mat> bgr;

//     cvtColor(bgr_end, gray, COLOR_BGR2GRAY);
//     split(bgr_end, bgr);

//     // cvtColor(src, gray, COLOR_BGR2GRAY);
//     // split(src, bgr);
//     // 颜色模式选择
//     int buff_color; 

//     // 初始化阈值参数（直接使用预设值）
//     static int threshold_channal_value = 100;
//     static int threshold_gray_value = 70;

//     // 移除BUFF_DEBUG相关代码（包括Trackbar和窗口）
//     if (buff_color == 1) {
//         subtract(bgr[0], bgr[2], Light_color);
//     } else {
//         subtract(bgr[2], bgr[0], Light_color);
//     }
//     // 图像处理逻辑（直接使用预设阈值）
//     threshold(Light_color, mask, threshold_channal_value, 255, THRESH_BINARY);
//     GaussianBlur(mask, mask, Size(3, 3), 3, 3);
//     threshold(gray, gray, threshold_gray_value, 255, THRESH_BINARY);
//     GaussianBlur(gray, gray, Size(3, 3), 3, 3);
//     // 清理中间变量
//     vector<Mat>().swap(bgr);
//     Light_color.release();
//     // 合并结果
//     Mat Mat_Result = mask & gray;
//     mask.release();
//     gray.release();
//     return Mat_Result;
// }

// Contour_RotatedRect ThrowStruct(vector<Point>contour,const Mat &src){
//     Contour_RotatedRect temporary;
//     // cv::RotatedRect originalRect = cv::minAreaRect(contour);
//     // // // 获取原矩形的中心、尺寸和角度
//     // cv::Point2f center = originalRect.center;  // 中心点不变
//     // cv::Size2f size = originalRect.size;       // 原尺寸
//     // float angle = originalRect.angle;          // 原旋转角度

//     // // 将边长放大 2 倍
//     // size.width *= 2.4f;
//     // size.height *= 2.4f;

//     // // 构造新的旋转矩形
//     // cv::RotatedRect enlargedRect(center, size, angle);

//     // // 赋值给目标变量
//     // temporary.rect = enlargedRect;

//     temporary.rect = cv::minAreaRect(contour);

//     // 确保width为长边（与矩形处理逻辑一致）
//     if (temporary.rect.size.width < temporary.rect.size.height) {
//         std::swap(temporary.rect.size.width, temporary.rect.size.height);
//         temporary.rect.angle += 90.0f;
//     }

//     // 获取旋转矩形的四个顶点
//     cv::Point2f vertices[4];
//     temporary.rect.points(vertices);

//     // 转换为vector并计算凸包以确保顶点顺序
//     std::vector<cv::Point2f> src_vertices;
//     for (int i = 0; i < 4; ++i) {
//         src_vertices.push_back(vertices[i]);
//     }
//     std::vector<cv::Point2f> hull;
//     cv::convexHull(src_vertices, hull, true); // 顺时针排列

//     // 定义目标顶点（保持与矩形处理相同的映射逻辑）
//     std::vector<cv::Point2f> dst_vertices = {
//         cv::Point2f(0, 0),
//         cv::Point2f(0, temporary.rect.size.height),
//         cv::Point2f(temporary.rect.size.width, temporary.rect.size.height),
//         cv::Point2f(temporary.rect.size.width, 0)
//     };

//     // 计算透视变换矩阵
//     cv::Mat transform = cv::getPerspectiveTransform(hull, dst_vertices);

//     // 应用透视变换
//     cv::warpPerspective(src, temporary.image, transform, 
//                     cv::Size(temporary.rect.size.width, temporary.rect.size.height));

//     // 图像处理（保持与矩形处理相同的参数）
//     cv::adaptiveThreshold(temporary.image, temporary.image, 255, 
//                         cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
//                         cv::THRESH_BINARY, 19, -0.5);
//     return temporary;

// }


// //找靶环
// void Buff_Discover::ExtractCirRegions(cv::Mat src,Mat &draw_img) 
// {
//     std::vector<std::vector<cv::Point>> circularContours;
//     double ratio_min_threshold = 0.65;
//     double ratio_max_threshold = 0.95;
//     int min_area = 1500;
//     int max_area = 6000;
//     for (const auto& contour : contours) {
//         double area = cv::contourArea(contour);
//         if (area < min_area) continue;
//         cv::Point2f center;
//         float radius;
//         cv::minEnclosingCircle(contour, center, radius);
//         // circle(draw_img,center,radius,Scalor(255,0,0),2);
//         double circle_area = CV_PI * radius * radius;
//         if (circle_area <= 0) continue;
//         // if(show){
//         //     circle(draw_img,center,radius,Scalar(200,0,200),2);
//         //     putText(draw_img,"Size: "+to_string(area)+"Bili :"+to_string(area / circle_area),center, FONT_HERSHEY_COMPLEX, 0.6, Scalar(205, 100, 225), 1);
//         // }
        
//         if (area > max_area) continue;
//         if ((area / circle_area) >= ratio_min_threshold&&(area / circle_area) <= ratio_max_threshold) {

//             circularContours.push_back(contour);
//         }
//     }
//     for (const auto& contour : circularContours) {
//         contour_rect_bh.emplace_back(ThrowStruct(contour,src));
//     }
// }


// //找流水灯
// void Buff_Discover::ExtractRectRegions(cv::Mat src, Mat draw_img) 
// {
//     std::vector<cv::Mat> output_regions;
//     std::vector<std::vector<cv::Point>> filteredContours;
//     int min_area = 600;
//     float min_aspect = 0.1f;
//     float max_aspect = 0.4f;
//     for (const auto& contour : contours) {
//         double area = cv::contourArea(contour);
//         if (area < min_area) continue;
        
//         cv::RotatedRect rect = cv::minAreaRect(contour);
//         cv::Point2f vertices[4];
//         rect.points(vertices);
//         float width = rect.size.width;
//         float height = rect.size.height;
//         float aspect_ratio = std::min(width, height) / std::max(width, height);
        
//         // if(show){
//         //     for (int i = 0; i < 4; ++i) {
//         //         cv::line(draw_img, vertices[i], vertices[(i+1)%4], 
//         //                 cv::Scalar(0, 255, 0), 8, cv::LINE_AA); // 原始矩形用绿色
//         //     }
//         //     putText(draw_img,"Size: "+to_string(area)+"Bili :"+to_string(aspect_ratio),rect.center, FONT_HERSHEY_COMPLEX,
//         //      0.6, Scalar(205, 100, 225), 1);
//         // }

//         if (aspect_ratio >= min_aspect && aspect_ratio <= max_aspect) {
//             Contour_RotatedRect sl;
//             sl.rect = rect;
//             sl.contour = contour;
//             contour_rect_sl.emplace_back(sl);
//         }
//     }

//     // 按照与靶环中心的距离排序（从小到大）

//         std::sort(contour_rect_sl.begin(), contour_rect_sl.end(), 
//             [this](const Contour_RotatedRect& a, const Contour_RotatedRect& b) {
//                 cv::Point2f bh_center_f = static_cast<cv::Point2f>(this->bh_center);
//                 float dist_a = Eigen::norm(a.rect.center - bh_center_f);
//                 float dist_b = Eigen::norm(b.rect.center - bh_center_f);
//                 return dist_a < dist_b;  // 从小到大排序
//             }
//         );
    
// }

// int i = 4000;
// void SaveImg(Mat roi,string area){
//     string Photo_name = area + to_string(i) + ".jpg";
//     if(i % 5 == 0 ){
//         imwrite(Photo_name, roi);
//     }
//     i++;
// }



// Contour_RotatedRect Buff_Discover::ProcessBH() {
//     Contour_RotatedRect best_bh ;
//     float max_confidence = 0.0f;
//     if(!contour_rect_bh.empty()){
//         for (auto &bh : contour_rect_bh) {
//             // 图像预处理
//             cv::Mat processed_img;
//             cv::resize(bh.image, bh.image, cv::Size(28, 28));
//             // SaveImg(bh.image,"/home/killer_machine/WRITE/photos/");
//             // 创建blob（与原始模型输入一致）
//             cv::Mat blob;
//             cv::dnn::blobFromImage(bh.image, blob, 1.0, cv::Size(28, 28)); // 强制缩放至模型输入尺寸
//             // 线程安全推理
//             {
//                 std::lock_guard<std::mutex> lock(mutex_bh);
//                 net_bh.setInput(blob);
//                 cv::Mat outputs = net_bh.forward();
                
//                 // 解析输出（假设二分类）
//                 float score0 = outputs.at<float>(0, 0);
//                 float score1 = outputs.at<float>(0, 1);
                
//                 // 筛选逻辑
//                 if (score1 < score0) {
//                     if (score0 > max_confidence) {
//                         max_confidence = score0;
//                         best_bh = bh;
//                     }
//                 }
//             }
//         }
//     }
//     return best_bh;
// }



// Contour_RotatedRect Buff_Discover::ProcessSL(Point2f center_bh) {
//     for (auto& candidate : contour_rect_sl) {
//         const Point2f center = candidate.rect.center;
//         const Point2f direction = center - center_bh;
        
//         const float angle_diff = [&]{
//             float rect_angle = candidate.rect.angle;
//             if (candidate.rect.size.width < candidate.rect.size.height) 
//                 rect_angle += 90.0f;
            
//             const float line_angle = cv::fastAtan2(direction.y, direction.x);
//             float delta = fmod(fabs(rect_angle - line_angle), 360.0f);
//             return delta > 180.0f ? 360.0f - delta : delta;
//         }();

//         // Calculate area of the rotated rectangle
//         float area = candidate.rect.size.area();
        
//         if ((angle_diff <= 10.0f || angle_diff >= 170.0f) && 
//             area > 1000.0f && area < 5000.0f) {
//             Point2f rect_vertices[4];
//             candidate.rect.points(rect_vertices);

//             // 获取轮廓点并找到每个矩形顶点最近的轮廓点
//             std::vector<Point2f> closest_points;
//             for (int i = 0; i < 4; ++i) {
//                 // 使用最小距离查找最近轮廓点
//                 auto closest = std::min_element(candidate.contour.begin(), candidate.contour.end(),
//                     [&](const Point2f& a, const Point2f& b) {
//                         return Eigen::norm(a - rect_vertices[i]) < Eigen::norm(b - rect_vertices[i]);
//                     });
//                 closest_points.push_back(*closest);
//             }

//             // 按参考角度排序
//             const float ref_angle = cv::fastAtan2(direction.y, direction.x);
//             std::sort(closest_points.begin(), closest_points.end(), [&](const Point2f& a, const Point2f& b) {
//                 auto rel_angle = [&](const Point2f& p) {
//                     return fmod(cv::fastAtan2(p.y - center.y, p.x - center.x) - ref_angle + 360.0f, 360.0f);
//                 };
//                 return rel_angle(a) < rel_angle(b);
//             });

//             Contour_RotatedRect sl;
//             sl.rect = candidate.rect;
//             sl.ordered_pnp_points = std::move(closest_points);
//             return sl;
//         }
//     }
//     return {};
// }
// void drawRotatedRect(cv::Mat& image, const cv::RotatedRect& rect,const std::vector<cv::Point2f>& ordered_points) 
// {
//     // 绘制原始旋转矩形（绿色）
//     if (rect.size.area() > 0) {
//         cv::Point2f vertices[4];
//         rect.points(vertices);
//         for (int i = 0; i < 4; ++i) {
//             cv::line(image, vertices[i], vertices[(i+1)%4], 
//                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA); // 原始矩形用绿色
//         }
//     }

//     if (!ordered_points.empty()) {
//         // 标注顶点序号
//         for (size_t i = 0; i < ordered_points.size(); ++i) {
//             cv::putText(image, std::to_string(i), ordered_points[i],
//                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
//         }
//     }
// }
// bool Buff_Discover::FoundBuff(Mat &img, Mat &draw_img) {

//     net_bh = cv::dnn::readNetFromONNX("../model/mlp_bh.onnx");
//     std::vector<cv::Vec4i> hierarchy;
//     contours.clear();
//     contour_rect_bh.clear();
//     contour_rect_sl.clear();
//     cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//     ExtractCirRegions(img,draw_img);
//     Contour_RotatedRect bh = ProcessBH();
//     if(bh.image.empty()){
//         cout<<"omg"<<endl;
//         return false;
//     }
//     imshow("bh",bh.image);
//     ExtractRectRegions(img,draw_img);
//     this->sl = ProcessSL(bh.rect.center);
//     this->bh_center = bh.rect.center;

//     if(show){
//         circle(draw_img,bh.rect.center,40,Scalar(200,0,200),20);
//         drawRotatedRect(draw_img,sl.rect,sl.ordered_pnp_points);
//     }
//     return false;
//     // if(sl.rect.size.area()>0){
//     //     pnp_armor.BuffPnP(sl.ordered_pnp_points,sl.pose,Rpose,Aimpose,sl.roll,sl.yaw);
//     //     return true;
//     // }
//     // return false;
// }

// Buff_Discover::Buff_Discover(){
    
//     // show = config["Camera"]["show"].as<int>();
//     show = 1;
// }

