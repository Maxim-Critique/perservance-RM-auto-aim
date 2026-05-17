#include "find_armor.h"

// 旋转矩形的长宽比
static double lw_rate(const cv::RotatedRect &rect) {
    return rect.size.height > rect.size.width ?
           rect.size.height / rect.size.width :
           rect.size.width / rect.size.height;
}
// 轮廓面积和其最小外接矩形面积之比
static double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
    return cv::contourArea(contour) / rect.size.area();
}
// 判断轮廓是否为一个灯条
static bool IsValidLightBlob(const cv::RotatedRect &rect) {
    return (1.5 < lw_rate(rect) && lw_rate(rect) < 40 && rect.size.area() < 6000 && rect.size.area() > 18);
}

// 倾斜角度检查
void SlopeValidation(LightBlobs &lightblobs)
{
    for (auto it = lightblobs.begin(); it != lightblobs.end(); ) {
        cv::Point2f direction = it->top - it->bottom;
        float tilt_angle = std::atan2(std::abs(direction.x), std::abs(direction.y));
        if (tilt_angle > tools::D2R(45)) {
            it = lightblobs.erase(it); // 删除并获取下一个迭代器
        } else {
            ++it;
        }
    }
}

static void ImagePreProcessDim(cv::Mat &src) {
    
    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::morphologyEx(src, src, cv::MORPH_CLOSE, kernel_dilate);
}

static int GetBlobColor(const cv::Mat &src, const cv::RotatedRect &blobPos) {
    // 1. 计算ROI区域
    cv::Rect region = blobPos.boundingRect();
    int offset_x = std::max(3, static_cast<int>(region.width * 0.1));
    int offset_y = std::max(3, static_cast<int>(region.height * 0.05));
    region.x -= offset_x;
    region.y -= offset_y;
    region.width += 2 * offset_x;
    region.height += 2 * offset_y;
    region &= cv::Rect(0, 0, src.cols, src.rows);
    if (region.empty()) {
        return BLOB_BLUE; 
    }
    
    // 2. 提取ROI
    cv::Mat roi = src(region);
    
    // 3. 使用split分离通道并计算总和
    std::vector<cv::Mat> channels;
    cv::split(roi, channels);  // 分离为B,G,R通道
    
    // 4. 计算B和R通道的总和
    double sum_b = cv::sum(channels[0])[0]; // B通道总和
    double sum_r = cv::sum(channels[2])[0]; // R通道总和
    
    return (sum_r > sum_b) ? BLOB_RED : BLOB_BLUE;
}

LightBlobs Tradition::FindLight(Mat &src, int enemy_color) { 
    LightBlobs light_blobs;
    enemy_color_ = 0;

    cv::Mat color_channel;
    cv::Mat src_bin;  // 只需要一个二值图像
    std::vector<cv::Mat> channels;
    cv::split(src, channels);

    if (enemy_color_ == ENEMY_BLUE) color_channel = channels[0]; 
    else color_channel = channels[2];       

    cv::threshold(color_channel, src_bin, threshold_, 255, THRESH_BINARY);
    ImagePreProcessDim(src_bin); 
    // imshow("src_bin",src_bin);
    std::vector<std::vector<cv::Point>> light_contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(src_bin, light_contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    
    for (size_t i = 0; i < light_contours.size(); i++) {
        if (hierarchy[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours[i]);
            if (IsValidLightBlob(rect)) {
                light_blobs.emplace_back(
                    rect, areaRatio(light_contours[i], rect), GetBlobColor(src, rect)
                );
            }
        }
    }
    SlopeValidation(light_blobs);
    return light_blobs;
}