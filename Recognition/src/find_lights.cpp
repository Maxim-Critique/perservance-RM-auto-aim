#include "find_armor.h"
#include "Coordinate.h"
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
    return (1.5 < lw_rate(rect) && lw_rate(rect) < 40 && rect.size.area() > 20 && rect.size.area() < 1500);
}
// 判断两个灯条区域是同一个灯条
static bool IsSameBlob(LightBlob blob1, LightBlob blob2) {
    auto dist = blob1.rect.center - blob2.rect.center;
    return (dist.x * dist.x + dist.y * dist.y) < 16 ;
}
// 倾斜角度检查
void SlopeValidation(LightBlobs &lightblobs)
{
    for (auto it = lightblobs.begin(); it != lightblobs.end(); ) {
        cv::Point2f direction = it->top - it->bottom;
        float tilt_angle = std::atan2(std::abs(direction.x), std::abs(direction.y));
        if (tilt_angle > D2R(45)) {
            it = lightblobs.erase(it); // 删除并获取下一个迭代器
        } else {
            ++it;
        }
    }
}

// 开闭运算
static void ImagePreProcessLight(cv::Mat &src) {

    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::morphologyEx(src, src, cv::MORPH_CLOSE, kernel_dilate);
}

static void ImagePreProcessDim(cv::Mat &src) {
    
    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::morphologyEx(src, src, cv::MORPH_CLOSE, kernel_dilate);
}

static uint8_t get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) {
    // 1. 计算 ROI 区域
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
    // 2. 提取 ROI 并直接计算通道总和（避免内存拷贝）
    cv::Mat roi = src(region);
    
    // 3. 使用 cv::sum() 直接计算 B 和 R 通道的总和
    cv::Scalar sum_b = cv::sum(roi.col(0)); // B 通道
    cv::Scalar sum_r = cv::sum(roi.col(2)); // R 通道

    return (sum_r[0] > sum_b[0]) ? BLOB_RED : BLOB_BLUE;
}

LightBlobs Tradition::FindLight(Mat &src) { // 默认参数为空ROI
    LightBlobs light_blobs;
    if(is_get_from_ds){
        std::unique_lock<std::mutex> lock(ds_mutex);
        enemy_color = ds.color;
    }

    cv::Mat color_channel;
    cv::Mat src_bin_light, src_bin_dim;
    std::vector<cv::Mat> channels;       // 通道拆分
    cv::split(src, channels); // 处理ROI子图像或原图

    if (enemy_color == ENEMY_BLUE) {         
        color_channel = channels[0];        
    } else if (enemy_color == ENEMY_RED) {    
        color_channel = channels[2];        
    }

    cv::threshold(color_channel, src_bin_light, light_threshold, 255, THRESH_BINARY);
    ImagePreProcessLight(src_bin_light);                                  

    cv::threshold(color_channel, src_bin_dim, dim_threshold, 255, THRESH_BINARY); 
    ImagePreProcessDim(src_bin_dim);                                  
    
    // imshow("light",src_bin_light);
    // imshow("dim",src_bin_dim);

    std::vector<std::vector<cv::Point>> light_contours_light, light_contours_dim;
    LightBlobs light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;

    cv::findContours(src_bin_light, light_contours_light, hierarchy_light, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    
    // 处理light_contours_light中的轮廓
    for (size_t i = 0; i < light_contours_light.size(); i++) {
        if (hierarchy_light[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
            if (IsValidLightBlob(rect)) {
                light_blobs_light.emplace_back(
                    rect, areaRatio(light_contours_light[i], rect), get_blob_color(src, rect)
                );
            }
        }
    }

    // 处理light_contours_dim中的轮廓
    for (size_t i = 0; i < light_contours_dim.size(); i++) {
        if (hierarchy_dim[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
            if (IsValidLightBlob(rect)) {
                light_blobs_dim.emplace_back(
                    rect, areaRatio(light_contours_dim[i], rect), get_blob_color(src, rect)
                );
            }
        }
    }

    std::vector<int> light_to_merge, dim_to_merge;
    for (size_t l = 0; l != light_blobs_light.size(); l++) {
        for (size_t d = 0; d != light_blobs_dim.size(); d++) {
            if (IsSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
                bool keep_light = false;
                keep_light = light_blobs_light[l].area_ratio >= light_blobs_dim[d].area_ratio;
                if (keep_light) {
                    light_to_merge.emplace_back(l);
                } else {
                    dim_to_merge.emplace_back(d);
                }
            }
        }
    }

    // 标记需要合并的元素
    std::vector<bool> light_blobs_light_to_merge(light_blobs_light.size(), false);
    std::vector<bool> light_blobs_dim_to_merge(light_blobs_dim.size(), false);
    for (auto x : light_to_merge) {
        light_blobs_light_to_merge[x] = true;
    }
    for (auto x : dim_to_merge) {
        light_blobs_dim_to_merge[x] = true;
    }

    // 合并结果
    for (size_t l = 0; l < light_blobs_light.size(); l++) {
        if (light_blobs_light_to_merge[l]) {
            light_blobs.emplace_back(light_blobs_light[l]);
        }
    }
    for (size_t d = 0; d < light_blobs_dim.size(); d++) {
        if (light_blobs_dim_to_merge[d]) {
            light_blobs.emplace_back(light_blobs_dim[d]);
        }
    }

    SlopeValidation(light_blobs);
    return light_blobs;
}