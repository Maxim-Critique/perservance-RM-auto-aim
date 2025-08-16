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
static bool isValidLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
    return true;
    return (0.8 < lw_rate(rect) && lw_rate(rect) < 120) &&
                (rect.size.area() < 6000) && ((rect.size.area() > 5)) ;
           ((rect.size.area() > 200 && areaRatio(contour, rect) > 0.5)||(rect.size.area() < 200));

}

// 判断两个灯条区域是同一个灯条
static bool isSameBlob(LightBlob blob1, LightBlob blob2) {
    auto dist = blob1.rect.center - blob2.rect.center;
    return (dist.x * dist.x + dist.y * dist.y) < 25 ;
}
// 开闭运算
static void imagePreProcess(cv::Mat &src) {
    

    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(src, src, kernel_dilate);

    static cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
    erode(src, src, kernel_erode);
    
    static cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(src, src, kernel_dilate2);

    static cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(src, src, kernel_erode2);
}
static void imagePreProcess1(cv::Mat &src) {
    
    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(src, src, kernel_dilate);
    
    static cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(src, src, kernel_erode);

    // static cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    // dilate(src, src, kernel_dilate2);

    // static cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    // erode(src, src, kernel_erode2);
}

// // 判断灯条颜色(此函数可以有性能优化).
// static uint8_t get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) {
//     auto region = blobPos.boundingRect();
//     region.x -= fmax(3, region.width * 0.1);
//     region.y -= fmax(3, region.height * 0.05);
//     region.width += 2 * fmax(3, region.width * 0.1);
//     region.height += 2 * fmax(3, region.height * 0.05);
//     region &= cv::Rect(0, 0, src.cols, src.rows);
//     cv::Mat roi = src(region);
//     int red_cnt = 0, blue_cnt = 0;
//     for (int row = 0; row < roi.rows; row++) {
//         for (int col = 0; col < roi.cols; col++) {
//             red_cnt += roi.at<cv::Vec3b>(row, col)[2];
//             blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
//         }
//     }
//     if (red_cnt > blue_cnt) {
//         return BLOB_RED;
//     } else {
//         return BLOB_BLUE;
//     }
// }

static uint8_t get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) {
    auto region = blobPos.boundingRect();
    int offset_x = fmax(3, static_cast<int>(region.width * 0.1));
    int offset_y = fmax(3, static_cast<int>(region.height * 0.05));
    region.x -= offset_x;
    region.y -= offset_y;
    region.width += 2 * offset_x;
    region.height += 2 * offset_y;
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);

    int red_cnt = 0, blue_cnt = 0;
    const uchar* ptr = roi.ptr<uchar>(0);
    int step = roi.step;
    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += ptr[2];
            blue_cnt += ptr[0];
            ptr += 3;
        }
        ptr += step - roi.cols * 3;
    }

    if (red_cnt > blue_cnt) {
        return BLOB_RED;
    } else {
        return BLOB_BLUE;
    }
}

Armors Tradition::Find_light(Mat &src, const cv::Rect& roi) { // 默认参数为空ROI
    Armors armors2;
    cv::Mat src_roi;
    
    if(is_get_from_ds){
        enemy_color = ds.color;
    }

    // 如果ROI为空，则处理整张图像
    if (roi.empty()) {
        src_roi = src; // 直接使用原图
    } else {
        src_roi = src(roi); // 提取ROI子图像
    }

    // 绘制ROI框
    // if (!roi.empty()) {
    //     cv::rectangle(src, roi, cv::Scalar(255, 0, 0), 2); // 蓝色框，线宽为2
    // }

    cv::Mat color_channel;
    cv::Mat src_bin_light, src_bin_dim;
    std::vector<cv::Mat> channels;       // 通道拆分
    
    armors2.clear();
    cv::split(src_roi, channels); // 处理ROI子图像或原图

    if (enemy_color == ENEMY_BLUE) {         
        color_channel = channels[0];        
    } else if (enemy_color == ENEMY_RED) {    
        color_channel = channels[2];        
    }

    int light_threshold;
    if(enemy_color == ENEMY_BLUE){
        light_threshold = 200;
    }else{
        light_threshold = 165;
    }
    cv::threshold(color_channel, src_bin_light, light_threshold, 255, THRESH_BINARY);
    imagePreProcess1(src_bin_light);                                  

    cv::threshold(color_channel, src_bin_dim, 100, 255, THRESH_BINARY); 
    imagePreProcess(src_bin_dim);                                  
    
    // imshow("light",src_bin_light);
    // imshow("dim",src_bin_dim);

    std::vector<std::vector<cv::Point>> light_contours_light, light_contours_dim;
    LightBlobs light_blobs_light, light_blobs_dim;
    std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;

    cv::findContours(src_bin_light, light_contours_light, hierarchy_light, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    
    // 如果ROI不为空，转换轮廓坐标到原图
    if (!roi.empty()) {
        for (auto& contour : light_contours_light) {
            for (auto& p : contour) {
                p.x += roi.x;
                p.y += roi.y;
            }
        }
        for (auto& contour : light_contours_dim) {
            for (auto& p : contour) {
                p.x += roi.x;
                p.y += roi.y;
            }
        }
    }

    // 处理light_contours_light中的轮廓
    for (int i = 0; i < light_contours_light.size(); i++) {
        if (hierarchy_light[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
            if (isValidLightBlob(light_contours_light[i], rect)) {
                light_blobs_light.emplace_back(
                    rect, areaRatio(light_contours_light[i], rect), get_blob_color(src, rect), light_contours_light[i].size()
                );
            }
        }
    }

    // 处理light_contours_dim中的轮廓
    for (int i = 0; i < light_contours_dim.size(); i++) {
        if (hierarchy_dim[i][2] == -1) {
            cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
            if (isValidLightBlob(light_contours_dim[i], rect)) {
                light_blobs_dim.emplace_back(
                    rect, areaRatio(light_contours_dim[i], rect), get_blob_color(src, rect), light_contours_dim[i].size()
                );
            }
        }
    }

    std::vector<int> light_to_merge, dim_to_merge;
    for (int l = 0; l != light_blobs_light.size(); l++) {
        for (int d = 0; d != light_blobs_dim.size(); d++) {
            if (isSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
                bool keep_light = false, keep_dim = false;
                if (light_blobs_light[l].rect.size.area() < 300) {
                    keep_light = light_blobs_light[l].rect.size.area() >= light_blobs_dim[d].rect.size.area();
                } else {
                    keep_light = light_blobs_light[l].area_ratio >= light_blobs_dim[d].area_ratio;
                }
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
    

    // 清理
    light_contours_light.clear();
    light_contours_dim.clear();
    light_blobs_light.clear();
    light_blobs_dim.clear();
    light_contours_light.clear();
    light_contours_dim.clear();
    channels.clear();

    // 绘制灯条到原图（坐标已正确）
    // for (const auto& blob : light_blobs) {
    //     cv::Point2f vertices[4];
    //     blob.rect.points(vertices);
    //     for (int i = 0; i < 4; i++) {
    //         cv::line(src, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0), 2);
    //     }
    //     cv::circle(src, blob.rect.center, 5, cv::Scalar(0,0,255), -1);
    // }

    if (light_blobs.size() > 1) {    
        armors2 = Find_armor(src_bin_light, light_blobs); 
        light_blobs.clear();
    }

    return armors2;
}