#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include "ArmorTypes.h"
#include <opencv2/dnn.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <mutex>

/**
 * @file ArmorDetector.h
 * @brief 当前使用的灯条几何检测接口；可通过 kLightBarsOnly 临时跳过 ONNX 数字分类。
 * 检测仅产生二维图像信息；三维位置和板朝向由后续 Slover 填充。
 * 已确认调用方：Backbone 的 AutoArmor、Omniperception 和键盘测试入口。
 * 这是当前构建使用的旧检测接口，随后经 adapter 进入 sentry 追踪，不属于被排除的历史滤波链。
 */

/**
 * @brief 依次完成灯条筛选、两灯配对、数字 ROI 提取、分类及数字小图叠加。
 *
 * 图像应为非空 8 位三通道，坐标单位为像素，左上为原点、x 向右、y 向下。
 * 实现的找灯条部分将通道 0 当蓝色、通道 2 当红色；数字灰度转换却使用
 * COLOR_RGB2GRAY，接入相机时应按实际源码核对通道约定，不能仅凭类名推断颜色顺序。
 */
class ArmorDetector {
public:
    /// false：加载模型并识别数字；临时只检查灯条时改为 true 并重新编译。
    static constexpr bool kLightBarsOnly = false;
    /// 供原追踪/协议接口使用的统一占位类别，不是数字识别结果，也不能区分不同目标身份。
    static constexpr const char* kLightBarsPlaceholderId = "1";

    /**
     * @brief 保存检测参数；仅灯条模式不加载网络/类别表，否则执行原来的数字模型加载。
     * @param model_path ONNX 模型路径，由 cv::dnn::readNetFromONNX 加载。
     * @param label_path 按网络输出索引排列的文本标签，每行一个类别。
     * @param classify_threshold 最大 softmax 概率的保留门槛，通常在 [0,1]。
     * @param binary_threshold 找灯条时对敌方颜色通道执行固定阈值二值化的门槛。
     * @param ignore_classes 分类后应删除的标签，例如 negative、base。
     * @param adaptive_block_size 数字 ROI 自适应阈值邻域边长，OpenCV 要求为大于 1 的奇数。
     * @param adaptive_c 数字 ROI 局部阈值中的常数 C，实际阈值为加权均值减 C。
     * @note 模型错误可能抛 OpenCV 异常；当前标签读取没有单独的打开失败检查，
     * 标签数量和模型输出维数需要由配置方保证一致。
     */
    ArmorDetector(
        const std::string &model_path, const std::string &label_path,
        const double classify_threshold, const int binary_threshold = 180,
        const std::vector<std::string> &ignore_classes = {},
        const int adaptive_block_size = 19, const int adaptive_c = -3);

    /**
     * @brief 一帧检测主入口；不足两个灯条或没有配对结果时返回空列表。
     * @param img 输入图像；仅灯条模式叠加框和 LIGHTS ONLY，原模式叠加数字 ROI 画中画。
     * @param enemy_color 敌方颜色编码：ENEMY_BLUE=0、ENEMY_RED=1。
     * @return 开启仅灯条模式时直接返回几何配对结果，不做数字分类或类别过滤；
     * number 为占位类别，confidence=1 仅表示几何配对通过，并非分类概率，number_img 为空。
     * 关闭 kLightBarsOnly 后恢复按分类概率、忽略类别及大小板/类别规则筛选。
     * 输出含 points、center、type、number_img、number、confidence；不包含已解算的位姿。
     * 本函数本身不做相机采集、不更新目标跟踪，也不生成控制或发射数据。
     */
    Armors Detect(Mat &img, int enemy_color);

private:
    /// 保存敌色，按颜色通道阈值、轮廓面积、长宽比和倾斜角提取灯条；不生成装甲板类别。
    LightBlobs FindLight(Mat &src, int enemy_color);
    /// 要求至少两个灯条；检查颜色/几何相容性和中间灯条，精修四点并判断大小板。
    Armors FindArmor(Mat &src, LightBlobs &light_blobs);
    /**
     * @brief 将每块板按 [左下、左上、右上、右下] 四点做透视展开。
     * 大小板展开宽度不同，中央数字 ROI 最终成为 28x28 单通道二值图，写回 number_img。
     * src 在此阶段只读；armors 的四个有效角点及 type 必须已经建立。
     */
    void ExtractNumbers(const Mat &src, Armors &armors);
    /// 逐板执行网络推理和 softmax，写标签/置信度，并原地删除低分、忽略及类型不匹配结果。
    void Classify(Armors &armors);
    /// 将 number_img 放大后叠加在原图板旁，边界不够时跳过；完整检测框由外层按需绘制。
    void VisualizeResult(Mat &src, const Armors &armors);

    // 灯条使用固定通道阈值；数字图像另外使用自适应阈值，两者用途不同。
    int enemy_color_;            ///< 当前 Detect/FindLight 调用使用的敌色编码，是会被更新的成员。
    int threshold_;              ///< 灯条颜色通道的固定二值化阈值，通常对应 0~255 灰度范围。
    int adaptive_block_size_;    ///< 数字 ROI 高斯自适应阈值邻域边长，像素。
    int adaptive_c_;             ///< 数字 ROI 阈值计算中减去的 C，可取负值。

    cv::dnn::Net net_;                        ///< 原数字模式加载的 ONNX 网络；仅灯条模式保持为空，不执行推理。
    std::vector<std::string> class_names_;    ///< 原数字模式的输出索引到标签映射；仅灯条模式不读取标签表。
    std::vector<std::string> ignore_classes_; ///< 无论置信度多高都要剔除的类别名。
    double classify_threshold_;              ///< 保留结果所需的最大类别概率下限，不是图像二值化阈值。
    std::mutex mutex_;                       ///< 仅保护 net_.setInput/forward；不代表整个 Detect 可并发共享调用。
};

#endif
