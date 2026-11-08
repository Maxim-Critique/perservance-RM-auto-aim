#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ArmorDetector.h"
#include "ArmorTypes.h"
#include "thread_safe_queue.hpp"
#include "usbcamera.hpp"

namespace Omniperception
{

// 检测结果结构体
struct DetectionResult
{
    Armors armors;
    double delta_pitch = 0.0;
    double delta_yaw = 0.0;
};

class Perceptron
{
public:
    Perceptron(
        io::USBCamera * usbcam1,
        io::USBCamera * usbcam2,
        int enemy_color,
        const std::string & config_path);

    ~Perceptron();

    void ParallelInfer(io::USBCamera * cam, ArmorDetector & detector, int enemy_color);

    void SetActive(bool active);

    // 获取检测队列中的所有结果
    std::vector<DetectionResult> GetDetectionQueue();

private:
    void DrawAndVisualize(
        const cv::Mat & img,
        const Armors & armors,
        const std::string & window_name);

    // 计算delta角度并将结果存入检测队列
    void CalculateDeltaAngles(const Armors & armors, io::USBCamera * cam);

    // 根据points的第1和第3个点向外延伸armor的points
    void ExpandArmorPoints(Armors & armors);

    // 线程安全队列用于存储检测结果
    tools::ThreadSafeQueue<DetectionResult> detection_queue_;

    std::vector<std::thread> threads_;
    std::unique_ptr<ArmorDetector> detector1_;
    std::unique_ptr<ArmorDetector> detector2_;

    bool stop_flag_;
    bool active_;
    bool show_visualization_;
    double fov_h_;
    double fov_v_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
};

}  // namespace Omniperception
#endif