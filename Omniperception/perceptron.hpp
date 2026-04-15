#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <chrono>
#include <list>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

#include "find_armor.h"
#include "usbcamera.hpp"
#include "thread_safe_queue.hpp"

namespace Omniperception
{

// 检测结果结构体
struct DetectionResult
{
  Armors armors;
  std::string camera_name;
  double delta_pitch = 0.0;
  double delta_yaw = 0.0;
  std::chrono::steady_clock::time_point timestamp;
};

class Perceptron
{
public:
  Perceptron(
    io::USBCamera * usbcam1, io::USBCamera * usbcam2,
    int enemy_color, const std::string & config_path);

  ~Perceptron();

  void parallel_infer(io::USBCamera * cam, Tradition & tradition, int enemy_color);

  // 获取检测队列中的所有结果
  std::vector<DetectionResult> get_detection_queue();

private:
  void draw_and_visualize(const cv::Mat & img, const Armors & armors,
                          const std::string & window_name);
  
  // 排序和过滤检测结果（移除armors为空的成员）
  std::vector<DetectionResult> filterAndSortResults(
    const std::vector<DetectionResult> & raw_results);
  
  // 线程安全队列用于存储检测结果
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;
  
  std::vector<std::thread> threads_;

  Tradition tradition1_;
  Tradition tradition2_;

  bool stop_flag_;
  bool show_visualization_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
};

}  // namespace Omniperception
#endif