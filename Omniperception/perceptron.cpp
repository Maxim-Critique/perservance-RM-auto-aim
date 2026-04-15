#include "perceptron.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include "find_armor.h"
#include "logger.hpp"

namespace Omniperception
{
Perceptron::Perceptron(
  io::USBCamera * usbcam1, io::USBCamera * usbcam2,
  int enemy_color, const std::string & config_path)
: detection_queue_(100), stop_flag_(false), show_visualization_(false)
{
  // 验证摄像头指针
  if (!usbcam1 || !usbcam2) {
    tools::logger()->error("Invalid camera pointers in Perceptron initialization!");
    throw std::runtime_error("Camera pointers are null");
  }

  // 读取YAML配置中的visualization参数
  try {
    auto yaml = YAML::LoadFile(config_path);
    show_visualization_ = yaml["Camera"]["show"].as<int>();
    tools::logger()->info("Camera visualization enabled: {}", show_visualization_);
  } catch (const std::exception & e) {
    tools::logger()->warn("Failed to load visualization config: {}", e.what());
    show_visualization_ = false;
  }

  tools::logger()->info("Initializing Perceptron with 2 cameras using Recognition module...enemy_color: {}", enemy_color);

  std::this_thread::sleep_for(std::chrono::seconds(2));
  std::cout<<"enemy_color: "<<enemy_color<<std::endl;
  // 创建两个线程进行并行推理（每个相机对应一个线程）
  threads_.emplace_back([this, usbcam1, enemy_color] { 
    parallel_infer(usbcam1, this->tradition1_, enemy_color); 
  });
  threads_.emplace_back([this, usbcam2, enemy_color] { 
    parallel_infer(usbcam2, this->tradition2_, enemy_color); 
  });

  // 给线程初始化的时间
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  tools::logger()->info("Perceptron initialized with 2 cameras using Recognition module.");
  
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置退出标志
  }
  condition_.notify_all();  // 唤醒所有等待的线程

  // 等待线程结束
  for (auto & t : threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  tools::logger()->info("Perceptron destructed.");
}

// 将并行推理逻辑移动到类成员函数
void Perceptron::parallel_infer(
  io::USBCamera * cam, Tradition & tradition, int enemy_color)
{
  if (!cam) {
    tools::logger()->error("Camera pointer is null!");
    return;
  }
  try {
    while (true) {
      std::chrono::steady_clock::time_point ts;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;  // 检查是否需要退出
      }

      cv::Mat usb_img = cam->read();
      if (usb_img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      // 使用Recognition模块进行推理
      // 找灯条
      LightBlobs light_blobs = tradition.FindLight(usb_img, enemy_color);
      
      // 找装甲板
      Armors armors;
      if (!light_blobs.empty()) {
        armors = tradition.FindArmor(usb_img, light_blobs);
        
        // 提取数字和分类
        if (!armors.empty()) {
          classifier.extractNumbers(usb_img, armors);
          classifier.classify(armors);
        }
      }
      
      // 创建检测结果
      DetectionResult result;
      result.armors = armors;
      result.camera_name = cam->device_name;
      result.timestamp = ts;
      
      // 根据相机名称设置 delta_yaw
      if (cam->device_name == "left") {
        result.delta_yaw = 90.0;
      } else if (cam->device_name == "right") {
        result.delta_yaw = -90.0;
      }
      result.delta_pitch = 0.0;
      
      // 存入队列
      detection_queue_.push(result);
      
      // 根据配置决定是否绘制和可视化
      if (show_visualization_) {
        draw_and_visualize(usb_img, armors, cam->device_name);
      }

    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in parallel_infer: {}", e.what());
  }
}

void Perceptron::draw_and_visualize(const cv::Mat & img, const Armors & armors,
                                     const std::string & window_name)
{
  // 绘制检测框
  cv::Mat vis_img = img.clone();
  for (const auto & armor : armors) {
    // 绘制关键点
    for (const auto & point : armor.points) {
      cv::circle(vis_img, point, 3, {0, 255, 0}, -1);
    }
    // 绘制置信度
    auto info = fmt::format("{:.2f}", armor.confidence);
    cv::putText(vis_img, info, armor.center, cv::FONT_HERSHEY_SIMPLEX, 
                0.6, {0, 255, 0}, 2);
  }

  // Resize到小窗口
  cv::Mat resized;
  cv::resize(vis_img, resized, {}, 0.5, 0.5);
  
  // 显示
  cv::imshow(window_name, resized);
}

std::vector<DetectionResult> Perceptron::filterAndSortResults(
  const std::vector<DetectionResult> & raw_results)
{
  std::vector<DetectionResult> filtered;
  
  // 过滤：移除armors为空的DetectionResult
  for (const auto & result : raw_results) {
    if (!result.armors.empty()) {
      filtered.push_back(result);
    }
  }
  
  // 排序：按armors数量降序排列（优先返回检测到更多装甲板的结果）
  std::sort(filtered.begin(), filtered.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.size() > b.armors.size();
    });
  
  return filtered;
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;

  // 非阻塞地弹出队列中的所有元素
  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  // 排序和过滤检测结果
  return filterAndSortResults(result);
}

}  // namespace Omniperception
