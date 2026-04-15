#include "usbcamera.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "logger.hpp"

using namespace std::chrono_literals;

namespace io
{
USBCamera::USBCamera(const std::string & open_name, const std::string & yaml_path)
: open_name_(open_name), open_count_(0), quit_(false), ok_(false), queue_(1)
{
  load_config(yaml_path);
  try_open();

  // 守护线程
  daemon_thread_ = std::thread{[this] {
    int consecutive_fails = 0;
    const int MAX_CONSECUTIVE_FAILS = 5;  // 允许 5 次连续失败后才放弃
    
    while (!quit_) {
      std::this_thread::sleep_for(100ms);

      if (ok_) {
        consecutive_fails = 0;  // 重置失败计数
        continue;
      }

      // 如果取图线程失败或设备掉线，尝试恢复
      if (capture_thread_.joinable()) {
        tools::logger()->info("Capture thread stopped, attempting recovery for {}", open_name_);
        capture_thread_.join();
      }

      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        close();
      }
      
      try_open();
      consecutive_fails++;

      // 20 次失败后放弃
      if (open_count_ > 80) {
        tools::logger()->warn("Give up to open {} USB camera after 20 attempts", open_name_);
        quit_ = true;
        break;
      }

      if (consecutive_fails > MAX_CONSECUTIVE_FAILS) {
        tools::logger()->warn("Too many consecutive failures for {}, waiting before retry...", open_name_);
        std::this_thread::sleep_for(500ms);  // 等待 500ms 后重试
      }
    }
  }};
}

USBCamera::~USBCamera()
{
  quit_ = true;
  {
    std::lock_guard<std::mutex> lock(cap_mutex_);
    close();
  }
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  tools::logger()->info("USBCamera destructed.");
}

cv::Mat USBCamera::read()
{
  CameraData data;
  queue_.pop(data);

  return data.img;
}


void USBCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}


void USBCamera::load_config(const std::string & yaml_path)
{
  auto yaml = YAML::LoadFile(yaml_path);
  new_image_height_ = yaml["USBCamera"]["new_image_height"].as<double>();
  new_image_width_ = yaml["USBCamera"]["new_image_width"].as<double>();
  new_usb_exposure_ = yaml["USBCamera"]["new_usb_exposure"].as<double>();
  usb_frame_rate_ = yaml["USBCamera"]["usb_frame_rate"].as<int>();
}

void USBCamera::open()
{
  std::lock_guard<std::mutex> lock(cap_mutex_);
  std::string true_device_name = "/dev/" + open_name_;
  tools::logger()->info("Attempting to open USB camera: {} (attempt {})", true_device_name, open_count_);
  
  cap_.open(true_device_name, cv::CAP_V4L);
  if (!cap_.isOpened()) {
    tools::logger()->warn("Failed to open USB camera at {}", true_device_name);
    return;
  }
  
  tools::logger()->info("Successfully opened {} at {}", open_name_, true_device_name);
  
  // 等待摄像头初始化
  std::this_thread::sleep_for(100ms);
  
  sharpness_ = cap_.get(cv::CAP_PROP_SHARPNESS);
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FPS, usb_frame_rate_);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);

  // 根据设备名称判断是哪个摄像头
  if (open_name_ == "video0") {
    device_name = "right";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, new_image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, new_image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, new_usb_exposure_);
  } else if (open_name_ == "video2") {
    device_name = "left";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, new_image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, new_image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, new_usb_exposure_);
  } else if (open_name_ == "video4") {
    device_name = "back";
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, new_image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, new_image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, new_usb_exposure_);
  } 

  tools::logger()->info("{} camera opened successfully (sharpness: {})", device_name, sharpness_);
  tools::logger()->info("Camera fps: {}", cap_.get(cv::CAP_PROP_FPS));

  // 取图线程
  capture_thread_ = std::thread{[this] {
    ok_ = true;
    int frame_count = 0;
    int consecutive_read_fails = 0;
    
    std::this_thread::sleep_for(50ms);
    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      cv::Mat img;
      bool success;
      {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        if (!cap_.isOpened()) {
          tools::logger()->warn("Camera device closed in capture thread");
          break;
        }
        success = cap_.read(img);
      }

      if (!success) {
        consecutive_read_fails++;
        if (consecutive_read_fails == 1) {
          tools::logger()->warn("Read failure detected for {}", open_name_);
        }
        if (consecutive_read_fails > 5) {
          tools::logger()->warn("Repeated read failures for {}, exiting capture thread", open_name_);
          break;
        }
        continue;  // 继续尝试而不是立即退出
      }

      consecutive_read_fails = 0;  // 重置失败计数
      frame_count++;
      auto timestamp = std::chrono::steady_clock::now();
      queue_.push({img, timestamp});
    }
    ok_ = false;
    tools::logger()->info("Capture thread exited for {} after {} frames", open_name_, frame_count);
  }};
}

void USBCamera::try_open()
{
  try {
    open();
    open_count_++;
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void USBCamera::close()
{
  if (cap_.isOpened()) {
    cap_.release();
    tools::logger()->info("USB camera released.");
  }
}

}  // namespace io
