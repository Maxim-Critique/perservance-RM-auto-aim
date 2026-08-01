#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <atomic>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "Base.h"
#include "thread_safe_queue.hpp"

namespace tools
{
class Recorder
{
public:
  Recorder(double segment_duration = 60.0, double fps = 30);
  ~Recorder();
  void record(const cv::Mat & img, const Data_Save & ds);

private:
  struct FrameData
  {
    cv::Mat img;
    Data_Save ds;
  };
  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  double segment_duration_;
  int segment_index_;
  cv::Size img_size_;
  std::string folder_path_;
  std::string base_name_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  std::chrono::steady_clock::time_point segment_start_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;

  void init(const cv::Mat & img);
  void save_to_file();
  void rotate_segment();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
