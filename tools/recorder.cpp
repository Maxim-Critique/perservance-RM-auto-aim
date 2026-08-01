#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "logger.hpp"

namespace tools
{
Recorder::Recorder(double segment_duration, double fps)
: init_(false), stop_thread_(false), fps_(fps), segment_duration_(segment_duration),
  segment_index_(0), queue_(1)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;
  segment_start_time_ = start_time_;

  folder_path_ = "records";
  base_name_ = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());

  std::filesystem::create_directory(folder_path_);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), Data_Save{}});
  if (saving_thread_.joinable()) saving_thread_.join();

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }

    img_size_ = frame.img.size();

    // 写入视频文件
    video_writer_.write(frame.img);

    // 写入Data_Save结构体中的所有变量
    auto since_begin = tools::delta_time(frame.ds.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {} {} {}\n",
      since_begin,
      frame.ds.camera_yaw_angle,
      frame.ds.camera_pitch_angle,
      frame.ds.camera_roll_angle,
      frame.ds.go_back,
      frame.ds.armor_or_buff,
      frame.ds.color);

    // 检查是否需要切换分段
    auto since_segment = tools::delta_time(frame.ds.timestamp, segment_start_time_);
    if (since_segment >= segment_duration_) {
      rotate_segment();
    }
  }
}

void Recorder::record(const cv::Mat & img, const Data_Save & ds)
{
  if (img.empty()) return;
  if (!init_) init(img);

  auto since_last = tools::delta_time(ds.timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = ds.timestamp;
  queue_.push({img.clone(), ds});
}

void Recorder::init(const cv::Mat & img)
{
  img_size_ = img.size();
  text_path_ = fmt::format("{}/{}_0000.txt", folder_path_, base_name_);
  video_path_ = fmt::format("{}/{}_0000.avi", folder_path_, base_name_);
  text_writer_.open(text_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  saving_thread_ = std::thread(&Recorder::save_to_file, this);
  init_ = true;
}

void Recorder::rotate_segment()
{
  text_writer_.close();
  video_writer_.release();

  segment_index_++;
  text_path_ = fmt::format("{}/{}_{:04d}.txt", folder_path_, base_name_, segment_index_);
  video_path_ = fmt::format("{}/{}_{:04d}.avi", folder_path_, base_name_, segment_index_);

  text_writer_.open(text_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img_size_);
  segment_start_time_ = std::chrono::steady_clock::now();

  tools::logger()->info("Recorder: switched to segment {}", segment_index_);
}

}  // namespace tools
