#include "player.hpp"

#include <algorithm>
#include <iterator>
#include <cmath>
#include <stdexcept>

#include <fmt/core.h>

#include "logger.hpp"

namespace tools
{
Player::Player(const std::string & input_path)
{
  video_path_ = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);

  video_.open(video_path_);
  text_.open(text_path);

  t0_ = std::chrono::steady_clock::now();

  fps_ = video_.get(cv::CAP_PROP_FPS);
  total_frames_ = static_cast<int>(video_.get(cv::CAP_PROP_FRAME_COUNT));

  // 某些编码格式 (MJPG/FFV1 等) CAP_PROP_FRAME_COUNT 返回 0，
  // 改为统计 txt 文件行数作为帧数（每行一帧）
  if (total_frames_ <= 0 && text_.is_open()) {
    total_frames_ = 0;
    std::string line;
    while (std::getline(text_, line)) ++total_frames_;
    text_.clear();               // 清除 eof 标志
    text_.seekg(0);              // 回到文件开头
  }

  if (is_opened()) {
    tools::logger()->info(
      "Player: opened {}, {}x{} {:.1f}fps {} frames",
      input_path,
      static_cast<int>(video_.get(cv::CAP_PROP_FRAME_WIDTH)),
      static_cast<int>(video_.get(cv::CAP_PROP_FRAME_HEIGHT)),
      fps_, total_frames_);
  } else {
    tools::logger()->error("Player: failed to open {}", input_path);
  }
}

bool Player::read(cv::Mat & img, Data_Save & ds)
{
  video_.read(img);
  if (img.empty()) return false;

  double since_begin, yaw, pitch, roll, go_back, armor_or_buff;
  int color;
  if (!(text_ >> since_begin >> yaw >> pitch >> roll >> go_back >> armor_or_buff >> color) ||
      !std::isfinite(since_begin) || since_begin < 0 || !std::isfinite(yaw) ||
      !std::isfinite(pitch) || !std::isfinite(roll) || !std::isfinite(go_back) ||
      !std::isfinite(armor_or_buff) || (color != 0 && color != 1) ||
      since_begin >= std::chrono::duration<double>(time_point::max() - t0_).count()) {
    img.release();
    return false;
  }

  ds.timestamp = t0_ + std::chrono::duration_cast<time_point::duration>(
    std::chrono::duration<double>(since_begin));
  ds.camera_yaw_angle = yaw;
  ds.camera_pitch_angle = pitch;
  ds.camera_roll_angle = roll;
  ds.go_back = go_back;
  ds.armor_or_buff = armor_or_buff;
  ds.color = color;

  return true;
}

void Player::seek(int frame_index)
{
  if (frame_index < 0 || (total_frames_ > 0 && frame_index >= total_frames_))
    throw std::out_of_range("recording frame index is out of range");
  // Some backends report seek success but keep the old frame. Reopen and skip sequentially.
  if (!video_.open(video_path_)) throw std::runtime_error("cannot reopen recording video");
  for (int i = 0; i < frame_index; ++i)
    if (!video_.grab()) throw std::out_of_range("recording video ended during seek");

  // 跳过文本文件中对应的行
  text_.clear();   // 清除eof标志
  text_.seekg(0);
  for (int i = 0; i < frame_index; i++) {
    std::string line;
    if (!std::getline(text_, line))
      throw std::out_of_range("recording metadata ended during seek");
  }
}

bool Player::is_opened() const { return video_.isOpened() && text_.is_open(); }

double Player::fps() const { return fps_; }

int Player::total_frames() const { return total_frames_; }

}  // namespace tools
