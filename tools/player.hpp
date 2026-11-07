#ifndef TOOLS__PLAYER_HPP
#define TOOLS__PLAYER_HPP

#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>

#include "Base.h"

namespace tools
{
class Player
{
public:
  // input_path: 不含扩展名的基础路径，例如 "records/2026-08-01_12-34-56_0000"
  // 将自动打开 {input_path}.avi 和 {input_path}.txt
  Player(const std::string & input_path);

  // 读取下一帧，返回false表示到达文件末尾
  bool read(cv::Mat & img, Data_Save & ds);

  // 跳转到指定帧号（从头开始计数）
  void seek(int frame_index);

  // 是否成功打开
  bool is_opened() const;

  // 视频fps
  double fps() const;

  // 总帧数
  int total_frames() const;

private:
  std::string video_path_;
  cv::VideoCapture video_;
  std::ifstream text_;
  std::chrono::steady_clock::time_point t0_;
  double fps_;
  int total_frames_;
};

}  // namespace tools

#endif  // TOOLS__PLAYER_HPP
