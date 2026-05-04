#ifndef IO__USBCamera_HPP
#define IO__USBCamera_HPP

#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "thread_safe_queue.hpp"

namespace io
{
class USBCamera
{
public:
    USBCamera(const std::string & open_name, const std::string & yaml_path);
    ~USBCamera();

    cv::Mat read();
    void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

    std::string device_name;
    double new_image_width_;
    double new_image_height_;

private:
    struct CameraData
    {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
    };

    std::mutex cap_mutex_;
    cv::VideoCapture cap_;
    std::string open_name_;

    int usb_frame_rate_;
    int sharpness_;
    double new_usb_exposure_;
    int open_count_;
    bool quit_;
    bool ok_;

    std::thread capture_thread_;
    std::thread daemon_thread_;
    tools::ThreadSafeQueue<CameraData> queue_;

    void TryOpen();
    void Open();
    void Close();
    void LoadConfig(const std::string & yaml_path);
};

}  // namespace io

#endif
