#include "usbcamera.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "logger.hpp"
#include "exiter.hpp"

using namespace std::chrono_literals;

namespace io
{
USBCamera::USBCamera(const std::string & open_name, const std::string & yaml_path)
: open_name_(open_name), open_count_(0), quit_(false), ok_(false), queue_(1)
{
    LoadConfig(yaml_path);
    TryOpen();

    daemon_thread_ = std::thread{[this] {
        int consecutive_fails = 0;
        const int max_consecutive_fails = 5;

        while (!quit_) {
            if (tools::exit_requested()) {
                quit_ = true;
                break;
            }
            std::this_thread::sleep_for(100ms);
            if (ok_) {
                consecutive_fails = 0;
                continue;
            }

            if (capture_thread_.joinable()) {
                tools::logger()->info("Capture thread stopped, attempting recovery for {}", open_name_);
                capture_thread_.join();
            }

            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                Close();
            }

            TryOpen();
            ++consecutive_fails;
            if (open_count_ > 80) {
                tools::logger()->warn("Give up to open {} USB camera after 20 attempts", open_name_);
                quit_ = true;
                break;
            }

            if (consecutive_fails > max_consecutive_fails) {
                tools::logger()->warn("Too many consecutive failures for {}, waiting before retry...", open_name_);
                std::this_thread::sleep_for(500ms);
            }
        }
    }};
}

USBCamera::~USBCamera()
{
    quit_ = true;
    {
        std::lock_guard<std::mutex> lock(cap_mutex_);
        Close();
    }
    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    tools::logger()->info("USBCamera destructed.");
}

cv::Mat USBCamera::read()
{
    CameraData data;
    while (!quit_) {
        if (tools::exit_requested()) {
            quit_ = true;
            break;
        }
        if (queue_.pop_for(data, 50ms)) {
            return data.img;
        }
    }
    return cv::Mat();
}

void USBCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
    CameraData data;
    while (!quit_) {
        if (tools::exit_requested()) {
            quit_ = true;
            break;
        }
        if (queue_.pop_for(data, 50ms)) {
            img = data.img;
            timestamp = data.timestamp;
            return;
        }
    }
    img.release();
    timestamp = std::chrono::steady_clock::time_point{};
}

void USBCamera::LoadConfig(const std::string & yaml_path)
{
    auto yaml = YAML::LoadFile(yaml_path);
    new_image_height_ = 720.0;
    new_image_width_ = 1280.0;
    new_usb_exposure_ = yaml["USBCamera"]["new_usb_exposure"].as<double>();
    usb_frame_rate_ = yaml["USBCamera"]["usb_frame_rate"].as<int>();
}

void USBCamera::Open()
{
    if (tools::exit_requested()) {
        quit_ = true;
        return;
    }
    std::lock_guard<std::mutex> lock(cap_mutex_);
    std::string true_device_name = "/dev/" + open_name_;
    tools::logger()->info("Attempting to open USB camera: {} (attempt {})", true_device_name, open_count_);

    cap_.open(true_device_name, cv::CAP_V4L);
    if (!cap_.isOpened()) {
        tools::logger()->warn("Failed to open USB camera at {}", true_device_name);
        return;
    }

    tools::logger()->info("Successfully opened {} at {}", open_name_, true_device_name);
    std::this_thread::sleep_for(100ms);

    sharpness_ = cap_.get(cv::CAP_PROP_SHARPNESS);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FPS, usb_frame_rate_);
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);

    if (open_name_ == "video0") {
        device_name = "right";
    } else if (open_name_ == "video2") {
        device_name = "left";
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, new_image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, new_image_height_);
    cap_.set(cv::CAP_PROP_EXPOSURE, new_usb_exposure_);

    tools::logger()->info("{} camera opened successfully (sharpness: {})", device_name, sharpness_);
    tools::logger()->info("Camera fps: {}", cap_.get(cv::CAP_PROP_FPS));

    capture_thread_ = std::thread{[this] {
        ok_ = true;
        int frame_count = 0;
        int consecutive_read_fails = 0;

        std::this_thread::sleep_for(50ms);
        while (!quit_) {
            if (tools::exit_requested()) {
                quit_ = true;
                break;
            }
            std::this_thread::sleep_for(1ms);

            cv::Mat img;
            bool success = false;
            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                if (!cap_.isOpened()) {
                    tools::logger()->warn("Camera device closed in capture thread");
                    break;
                }
                success = cap_.read(img);
            }

            if (!success) {
                ++consecutive_read_fails;
                if (consecutive_read_fails == 1) {
                    tools::logger()->warn("Read failure detected for {}", open_name_);
                }
                if (consecutive_read_fails > 5) {
                    tools::logger()->warn("Repeated read failures for {}, exiting capture thread", open_name_);
                    break;
                }
                continue;
            }

            consecutive_read_fails = 0;
            ++frame_count;
            auto timestamp = std::chrono::steady_clock::now();
            queue_.push({img, timestamp});
        }
        ok_ = false;
        tools::logger()->info("Capture thread exited for {} after {} frames", open_name_, frame_count);
    }};
}

void USBCamera::TryOpen()
{
    try {
        Open();
        ++open_count_;
    } catch (const std::exception & e) {
        tools::logger()->warn("{}", e.what());
    }
}

void USBCamera::Close()
{
    if (cap_.isOpened()) {
        cap_.release();
        tools::logger()->info("USB camera released.");
    }
}

}  // namespace io
