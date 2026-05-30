#include "perceptron.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include "find_armor.h"
#include "logger.hpp"

namespace Omniperception
{

Perceptron::Perceptron(
    io::USBCamera * usbcam1,
    io::USBCamera * usbcam2,
    io::USBCamera * usbcam3,
    int enemy_color,
    const std::string & config_path)
: detection_queue_(100),
    stop_flag_(false),
    active_(true),
    show_visualization_(false),
    fov_h_(27.0),
    fov_v_(40.9)
{
    if (!usbcam1 || !usbcam2 || !usbcam3) {
        tools::logger()->error("Invalid camera pointers in Perceptron initialization!");
        throw std::runtime_error("Camera pointers are null");
    }

    try {
        auto yaml = YAML::LoadFile(config_path);
        show_visualization_ = yaml["Camera"]["show"].as<int>();
        fov_h_ = yaml["USBCamera"]["new_fov_h"].as<double>();
        fov_v_ = yaml["USBCamera"]["new_fov_v"].as<double>();
        tools::logger()->info("Camera visualization enabled: {}", show_visualization_);
    } catch (const std::exception & e) {
        tools::logger()->warn("Failed to load visualization config: {}", e.what());
        show_visualization_ = false;
    }

    tools::logger()->info("Initializing Perceptron with 2 cameras using Recognition module...enemy_color: {}", enemy_color);

    threads_.emplace_back([this, usbcam1, enemy_color] {
        ParallelInfer(usbcam1, tradition1_, enemy_color);
    });
    threads_.emplace_back([this, usbcam2, enemy_color] {
        ParallelInfer(usbcam2, tradition2_, enemy_color);
    });
    threads_.emplace_back([this, usbcam3, enemy_color] {
        ParallelInfer(usbcam3, tradition3_, enemy_color);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    tools::logger()->info("Perceptron initialized with 3 cameras using Recognition module.");
}

Perceptron::~Perceptron()
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_flag_ = true;
    }
    condition_.notify_all();

    for (auto & thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    tools::logger()->info("Perceptron destructed.");
}

void Perceptron::ParallelInfer(io::USBCamera * cam, Tradition & tradition, int enemy_color)
{
    if (!cam) {
        tools::logger()->error("Camera pointer is null!");
        return;
    }

    try {
        while (true) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] { return stop_flag_ || active_; });
                if (stop_flag_) break;
            }

            cv::Mat usb_img = cam->read();
            if (usb_img.empty()) continue;

            Armors armors;
            LightBlobs light_blobs = tradition.FindLight(usb_img, enemy_color);
            if (!light_blobs.empty()) {
                armors = tradition.FindArmor(usb_img, light_blobs);
                if (!armors.empty()) {
                    classifier.extractNumbers(usb_img, armors);
                    classifier.classify(armors);
                    if (!armors.empty()) CalculateDeltaAngles(armors, cam);
                }
            }

            if (show_visualization_) 
                DrawAndVisualize(usb_img, armors, cam->device_name);
            
        }
    } catch (const std::exception & e) {
        tools::logger()->error("Exception in parallelInfer: {}", e.what());
    }
}

void Perceptron::SetActive(bool active)
{
    {
        std::unique_lock<std::mutex> lock(mutex_);
        active_ = active;
    }
    condition_.notify_all();
}

void Perceptron::CalculateDeltaAngles(const Armors & armors, io::USBCamera * cam)
{
    DetectionResult result;
    result.armors = armors;

    cv::Point2f norm_center(
        armors[0].center.x / static_cast<float>(cam->new_image_width_),
        armors[0].center.y / static_cast<float>(cam->new_image_height_));

    if (cam->device_name == "left") {
        result.delta_yaw = 90;
    } else if (cam->device_name == "right") {
        result.delta_yaw = -90.0;
    } else  result.delta_yaw = 160;

    result.delta_pitch = -10;
;
    detection_queue_.push(result);
}

void Perceptron::DrawAndVisualize(
    const cv::Mat & img,
    const Armors & armors,
    const std::string & window_name)
{
    if (img.empty()) return;  // 检查图像是否有效
    
    cv::Mat vis_img = img.clone();
    for (const auto & armor : armors) {
        for (const auto & point : armor.points) {
            // 检查坐标是否在图像范围内
            if (point.x >= 0 && point.x < vis_img.cols && point.y >= 0 && point.y < vis_img.rows) {
                cv::circle(vis_img, point, 3, {0, 255, 0}, -1);
            }
        }
        // 检查中心坐标是否在范围内
        if (armor.center.x >= 0 && armor.center.x < vis_img.cols && armor.center.y >= 0 && armor.center.y < vis_img.rows) {
            auto info = fmt::format("{} {:.2f}", armor.number, armor.confidence);
            cv::putText(vis_img, info, armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
        }
    }

    cv::Mat resized;
    cv::resize(vis_img, resized, {}, 0.5, 0.5);
    cv::imshow(window_name, resized);
}

std::vector<DetectionResult> Perceptron::GetDetectionQueue()
{
    std::vector<DetectionResult> result;
    DetectionResult temp;
    while (!detection_queue_.empty()) {
        detection_queue_.pop(temp);
        if (!temp.armors.empty()) result.push_back(std::move(temp));
    }

    auto detectionPriority = [](const DetectionResult & detection_result) {
        int highest_priority = 0;
        for (const auto & armor : detection_result.armors) {
            const int priority = armor.number == "1" ? 4 : armor.number == "3" ? 3 
                : armor.number == "4" ? 2 : armor.number == "groud" ? 1 : 0;
            highest_priority = std::max(highest_priority, priority);
            if (highest_priority == 4) break;
        }
        return highest_priority;
    };

    std::stable_sort(result.begin(), result.end(), [&detectionPriority](const DetectionResult & lhs, const DetectionResult & rhs) {
        return detectionPriority(lhs) > detectionPriority(rhs);
    });

    return result;
}

}  // namespace Omniperception

