#include "subscribe2nav.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  autoaim_target_queue_(1),
  autoaim_target_counter_(0)
{

  autoaim_target_subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    "auto_aim_target_pos", // 注意话题名可能需要与发送方保持一致
    10,
    std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::autoaim_target_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{

  if (msg->data.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Received Int16MultiArray with insufficient data size: %zu", msg->data.size());
    return;
  }

  autoaim_target_queue_.clear();
  autoaim_target_queue_.push(*msg);

  autoaim_target_counter_++;

  if (autoaim_target_counter_ >= 2) {
    if (autoaim_target_timer_) {
      autoaim_target_timer_->cancel();
    }
    autoaim_target_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      autoaim_target_queue_.clear();
      autoaim_target_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Autoaim target queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::vector<int16_t> Subscribe2Nav::subscribe_autoaim_target()
{

  if (autoaim_target_queue_.empty()) {
    return std::vector<int16_t>();
  }

  std_msgs::msg::Int16MultiArray msg;
  autoaim_target_queue_.back(msg);
  return msg.data;
}

}  // namespace io