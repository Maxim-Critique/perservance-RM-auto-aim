#include "publish2nav.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "logger.hpp"

namespace io
{

Publish2Nav::Publish2Nav() : Node("auto_aim_target_pos_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::Int16>("auto_aim_target_pos", 10);

  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node initialized.");
}

Publish2Nav::~Publish2Nav()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node shutting down.");
}

void Publish2Nav::send_data(const int16_t & target)
{
  auto message = std::make_shared<std_msgs::msg::Int16>();
  message->data = target;
  publisher_->publish(*message);
}

void Publish2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "auto_aim_target_pos_publisher node starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

}  // namespace io