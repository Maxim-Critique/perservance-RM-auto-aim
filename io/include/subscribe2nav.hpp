#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#include "std_msgs/msg/int16_multi_array.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();

  ~Subscribe2Nav();

  void start();

  std::vector<int16_t> subscribe_autoaim_target();

private:

  void autoaim_target_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

  int autoaim_target_counter_;

  rclcpp::TimerBase::SharedPtr autoaim_target_timer_;

  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr autoaim_target_subscription_;

  tools::ThreadSafeQueue<std_msgs::msg::Int16MultiArray> autoaim_target_queue_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP