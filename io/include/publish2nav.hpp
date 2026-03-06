#ifndef IO__PBLISH2NAV_HPP
#define IO__PBLISH2NAV_HPP

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp" 

namespace io
{
class Publish2Nav : public rclcpp::Node
{
public:
  Publish2Nav();

  ~Publish2Nav();

  void start();

  void send_data(const int16_t & data);

private:
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
};

}  

#endif 