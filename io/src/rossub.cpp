// Nav2Sub.cpp
#include "Nav2Sub.hpp"
#include <chrono>

Nav2Sub::Nav2Sub(
    const std::string& node_name,
    const std::string& topic_name,
    size_t queue_size
) : Node(node_name),
    data_queue_(queue_size, [this]() { this->onQueueFull(); }),
    spin_thread_running_(false),
    thread_should_stop_(false)
{
    // 创建订阅器
    subscription_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        topic_name,
        rclcpp::QoS(10),  // QoS深度
        [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
            this->dataCallback(msg);
        }
    );

    RCLCPP_INFO(get_logger(), "Nav2Sub initialized, subscribing to topic: %s", 
                topic_name.c_str());
}

Nav2Sub::~Nav2Sub()
{
    stopSpinThread();
    
    RCLCPP_INFO(get_logger(), "Nav2Sub shutting down");
}

void Nav2Sub::startSpinThread()
{
    if (!spin_thread_running_) {
        thread_should_stop_ = false;
        spin_thread_ = std::thread(&Nav2Sub::spinThreadFunction, this);
        spin_thread_running_ = true;
        RCLCPP_INFO(get_logger(), "Spin thread started");
    }
}

void Nav2Sub::stopSpinThread()
{
    if (spin_thread_running_) {
        thread_should_stop_ = true;
        
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        
        spin_thread_running_ = false;
        RCLCPP_INFO(get_logger(), "Spin thread stopped");
    }
}

void Nav2Sub::spinThreadFunction()
{
    RCLCPP_INFO(get_logger(), "Spin thread function started");
    
    // 创建单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_from_this());
    
    // 在单独的线程中运行spin
    while (rclcpp::ok() && !thread_should_stop_) {
        executor.spin_some(std::chrono::milliseconds(10));
    }
    
    RCLCPP_INFO(get_logger(), "Spin thread function exiting");
}

void Nav2Sub::dataCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 2) {
        Int16Pair pair;
        pair.first = msg->data[0];
        pair.second = msg->data[1];
        
        data_queue_.push(pair);
        
        RCLCPP_DEBUG(get_logger(), "Received data: [%d, %d]", 
                     pair.first, pair.second);
    } else {
        RCLCPP_WARN(get_logger(), "Received invalid message with size: %zu", 
                   msg->data.size());
    }
}

bool Nav2Sub::getLatestData(int16_t& value1, int16_t& value2)
{    
    // 尝试获取最新数据
    if (!data_queue_.empty()) {
        Int16Pair latest_data;
        data_queue_.back(latest_data);
        value1 = latest_data.first;
        value2 = latest_data.second;
        data_queue_.clear();
        return true;
    }
    
    return false;
}
