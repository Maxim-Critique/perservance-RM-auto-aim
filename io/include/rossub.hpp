// Nav2Sub.hpp
#ifndef INCLUDE__NAV2_SUB_HPP
#define INCLUDE__NAV2_SUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>

#include "tools/thread_safe_queue.hpp"

class Nav2Sub : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数（不包含ROS初始化）
     * @param node_name ROS2节点名称
     * @param topic_name 订阅的主题名称
     * @param queue_size 内部队列的最大容量
     */
    Nav2Sub(
        const std::string& node_name = "nav2_subscriber",
        const std::string& topic_name = "/nav2_data",
        size_t queue_size = 10
    );

    /**
     * @brief 析构函数，确保线程安全退出
     */
    ~Nav2Sub();

    /**
     * @brief 启动内部spin线程
     */
    void startSpinThread();

    /**
     * @brief 停止内部spin线程
     */
    void stopSpinThread();

    /**
     * @brief 获取最新的数据
     * @param value1 返回第一个int16值
     * @param value2 返回第二个int16值
     * @return 成功获取返回true，队列为空返回false
     */
    bool getLatestData(int16_t& value1, int16_t& value2);

    /**
     * @brief 检查spin线程是否在运行
     * @return 线程运行中返回true
     */
    bool isSpinThreadRunning() const { return spin_thread_running_; }

private:
    void dataCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void spinThreadFunction();

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr subscription_;

    // 存储int16对的数据结构
    struct Int16Pair {
        int16_t first;
        int16_t second;
    };
    
    tools::ThreadSafeQueue<Int16Pair, true> data_queue_;

    // Spin线程
    std::thread spin_thread_;
    
    // 线程控制标志
    std::atomic<bool> spin_thread_running_;
    std::atomic<bool> thread_should_stop_;

    void onQueueFull() {
        RCLCPP_WARN(get_logger(), "Data queue is full, dropping oldest data.");
    }
};

#endif // INCLUDE__NAV2_SUB_HPP