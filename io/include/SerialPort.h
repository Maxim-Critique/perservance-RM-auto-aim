#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H
#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include "Base.h"
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include "logger.hpp"
#include "thread_safe_queue.hpp"
namespace io{
class SerialPort
{
public:
    SerialPort(const char *filename, bool exit_on_timeout = true);
    ~SerialPort();
    bool configure_serial(int fd);
    void restart_serial(void); // 尝试重连的函数
    void read_data();
    void SendData(const struct Data_Get &data);
    void Query(Data_Save &ds,const time_point &t);
    // Latest received sample, without waiting for a future timestamp.
    bool TryLatest(Data_Save &ds);
    int fd;
    int last_fd;
private:
    tools::ThreadSafeQueue<Data_Save,true> queue_;         // 线程通讯队列
    std::thread serial_thread_;                       // 串口采集线程
    const char *file_name_;
    int buadrate_;
    bool DEBUG_SERIAL;
    bool exit_on_timeout_;
    Data_Save data_ahead_, data_behind_;
    std::chrono::steady_clock::time_point last_receive_time_;  // 看门狗：上次收到数据的时间
    std::atomic<bool> serial_quit;                        // 串口退出标准采集
};
}
/*--------------- serial data transmission ----------------*/
// static SerialPort serial_("/dev/ttyACM0");
/*--------------- serial data transmission ----------------*/
#endif
