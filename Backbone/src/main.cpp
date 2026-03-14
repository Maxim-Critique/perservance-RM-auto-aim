#include <unistd.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <systemd/sd-daemon.h>
#include "test.h"
#include "AutoArmor.h"
#include "hikrobot.hpp" 
#include "exiter.hpp"
#include "SerialPort.h"
#include "ros2.hpp"

void drawAngleInfo(cv::Mat& img, const Data_Save& data) {
    cv::putText(img, cv::format("Y:%.1f P:%.1f R:%.1f", 
                tools::R2D(data.camera_yaw_angle), 
                tools::R2D(data.camera_pitch_angle), 
                tools::R2D(data.camera_roll_angle)),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
}

auto yaml = YAML::LoadFile(YamlPath);
auto exposure_time = yaml["Camera"]["exposure_time"].as<double>();
auto show = yaml["Camera"]["show"].as<int>();
int main()
{
    tools::Exiter exiter;
    std::unique_ptr<io::HikRobot> camera_ = std::make_unique<io::HikRobot>(exposure_time, 0, "2bdf:0001");
    std::unique_ptr<io::SerialPort> serial_ = std::make_unique<io::SerialPort>("/dev/ttyACM0");
    std::unique_ptr<io::ROS2>ros2_ = std::make_unique<io::ROS2>();
    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
    
    
    Mat img;time_point t;Data_Save ds;Data_Get dg;
    while(!exiter.exit()){
        camera_->Query(img,t);
        serial_->Query(ds,t);
        armor_->AutoAim(img,ds,dg);
        
        std::vector<int16_t> received_data = ros2_->subscribe_autoaim_target();
        if(!received_data.empty()) dg.Nav2Control(received_data[0],received_data[1]);
        serial_->SendData(dg);

        uint8_t found = dg.raw_data[18];
        if(found) ros2_->publish(0);
        else{
            if(ds.go_back)ros2_->publish(2);
            else ros2_->publish(1);
        }

        
        if(show)
        {
            // 定义中心 ROI 区域 (640x480)
            int roi_width = 640;
            int roi_height = 480;
            int start_x = (img.cols - roi_width) / 2;  // (1440 - 640) / 2 = 400
            int start_y = (img.rows - roi_height) / 2; // (1080 - 480) / 2 = 300

            // 确保起始坐标和 ROI 大小在图像范围内
            if (start_x >= 0 && start_y >= 0 && 
                start_x + roi_width <= img.cols && 
                start_y + roi_height <= img.rows) 
            {
                // 提取中心 ROI
                cv::Mat roi = img(cv::Rect(start_x, start_y, roi_width, roi_height));
                imshow("image", roi);
                cv::waitKey(1);
            }
        }
    }
    return 0;
}

// int main(){
//     Test();
//     // MPCTest();
//     return 0;
// }