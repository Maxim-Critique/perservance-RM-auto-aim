#include <unistd.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <systemd/sd-daemon.h>
#include "test.h"
#include "AutoArmor.h"
#include "hikrobot.hpp"
#include "usbcamera.hpp"
#include "exiter.hpp"
#include "SerialPort.h"
#include "ros2.hpp"
#include "perceptron.hpp"

void drawAngleInfo(cv::Mat& img, const Data_Save& data) {
    cv::putText(img, cv::format("Y:%.1f P:%.1f R:%.1f", 
                tools::R2D(data.camera_yaw_angle), 
                tools::R2D(data.camera_pitch_angle), 
                tools::R2D(data.camera_roll_angle)),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
}

int main()
{
    auto yaml = YAML::LoadFile(YamlPath);
    auto exposure_time = yaml["Camera"]["exposure_time"].as<double>();
    auto show = yaml["Camera"]["show"].as<int>();
    tools::Exiter exiter;
    std::unique_ptr<io::HikRobot> camera_ = std::make_unique<io::HikRobot>(exposure_time, 0, "2bdf:0001");
    std::unique_ptr<io::SerialPort> serial_ = std::make_unique<io::SerialPort>("/dev/ttyACM0");
    std::unique_ptr<io::ROS2>ros2_ = std::make_unique<io::ROS2>();
    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
    std::unique_ptr<io::USBCamera> usb_camera0 = std::make_unique<io::USBCamera>("video0", YamlPath);
    std::unique_ptr<io::USBCamera> usb_camera2 = std::make_unique<io::USBCamera>("video2", YamlPath);
    std::unique_ptr<Omniperception::Perceptron> perceptron; 
    Mat img;time_point t;Data_Save ds;Data_Get dg;
    
    
    while(!exiter.exit()){
        camera_->Query(img,t);
        serial_->Query(ds,t);
        armor_->AutoAim(img,ds,dg);
        if(!perceptron) perceptron = std::make_unique<Omniperception::Perceptron>(usb_camera0.get(), usb_camera2.get(), ds.color, YamlPath);
        std::vector<int16_t> received_data = ros2_->subscribe_autoaim_target();
        if(!received_data.empty()) dg.Nav2Control(received_data[0],received_data[1]);
        
        uint8_t found = dg.raw_data[18];
        if(found) ros2_->publish(0);
        else{
            if(ds.go_back)ros2_->publish(2);
            else ros2_->publish(1);
            auto list = perceptron->GetDetectionQueue();
            if(!list.empty())dg.Vision2Control((tools::R2D(ds.camera_yaw_angle)+ list[0].delta_yaw)*1e4,list[0].delta_pitch*1e4,0,1,0);
        }
        serial_->SendData(dg);
        if(show)
        {            
            drawAngleInfo(img, ds);
            imshow("image", img);
            cv::waitKey(1);
        }
    }
    return 0;
}


