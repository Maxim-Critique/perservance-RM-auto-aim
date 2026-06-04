#include <unistd.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <systemd/sd-daemon.h>
#include "test.h"
#include "AutoArmor.h"
#include "AutoBuff.h"
#include "hikrobot.hpp"
#include "usbcamera.hpp"
#include "exiter.hpp"
#include "SerialPort.h"
#include "ros2.hpp"
#include "perceptron.hpp"
#include "State.h"


void drawAngleInfo(cv::Mat& img, const Data_Save& data, const Data_Get& dg) {
    cv::putText(img, cv::format("Y:%.1f P:%.1f R:%.1f", 
                tools::R2D(data.camera_yaw_angle), 
                tools::R2D(data.camera_pitch_angle), 
                tools::R2D(data.camera_roll_angle)),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(img, cv::format("Mode:%s", AimStateMachine::Instance().ModeName()),
                cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

    const uint32_t x_u = (static_cast<uint32_t>(dg.raw_data[2]) << 24) |
                         (static_cast<uint32_t>(dg.raw_data[3]) << 16) |
                         (static_cast<uint32_t>(dg.raw_data[4]) << 8) |
                         static_cast<uint32_t>(dg.raw_data[5]);
    const uint32_t y_u = (static_cast<uint32_t>(dg.raw_data[6]) << 24) |
                         (static_cast<uint32_t>(dg.raw_data[7]) << 16) |
                         (static_cast<uint32_t>(dg.raw_data[8]) << 8) |
                         static_cast<uint32_t>(dg.raw_data[9]);
    const int32_t x = static_cast<int32_t>(x_u);
    const int32_t y = static_cast<int32_t>(y_u);
    cv::putText(img, cv::format("dg x:%.4f y:%.4f id:%u f:%u fire:%u",
                x * 1e-4, y * 1e-4,
                static_cast<unsigned>(dg.raw_data[17]),
                static_cast<unsigned>(dg.raw_data[18]),
                static_cast<unsigned>(dg.raw_data[19])),
                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}


int main()
{
    auto yaml = YAML::LoadFile(YamlPath);
    auto exposure_armor = yaml["Camera"]["exposure_armor"].as<double>();
    auto exposure_buff = yaml["Camera"]["exposure_buff"].as<double>();
    auto show = yaml["Camera"]["show"].as<int>();
    tools::Exiter exiter;
    std::unique_ptr<io::HikRobot> camera_ = std::make_unique<io::HikRobot>(exposure_buff, 0, "2bdf:0001");
    std::unique_ptr<io::SerialPort> serial_ = std::make_unique<io::SerialPort>("/dev/ttyACM0");
    std::unique_ptr<io::ROS2>ros2_ = std::make_unique<io::ROS2>();
    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
    std::unique_ptr<AutoBuff> buff_ = std::make_unique<AutoBuff>();
    std::unique_ptr<io::USBCamera> usb_camera0 = std::make_unique<io::USBCamera>("video0", YamlPath);
    std::unique_ptr<io::USBCamera> usb_camera2 = std::make_unique<io::USBCamera>("video2", YamlPath);
    std::unique_ptr<io::USBCamera> usb_camera4 = std::make_unique<io::USBCamera>("video4", YamlPath);
    std::unique_ptr<Omniperception::Perceptron> perceptron; 
    Mat img;time_point t;Data_Save ds;Data_Get dg;
    bool last_mode = false;
    
    while(!exiter.exit()){
        camera_->Query(img,t);
        serial_->Query(ds,t);
        
        if(!ds.armor_or_buff){
            if(ds.armor_or_buff != last_mode) camera_->SetExposure(exposure_armor);
            armor_->AutoAim(img,ds,dg);
            if(!perceptron) perceptron = std::make_unique<Omniperception::Perceptron>(usb_camera0.get(), usb_camera2.get(), usb_camera4.get(),ds.color, YamlPath);
            uint8_t found = dg.raw_data[18];
            bool usb_active = AimStateMachine::Instance().UsbActive();
            if(!found && usb_active){
                auto list = perceptron->GetDetectionQueue();
                if(!list.empty()) dg.Vision2Control((tools::R2D(_std_radian(ds.camera_yaw_angle+ list[0].delta_yaw)))*1e4,tools::R2D(list[0].delta_pitch)*1e4,0,2,0);
                ros2_->publish(ds.go_back);
            }else ros2_->publish(0);
            AimStateMachine::Instance().Step(t, dg);
            perceptron->SetActive(usb_active);
        }else{
            if(ds.armor_or_buff != last_mode) camera_->SetExposure(exposure_buff);
            ros2_->publish(ds.go_back);
            buff_->AutoAim(img,ds,dg);
        }

        last_mode = ds.armor_or_buff;
        tools::logger()->info("ds.go_back {} ,mode {}",ds.go_back,ds.armor_or_buff);

        std::vector<int16_t> received_data = ros2_->subscribe_autoaim_target();
        if(!received_data.empty()) {
            dg.Nav2Control(received_data[0],received_data[1]);
            tools::logger()->info("received_data {} {}",received_data[0],received_data[1]);
        }

        serial_->SendData(dg);

        if(show)
        {            
            drawAngleInfo(img, ds, dg);
            imshow("image", img);
            cv::waitKey(1);
        }
    }
    return 0;
}


