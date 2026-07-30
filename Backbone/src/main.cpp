#include <unistd.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <systemd/sd-daemon.h>
#include "test.h"
#include "AutoArmor.h"
#include "hikrobot.hpp" 
#include "exiter.hpp"
#include "SerialPort.h"
#include "AutoBuff.h"


void drawAngleInfo(cv::Mat& img, const Data_Save& data) {
    cv::putText(img, cv::format("Y:%.1f P:%.1f R:%.1f", 
                tools::R2D(data.camera_yaw_angle), 
                tools::R2D(data.camera_pitch_angle), 
                tools::R2D(data.camera_roll_angle)),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
}

static std::string build_video_path()
{
    std::time_t now = std::time(nullptr);
    std::tm local_tm{};
    localtime_r(&now, &local_tm);
    char buf[64] = {0};
    std::strftime(buf, sizeof(buf), "output_%Y%m%d_%H%M%S.avi", &local_tm);
    return std::string(buf);
}

auto yaml = YAML::LoadFile(YamlPath);
auto exposure_armor = yaml["Camera"]["exposure_armor"].as<double>();
auto exposure_buff = yaml["Camera"]["exposure_buff"].as<double>();
auto show =  yaml["Camera"]["show"].as<int>();
int main()
{
    tools::Exiter exiter;
    std::unique_ptr<io::HikRobot> camera_ = std::make_unique<io::HikRobot>(exposure_armor, 0, "2bdf:0001");
    std::unique_ptr<io::SerialPort> serial_ = std::make_unique<io::SerialPort>("/dev/ttyACM0");
    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
    std::unique_ptr<AutoBuff> buffer_ = std::make_unique<AutoBuff>();
    Mat img;time_point t;Data_Save ds;Data_Get dg;
    bool last_mode = false;

    
    while(!exiter.exit()){
        camera_->Query(img,t);
        serial_->Query(ds,t);

        // ds.armor_or_buff = 1;
        if(!ds.armor_or_buff){
            armor_->AutoAim(img,ds,dg);
            if(last_mode != ds.armor_or_buff) camera_->SetExposureTime(exposure_armor);
        }
        else {
            buffer_->AutoAim(img,ds,dg);
            if(last_mode != ds.armor_or_buff) camera_->SetExposureTime(exposure_buff);
        }
        last_mode = ds.armor_or_buff;
        tools::logger()->info(" ds.armor_or_buff : {}", ds.armor_or_buff);
        serial_->SendData(dg);
        if(show){
            drawAngleInfo(img,ds);
            imshow("image",img);
            waitKey(1);
        }
    }
    return 0;
}
