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
// int main()
// {
//     tools::Exiter exiter;
//     std::unique_ptr<io::HikRobot> camera_ = std::make_unique<io::HikRobot>(1000, 0, "2bdf:0001");
//     std::unique_ptr<io::SerialPort> serial_ = std::make_unique<io::SerialPort>("/dev/ttyACM0");
//     std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
//     Mat img;time_point t;Data_Save ds;Data_Get dg;
//     while(!exiter.exit()){
//         camera_->Query(img,t);
//         serial_->Query(ds,t);
//         armor_->AutoAim(img,ds,dg);
//         serial_->SendData(dg);
//         // imshow("image",img);
//         // waitKey(1);
//     }
 
//     return 0;
// }

int main(){
    Mat img;time_point t;Data_Save ds;Data_Get dg;
    std::string video_source = "/home/nav4/leaf/4.avi"; 
    cv::VideoCapture cap(video_source);
    std::unique_ptr<AutoBuff>buffer_ = std::make_unique<AutoBuff>();
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频源: " << video_source << std::endl;
        return -1;
    }
    while (cap.read(img)) {
        if (img.empty()) {
            std::cerr << "读取到空帧，可能视频已结束。" << std::endl;
            break;
        }
        buffer_->AutoAim(img,ds,dg);
        imshow("image",img);
        waitKey(100);
    }
    // TestBuff();

}
