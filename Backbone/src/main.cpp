#include <unistd.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <systemd/sd-daemon.h>
#include "test.h"
#include "AutoArmor.h"
#include "hikrobot.hpp" 
#include "exiter.hpp"
#include "SerialPort.h"

auto yaml = YAML::LoadFile(YamlPath);
auto exposure_time = yaml["Camera"]["exposure_time"].as<double>();
int main()
{
    tools::Exiter exiter;
    std::unique_ptr<io::HikRobot> camera_ = std::make_unique<io::HikRobot>(exposure_time, 0, "2bdf:0001");
    std::unique_ptr<io::SerialPort> serial_ = std::make_unique<io::SerialPort>("/dev/ttyACM0");
    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
    Mat img;time_point t;Data_Save ds;Data_Get dg;
    while(!exiter.exit()){
        camera_->Query(img,t);
        serial_->Query(ds,t);
        armor_->AutoAim(img,ds,dg);
        serial_->SendData(dg);
        imshow("image",img);
        waitKey(1);
    }
 
    return 0;
}
