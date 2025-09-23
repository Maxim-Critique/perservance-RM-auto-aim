#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <ThreadControl.h>
#include "Camera.h"
#include "Base.h"
#include <systemd/sd-daemon.h>
#include "test.h"
using namespace std;
using namespace cv;

int main()
{

    YAML::Node config = YAML::LoadFile(Yaml);
    int is_test = config["test"].as<int>();
    if(is_test){
        Test();
        // MPCTest();
    }else{
        // 开启相关线程
        ThreadControl ImageControl;
        // 图像生成线程
        std::thread produce_task(&ThreadControl::ImageProduce, std::ref(ImageControl));
        // 图像处理线程
        std::thread process_task(&ThreadControl::ImageProcess, std::ref(ImageControl));
        
        std::thread stm32_task(&ThreadControl::GetSTM32,std::ref(ImageControl));

        stm32_task.detach();
        produce_task.join();
        process_task.join();
    }
    
    return 0;
}
