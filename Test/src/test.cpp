#include <opencv2/opencv.hpp>

#include "AutoArmor.h"
#include "AutoBuff.h"
#include "exiter.hpp"
#include "logger.hpp"
#include "perceptron.hpp"
#include "player.hpp"
#include "State.h"
#include "usbcamera.hpp"


void drawAngleInfo(cv::Mat & img, const Data_Save & data, const Data_Get & dg)
{
    cv::putText(
        img,
        cv::format(
            "Y:%.1f P:%.1f R:%.1f",
            tools::R2D(data.camera_yaw_angle),
            tools::R2D(data.camera_pitch_angle),
            tools::R2D(data.camera_roll_angle)),
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
    cv::putText(
        img, cv::format("Mode:%s", AimStateMachine::Instance().ModeName()),
        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

    const uint32_t x_u =
        (static_cast<uint32_t>(dg.raw_data[2]) << 24) |
        (static_cast<uint32_t>(dg.raw_data[3]) << 16) |
        (static_cast<uint32_t>(dg.raw_data[4]) << 8) |
        static_cast<uint32_t>(dg.raw_data[5]);
    const uint32_t y_u =
        (static_cast<uint32_t>(dg.raw_data[6]) << 24) |
        (static_cast<uint32_t>(dg.raw_data[7]) << 16) |
        (static_cast<uint32_t>(dg.raw_data[8]) << 8) |
        static_cast<uint32_t>(dg.raw_data[9]);
    const int32_t x = static_cast<int32_t>(x_u);
    const int32_t y = static_cast<int32_t>(y_u);
    cv::putText(
        img,
        cv::format(
            "dg x:%.4f y:%.4f id:%u f:%u fire:%u",
            x * 1e-4, y * 1e-4,
            static_cast<unsigned>(dg.raw_data[17]),
            static_cast<unsigned>(dg.raw_data[18]),
            static_cast<unsigned>(dg.raw_data[19])),
        cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}


int main(int argc, char * argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " <record_prefix> [show=1] [start_frame=0]"
                  << std::endl;
        std::cerr << "  record_prefix: path without extension,"
                  << " e.g. records/2026-08-01_12-34-56_0000"
                  << std::endl;
        return 1;
    }

    std::string record_path = argv[1];
    int show = 1;
    int start_frame = 0;
    if (argc >= 3) show = std::stoi(argv[2]);
    if (argc >= 4) start_frame = std::stoi(argv[3]);

    tools::Player player(record_path);
    if (!player.is_opened()) {
        std::cerr << "Failed to open: " << record_path << std::endl;
        return 1;
    }
    if (start_frame > 0) player.seek(start_frame);

    tools::Exiter exiter;

    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>();
    std::unique_ptr<AutoBuff> buff_ = std::make_unique<AutoBuff>();
    std::unique_ptr<Omniperception::Perceptron> perceptron;

    // usb相机用于全感知模型，离线模式下可能不可用
    std::unique_ptr<io::USBCamera> usb_camera0;
    std::unique_ptr<io::USBCamera> usb_camera2;
    try {
        usb_camera0 = std::make_unique<io::USBCamera>("video0", YamlPath);
        usb_camera2 = std::make_unique<io::USBCamera>("video2", YamlPath);
    } catch (const std::exception & e) {
        tools::logger()->warn("USB cameras unavailable, perceptron disabled: {}", e.what());
    }

    cv::Mat img;
    Data_Save ds;
    Data_Get dg;
    bool last_mode = false;

    while (!exiter.exit()) {
        if (!player.read(img, ds)) {
            tools::logger()->info("Playback finished");
            break;
        }

        if (!ds.armor_or_buff || ds.armor_or_buff == 3) {
            // 自瞄模式
            if (ds.armor_or_buff != last_mode)
                tools::logger()->info("Switch to armor mode");

            armor_->AutoAim(img, ds, dg);

            if (!perceptron && usb_camera0 && usb_camera2)
                perceptron = std::make_unique<Omniperception::Perceptron>(
                    usb_camera0.get(), usb_camera2.get(), ds.color, YamlPath);

            auto & state = AimStateMachine::Instance();

            if (perceptron && state.NeedPerceptron(dg)) {
                auto list = perceptron->GetDetectionQueue();
                if (!list.empty())
                    dg.Vision2Control(
                        (tools::R2D(
                             _std_radian(ds.camera_yaw_angle + list[0].delta_yaw))) *
                            1e4,
                        tools::R2D(list[0].delta_pitch) * 1e4, 0, 2, 0);
            }

            state.Step(ds.timestamp, dg);
            if (perceptron) perceptron->SetActive(state.UsbActive());

        } else {
            // 大符模式
            if (ds.armor_or_buff != last_mode)
                tools::logger()->info("Switch to buff mode");

            buff_->AutoAim(img, ds, dg);
        }

        last_mode = ds.armor_or_buff;
        tools::logger()->info("ds.go_back {} ,mode {}", ds.go_back, ds.armor_or_buff);

        if (show) {
            drawAngleInfo(img, ds, dg);
            cv::imshow("playback", img);
            int key = cv::waitKey(30);
            if (key == 'q' || key == 27) break;
        }
    }

    return 0;
}
