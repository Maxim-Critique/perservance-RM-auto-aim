#include "hikrobot.hpp"
#include "AutoArmor.h"
#include "Base.h"
#include "exiter.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstdint>

static int32_t read_be(const unsigned char* p)
{
    uint32_t value =
        (uint32_t(p[0]) << 24) |
        (uint32_t(p[1]) << 16) |
        (uint32_t(p[2]) << 8)  |
        uint32_t(p[3]);

    return static_cast<int32_t>(value);
}

int main(int argc, char** argv)
{
    int enemy_color = argc > 1 ? std::stoi(argv[1]) : 0;

    tools::Exiter exiter;

    std::cout << "Starting Hikrobot camera..." << std::endl;

    const auto config = YAML::LoadFile(YamlPath);
    io::HikRobot camera(config["Camera"]["exposure_armor"].as<double>(), 0.0, "2bdf:0001");

    std::cout << "Initializing AutoArmor..." << std::endl;

    AutoArmor armor;

    Data_Save ds{};
    Data_Get dg;

    ds.color = enemy_color;
    ds.armor_or_buff = 0;
    ds.go_back = 0;
    ds.camera_yaw_angle = 0;
    ds.camera_pitch_angle = 0;
    ds.camera_roll_angle = 0;

    cv::Mat img;
    time_point timestamp;

    const char* display_env = std::getenv("DISPLAY");
    bool show_window = display_env && *display_env;

    if (show_window)
        cv::namedWindow("Sentry AutoAim Preview", cv::WINDOW_NORMAL);

    int frames = 0;

    std::cout << "AutoAim started. Press Q or ESC to exit." << std::endl;

    while (!exiter.exit())
    {
        camera.Query(img, timestamp);

        if (img.empty())
            continue;

        ds.timestamp = timestamp;

        // 执行原工程完整主自瞄算法，但不发送串口数据
        bool selected = armor.AutoAim(img, ds, dg);

        int32_t yaw = read_be(&dg.raw_data[2]);
        int32_t pitch = read_be(&dg.raw_data[6]);

        std::string info = cv::format(
            "Target: %s  Yaw: %.2f  Pitch: %.2f",
            selected ? "FOUND" : "NONE",
            yaw * 1e-4,
            pitch * 1e-4
        );

        cv::putText(img, info, cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 0), 2);

        frames++;

        if (frames == 1 || frames % 30 == 0)
        {
            cv::imwrite("../logs/aim_preview.jpg", img);
        }

        if (frames % 100 == 0)
        {
            std::cout
                << "Frames: " << frames
                << " | Target: " << selected
                << " | Yaw: " << yaw * 1e-4
                << " | Pitch: " << pitch * 1e-4
                << " | Fire flag (not transmitted): "
                << static_cast<int>(dg.raw_data[19])
                << std::endl;
        }

        if (show_window)
        {
            cv::imshow("Sentry AutoAim Preview", img);

            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
                break;
        }
    }

    cv::destroyAllWindows();

    std::cout << "AutoAim preview stopped." << std::endl;

    return 0;
}
