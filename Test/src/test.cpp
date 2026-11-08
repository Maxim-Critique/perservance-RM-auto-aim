#include <opencv2/opencv.hpp>

#include "AutoArmor.h"
#include "AutoBuff.h"
#include "exiter.hpp"
#include "logger.hpp"
#include "player.hpp"
#include "State.h"


// 进度条状态
struct PlaybackSlider
{
    int pos = 0;
    bool seek_requested = false;
    tools::Player * player = nullptr;
};

static void onFrameSlider(int pos, void * userdata)
{
    auto * slider = static_cast<PlaybackSlider *>(userdata);
    slider->pos = pos;
    slider->seek_requested = true;
}


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

    const int total_frames = player.total_frames();
    if (total_frames <= 0) {
        std::cerr << "Error: recording has 0 frames" << std::endl;
        return 1;
    }

    // 创建进度条
    PlaybackSlider slider;
    slider.pos = start_frame;
    slider.player = &player;
    if (show) {
        cv::namedWindow("playback", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar(
            "Frame", "playback", &slider.pos, total_frames - 1, onFrameSlider, &slider);
    }

    int current_frame = start_frame;
    bool paused = false;

    tools::Exiter exiter;

    std::unique_ptr<AutoArmor> armor_ = std::make_unique<AutoArmor>(YamlPath, false);
    std::unique_ptr<AutoBuff> buff_ = std::make_unique<AutoBuff>();

    cv::Mat img;
    Data_Save ds;
    Data_Get dg;
    int last_mode = -1;
    int last_color = -1;

    while (!exiter.exit()) {
        // 用户拖动了进度条
        if (slider.seek_requested) {
            slider.seek_requested = false;
            player.seek(slider.pos);
            armor_->Reset();
            AimStateMachine::Instance().Reset();
            dg.Vision2Control();
            last_mode = last_color = -1;
            current_frame = slider.pos;
            paused = false;  // 拖动进度条时自动恢复播放
        }

        if (!paused) {
            if (!player.read(img, ds)) {
                tools::logger()->info("Playback finished");
                break;
            }

            if (ds.armor_or_buff != last_mode || ds.color != last_color) {
                AimStateMachine::Instance().Reset();
                armor_->Reset();
            }

            if (!ds.armor_or_buff || ds.armor_or_buff == 3) {
                // 自瞄模式
                if (ds.armor_or_buff != last_mode)
                    tools::logger()->info("Switch to armor mode");

                armor_->AutoAim(img, ds, dg);

                auto & state = AimStateMachine::Instance();

                if (ds.armor_or_buff == 0) state.Step(ds.timestamp, dg);

            } else {
                // 大符模式
                if (ds.armor_or_buff != last_mode)
                    tools::logger()->info("Switch to buff mode");

                buff_->AutoAim(img, ds, dg);
            }

            last_mode = ds.armor_or_buff;
            last_color = ds.color;
            tools::logger()->info("ds.go_back {} ,mode {}", ds.go_back, ds.armor_or_buff);
        }

        if (show) {
            if (!paused) {
                drawAngleInfo(img, ds, dg);
                cv::putText(
                    img,
                    cv::format("Frame: %d / %d", current_frame, total_frames),
                    cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 255), 2);
            }

            if (paused) {
                cv::putText(
                    img, "PAUSED",
                    cv::Point(10, 170), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, 255), 2);
            }

            cv::imshow("playback", img);

            if (!paused) {
                // 更新进度条（setTrackbarPos 会触发回调，需抑制 seek）
                current_frame++;
                slider.seek_requested = false;
                cv::setTrackbarPos("Frame", "playback", current_frame);
                slider.seek_requested = false;
            }

            // 始终使用 30ms 延时，避免 waitKey(0) 在某些 OpenCV 后端上的兼容问题
            int key = cv::waitKey(30);
            if (key == ' ') {
                paused = !paused;
            } else if (key == 'q' || key == 27) {
                break;
            }
        } else {
            ++current_frame;
        }
    }

    return 0;
}
