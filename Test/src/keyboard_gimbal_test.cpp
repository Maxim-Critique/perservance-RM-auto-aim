// Manual camera/gimbal preview. Target: keyboard_gimbal_test. Space holds a manual fire request.
#include "ArmorDetector.h"
#include "AutoArmor.h"
#include "SerialPort.h"
#include "hikrobot.hpp"
#include "exiter.hpp"
#include "logger.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <QApplication>
#include <QCloseEvent>
#include <QFocusEvent>
#include <QImage>
#include <QScreen>
#include <QSocketNotifier>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QStringList>
#include <QTimer>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

using Clock = std::chrono::steady_clock;

// A fixed terminal panel; lifecycle messages remain in the file log.
class TerminalPanel {
    bool tty_ = isatty(STDOUT_FILENO);
public:
    TerminalPanel() {
        if (tty_) std::cout << "\033[?1049h\033[?25h\033[?7l\033[2J";
        std::cout << "R：切换追踪自瞄；方向键：切回手动并转动\n"
                     "窗口按键需焦点；终端输入 yaw pitch 整数并回车，空行取消目标\n"
                     "空格：按住请求发射，松开停止；B / F：蓝 / 红目标；诊断始终显示；Q / Esc：退出\n"
                     "角度单位：度；测试程序不限制俯仰角；终端转向不发射\n";
        if (tty_) std::cout << "\033[9;1H输入 yaw pitch > ";
        std::cout << std::flush;
    }
    void status(double yaw, double pitch, bool fresh, bool focused, bool moving, bool fire, bool tracking, bool target, bool positioning) {
        if (!tty_) return;
        std::cout << "\033[s\033[6;1H\033[2K" << "反馈 yaw=" << yaw << "° pitch=" << pitch
                  << "° | " << (fresh ? "数据正常" : "等待数据/已过期")
                  << " | " << (focused ? "窗口有焦点" : "窗口失焦")
                  << " | " << (moving ? "移动" : "停止")
                  << " | " << (tracking ? (target ? "追踪：有瞄准输出" : "追踪：搜索目标") : (positioning ? "终端转向" : "手动"))
                  << " | 发射请求=" << (fire ? 1 : 0) << "\033[u" << std::flush;
    }
    void message(const std::string& text) {
        if (tty_) {
            std::cout << "\033[s\033[7;1H\033[2K" << text << "\033[u" << std::flush;
        } else std::cout << text << '\n';
    }
    void prompt() {
        if (tty_) std::cout << "\033[9;1H\033[2K\033[10;1H\033[2K\033[9;1H输入 yaw pitch > " << std::flush;
    }
    ~TerminalPanel() {
        if (tty_) std::cout << "\033[?7h\033[?25h\033[?1049l" << std::flush;
    }
};

class Preview : public QWidget {
    io::HikRobot& camera_;
    io::SerialPort& serial_;
    ArmorDetector& detector_;
    AutoArmor& auto_armor_;
    bool tracking_ = false, aim_ready_ = false;
    Data_Get aim_packet_;
    time_point aim_time_{};
    TerminalPanel& terminal_;
    tools::Exiter& exiter_;
    QTimer timer_;
    std::unique_ptr<QSocketNotifier> stdin_notifier_;
    std::string input_buffer_;
    std::optional<std::pair<int,int>> terminal_target_;
    bool position_mode_ = false;
    std::optional<std::pair<double,double>> predicted_direction_;
    std::array<bool, 4> held_{};
    Data_Save feedback_{};
    time_point image_time_{}, last_tick_ = Clock::now(), panel_time_{};
    bool have_feedback_ = false, initialized_ = false, commanded_ = false;
    bool fire_held_ = false, last_fire_request_ = false;
    double yaw_ = 0, pitch_ = 0, origin_yaw_ = 0;
    double speed_, yaw_sign_, pitch_sign_;
    int color_;
    QImage picture_;
    cv::Mat frame_;
    QString state_ = "Waiting for camera and feedback";
    QString error_;
    bool have_tx_=false;
    Data_Get last_tx_;
    time_point tx_time_{};
    uint64_t tx_attempts_=0, feedback_updates_=0, image_updates_=0;
    int last_tx_fd_=-1;
    double raw_interval_ms_=0, used_dt_ms_=0, processing_ms_=0, detection_ms_=0, send_ms_=0;
    double feedback_gap_ms_=0, yaw_rate_=0, pitch_rate_=0;
    bool have_rate_=false, yaw_origin_clipped_=false, yaw_feedback_clipped_=false;
    double proposed_yaw_=0;
    size_t detection_count_=0;
    std::optional<cv::Point2f> candidate_pixel_;
    std::optional<Eigen::Vector3d> predicted_xyz_;
    QString tx_source_="--", last_event_="启动，等待数据", input_status_="尚未输入";

    static QString number(double value, int precision=1) {
        return std::isfinite(value) ? QString::number(value,'f',precision) : "--";
    }
    static QString age(time_point timestamp) {
        if (timestamp==time_point{}) return "--";
        return number(std::chrono::duration<double,std::milli>(Clock::now()-timestamp).count());
    }
    QString blockedReason() const {
        if (!error_.isEmpty()) return "异常";
        if (!have_feedback_) return "无串口反馈";
        if (image_time_==time_point{}) return "无图像";
        if (Clock::now()-feedback_.timestamp>=std::chrono::milliseconds(250)) return "串口反馈过期";
        if (Clock::now()-image_time_>=std::chrono::milliseconds(250)) return "图像过期/处理超时";
        if (!initialized_) return "未初始化";
        if ((!isActiveWindow() || !hasFocus()) && !position_mode_) return "窗口失焦";
        return "无";
    }
    void inputMessage(const std::string& text) {
        input_status_=QString::fromStdString(text);
        terminal_.message(text);
    }
    QStringList diagnosticLines() const {
        const bool valid_tx=have_tx_;
        const double rx_yaw=feedback_.camera_yaw_angle*180.0/PI;
        const double command_yaw=valid_tx ? decodeAngle(last_tx_.raw_data+2) : 0;
        const double command_pitch=valid_tx ? decodeAngle(last_tx_.raw_data+6) : 0;
        const QString mode=tracking_ ? "追踪" : (position_mode_ ? "终端转向" : "手动/保持");
        QStringList lines;
        lines << QString("模式=%1  阻断=%2  窗口激活/焦点=%3/%4  键[上 下 左 右 空格]=%5 %6 %7 %8 %9")
            .arg(mode,blockedReason()).arg(isActiveWindow()).arg(hasFocus())
            .arg(held_[0]).arg(held_[1]).arg(held_[2]).arg(held_[3]).arg(fire_held_);
        lines << QString("RX角度(度) yaw原解码=%1 展开=%2  pitch=%3 roll=%4 | 速率Y/P=%5/%6 度/s")
            .arg(have_feedback_ ? number(rx_yaw,2) : "--",have_feedback_ ? number(measuredYaw(),2) : "--",
                 have_feedback_ ? number(measuredPitch(),2) : "--",have_feedback_ ? number(feedback_.camera_roll_angle*180.0/PI,2) : "--",
                 have_rate_ ? number(yaw_rate_) : "--",have_rate_ ? number(pitch_rate_) : "--");
        lines << QString("TX尝试#%1 源=%2 fd=%3 ID/found/fire=%4/%5/%6 | 角度Y/P=%7/%8 (无电控ACK)")
            .arg(static_cast<qulonglong>(tx_attempts_)).arg(tx_source_)
            .arg(QString("%1→%2").arg(last_tx_fd_).arg(serial_.fd))
            .arg(valid_tx ? QString::number(last_tx_.raw_data[17]) : "--")
            .arg(valid_tx ? QString::number(last_tx_.raw_data[18]) : "--")
            .arg(valid_tx ? QString::number(last_tx_.raw_data[19]) : "--")
            .arg(valid_tx ? number(command_yaw,2) : "--",valid_tx ? number(command_pitch,2) : "--");
        lines << QString("TX整数Y/P=%1/%2 | 指令-反馈: yaw直接=%3 最短角=%4 pitch=%5 度")
            .arg(valid_tx ? QString::number(static_cast<qlonglong>(std::llround(command_yaw*10000))) : "--")
            .arg(valid_tx ? QString::number(static_cast<qlonglong>(std::llround(command_pitch*10000))) : "--")
            .arg(valid_tx && have_feedback_ ? number(command_yaw-measuredYaw(),2) : "--",
                 valid_tx && have_feedback_ ? number(std::remainder(command_yaw-measuredYaw(),360.0),2) : "--",
                 valid_tx && have_feedback_ ? number(command_pitch-measuredPitch(),2) : "--");
        lines << QString("数据年龄ms 图像/RX/瞄准/TX=%1/%2/%3/%4 | RX时间-图像时间=%5ms | 阈值250ms")
            .arg(age(image_time_),have_feedback_ ? age(feedback_.timestamp) : "--",age(aim_time_),age(tx_time_),
                 have_feedback_ && image_time_!=time_point{} ? number(std::chrono::duration<double,std::milli>(feedback_.timestamp-image_time_).count()) : "--");
        lines << QString("上次耗时ms 周期/控制dt/处理/检测自瞄/TX=%1/%2/%3/%4/%5 | RX间隔=%6 图像/RX更新=%7/%8")
            .arg(number(raw_interval_ms_),number(used_dt_ms_),number(processing_ms_),number(detection_ms_),number(send_ms_),number(feedback_gap_ms_))
            .arg(static_cast<qulonglong>(image_updates_)).arg(static_cast<qulonglong>(feedback_updates_));
        lines << QString("yaw限幅 启动[%1,%2]命中=%3  反馈±1命中=%4 | 本周期候选yaw=%5 | 速度=%6 符号Y/P=%7/%8")
            .arg(number(origin_yaw_-10),number(origin_yaw_+10)).arg(yaw_origin_clipped_).arg(yaw_feedback_clipped_)
            .arg(number(proposed_yaw_,2),number(speed_)).arg(yaw_sign_).arg(pitch_sign_);
        lines << QString("AUTO ready=%1 ID/found/fire=%2/%3/%4  角度Y/P=%5/%6 | 检测数=%7")
            .arg(aim_ready_).arg(aim_packet_.raw_data[17]).arg(aim_packet_.raw_data[18]).arg(aim_packet_.raw_data[19])
            .arg(aim_time_!=time_point{} ? number(decodeAngle(aim_packet_.raw_data+2),2) : "--",
                 aim_time_!=time_point{} ? number(decodeAngle(aim_packet_.raw_data+6),2) : "--")
            .arg(static_cast<qulonglong>(detection_count_));
        lines << QString("预测XYZ(m)=%1 | 最近候选像素=%2（不等于跟踪锁定点）")
            .arg(predicted_xyz_ ? QString("%1,%2,%3").arg(number((*predicted_xyz_)[0],2),number((*predicted_xyz_)[1],2),number((*predicted_xyz_)[2],2)) : "--",
                 candidate_pixel_ ? QString("%1,%2").arg(number(candidate_pixel_->x),number(candidate_pixel_->y)) : "--");
        lines << QString("最近事件：%1 | 终端：%2").arg(last_event_,input_status_);
        return lines;
    }
    static int index(int key) {
        switch (key) {
        case Qt::Key_Up: return 0;
        case Qt::Key_Down: return 1;
        case Qt::Key_Left: return 2;
        case Qt::Key_Right: return 3;
        default: return -1;
        }
    }
    double measuredYaw() const {
        const double wrapped = feedback_.camera_yaw_angle * 180.0 / PI;
        return initialized_ ? origin_yaw_ + std::remainder(wrapped-origin_yaw_, 360.0) : wrapped;
    }
    double measuredPitch() const { return feedback_.camera_pitch_angle * 180.0 / PI; }
    bool fresh() const {
        const auto now = Clock::now();
        return have_feedback_ && image_time_ != time_point{} &&
               now-feedback_.timestamp < std::chrono::milliseconds(250) &&
               now-image_time_ < std::chrono::milliseconds(250);
    }
    static int32_t encodeAngle(double degrees) {
        const double scaled=degrees*10000.0;
        if (!std::isfinite(scaled) || scaled<std::numeric_limits<int32_t>::min() ||
            scaled>std::numeric_limits<int32_t>::max())
            throw std::out_of_range("Angle cannot be encoded as int32 degrees * 10000");
        return static_cast<int32_t>(std::llround(scaled));
    }
    static double decodeAngle(const u_char* bytes) {
        const uint32_t value = (uint32_t(bytes[0])<<24) | (uint32_t(bytes[1])<<16) |
                               (uint32_t(bytes[2])<<8) | uint32_t(bytes[3]);
        const int64_t signed_value = value >= 0x80000000u ? int64_t(value)-0x100000000LL : value;
        return signed_value * 1e-4;
    }
    void resetAim() {
        aim_ready_=false;
        predicted_direction_.reset();
        predicted_xyz_.reset();
        aim_time_=time_point{};
        aim_packet_.Vision2Control();
        auto_armor_.Reset();
    }
    void send(bool valid) {
        valid = valid && initialized_ && fresh() && error_.isEmpty();
        const bool focused = isActiveWindow() && hasFocus();
        Data_Get packet;
        if (tracking_ && valid && focused && aim_ready_ &&
            Clock::now()-aim_time_ < std::chrono::milliseconds(250)) {
            // Preserve existing aim angles, target ID/found and Shooter decisions.
            tx_source_="AUTO";
            packet=aim_packet_;
            packet.raw_data[19] = (fire_held_ && packet.raw_data[19] != 0) ? 1 : 0;
            yaw_=decodeAngle(packet.raw_data+2);
            pitch_=decodeAngle(packet.raw_data+6);
        } else {
            tx_source_=!valid ? "INVALID" : (tracking_ ? "AUTO保持" : (position_mode_ ? "终端" : "手动/保持"));
            const bool fire = !tracking_ && !position_mode_ && valid && focused && fire_held_;
            // No valid auto target: hold feedback angle and never reuse a stale aim packet.
            if (tracking_ && initialized_) { yaw_=measuredYaw(); pitch_=measuredPitch(); }
            packet.Vision2Control(encodeAngle(yaw_), encodeAngle(pitch_), 0,
                                  valid ? 2 : 0, static_cast<uint8_t>(fire));
        }
        packet.Nav2Control(0, 0);
        const auto send_start=Clock::now();
        last_tx_fd_=serial_.fd;
        ++tx_attempts_;
        serial_.SendData(packet);
        send_ms_=std::chrono::duration<double,std::milli>(Clock::now()-send_start).count();
        last_tx_=packet;
        tx_time_=Clock::now();
        have_tx_=true;
        yaw_=decodeAngle(packet.raw_data+2);
        pitch_=decodeAngle(packet.raw_data+6);
        last_fire_request_ = packet.raw_data[19] != 0;
    }
    void stop(const QString& reason="停止/取消") {
        last_event_=reason;
        held_.fill(false);
        fire_held_ = false;
        tracking_ = false;
        position_mode_ = false;
        predicted_direction_.reset();
        resetAim();
        if (initialized_) { yaw_ = measuredYaw(); pitch_ = measuredPitch(); }
        if (commanded_) send(fresh());
    }
    void submitTarget(const std::string& line) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) {
            stop(); terminal_target_.reset(); inputMessage("已取消终端目标"); return;
        }
        std::istringstream parser(line);
        std::string a,b,extra;
        if (!(parser>>a>>b) || (parser>>extra)) {
            inputMessage("格式错误：请输入两个整数，如 32 -21"); return;
        }
        const auto integer=[](const std::string& text) {
            const size_t start=(!text.empty() && (text[0]=='-' || text[0]=='+')) ? 1 : 0;
            return start<text.size() && text.find_first_not_of("0123456789",start)==std::string::npos;
        };
        if (!integer(a) || !integer(b)) {
            inputMessage("仅接受整数角度"); return;
        }
        int y,p;
        try { y=std::stoi(a); p=std::stoi(b); }
        catch (const std::exception&) { inputMessage("整数超出范围"); return; }
        if (y < -180 || y > 180) {
            inputMessage("yaw 范围为 [-180,180]"); return;
        }
        // This checks wire representation only; no mechanical pitch range is imposed.
        try { (void)encodeAngle(p); }
        catch (const std::exception&) {
            inputMessage("pitch 超出串口 int32（度乘 10000）可编码范围"); return;
        }
        if (!initialized_ || !fresh() || !error_.isEmpty()) {
            inputMessage("数据未就绪或过期，未执行；恢复后请重新输入"); return;
        }
        stop();
        last_event_="收到终端角度，转向启用";
        terminal_target_=std::make_pair(y,p);
        position_mode_=true;
        commanded_=true;
        inputMessage("目标已接受，按设定速度转向；空行可停止");
    }
    void readTerminal() {
        char data[1024];
        const auto size=::read(STDIN_FILENO,data,sizeof(data));
        if (size<=0) { stdin_notifier_->setEnabled(false); return; }
        input_buffer_.append(data,static_cast<size_t>(size));
        for (size_t newline; (newline=input_buffer_.find('\n'))!=std::string::npos;) {
            const auto line=input_buffer_.substr(0,newline);
            input_buffer_.erase(0,newline+1);
            if (line.size()>128) inputMessage("输入过长，未执行");
            else submitTarget(line);
            terminal_.prompt();
        }
        if (input_buffer_.size()>4096) { input_buffer_.clear(); inputMessage("输入过长，已清除"); }
    }
    void drawFrame() {
        const auto detection_start=Clock::now();
        auto display = frame_.clone();
        Armors armors;
        if (tracking_ && fresh() && error_.isEmpty() && isActiveWindow() && hasFocus()) {
            Data_Save input=feedback_;
            input.timestamp=image_time_;
            input.color=color_;
            input.armor_or_buff=0;
            // Run once per new camera frame; reuse observations for pixel overlays.
            aim_ready_=auto_armor_.AutoAim(display,input,aim_packet_,&armors);
            aim_ready_=aim_ready_ && aim_packet_.raw_data[18] == 1;
            aim_time_=image_time_;
            predicted_direction_.reset();
            predicted_xyz_.reset();
            const auto& point=auto_armor_.PredictedAimPoint();
            if (point.valid && point.xyza.allFinite()) {
                predicted_xyz_=point.xyza.head<3>();
                const double x=point.xyza[0], y=point.xyza[1], z=point.xyza[2];
                if (std::hypot(x,y)>1e-6)
                    predicted_direction_=std::make_pair(std::atan2(y,x)*180.0/PI,
                                                        std::atan2(z,std::hypot(x,y))*180.0/PI);
            }
        } else {
            predicted_direction_.reset();
            predicted_xyz_.reset();
            armors=detector_.Detect(display,color_);
        }
        detection_count_=armors.size();
        candidate_pixel_.reset();
        const cv::Point2f screen_center(display.cols*0.5f, display.rows*0.5f);
        cv::drawMarker(display, screen_center, cv::Scalar(255,255,255), cv::MARKER_CROSS, 20, 1);
        double best_distance = std::numeric_limits<double>::infinity();
        cv::Point2f candidate;
        bool found = false;
        for (const auto& armor : armors) {
            if (armor.points.size() < 4) continue;
            cv::Point2f center(0,0);
            for (int i=0; i<4; ++i) {
                center += armor.points[i]*0.25f;
                cv::line(display, armor.points[i], armor.points[(i+1)%4], cv::Scalar(0,255,0), 2);
            }
            cv::drawMarker(display, center, cv::Scalar(0,255,255), cv::MARKER_CROSS, 16, 2);
            cv::putText(display, cv::format("%s %.2f (%.1f,%.1f)", armor.number.c_str(),
                        armor.confidence, center.x, center.y), armor.points[0],
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0,255,255), 2);
            const double distance = cv::norm(center-screen_center);
            if (distance < best_distance) { best_distance=distance; candidate=center; found=true; }
        }
        if (found) {
            candidate_pixel_=candidate;
            cv::arrowedLine(display, screen_center, candidate, cv::Scalar(255,0,255), 2);
            cv::putText(display, cv::format("Candidate pixel (%.1f, %.1f)", candidate.x, candidate.y),
                        cv::Point(15,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,0,255), 2);
        } else {
            cv::putText(display, "No candidate", cv::Point(15,30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0,255,255), 2);
        }
        detection_ms_=std::chrono::duration<double,std::milli>(Clock::now()-detection_start).count();
        cv::Mat rgb;
        cv::cvtColor(display, rgb, cv::COLOR_BGR2RGB);
        picture_ = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    void tick() {
        if (exiter_.exit()) { close(); return; }
        const auto now = Clock::now();
        raw_interval_ms_=std::chrono::duration<double,std::milli>(now-last_tick_).count();
        const double dt = std::clamp(std::chrono::duration<double>(now-last_tick_).count(), 0.0, 0.05);
        used_dt_ms_=dt*1000;
        last_tick_ = now;
        yaw_origin_clipped_=yaw_feedback_clipped_=false;
        proposed_yaw_=yaw_;
        Data_Save sample{};
        if (serial_.TryLatest(sample)) {
            if (std::isfinite(sample.camera_yaw_angle) && std::isfinite(sample.camera_pitch_angle)) {
                if (have_feedback_ && sample.timestamp>feedback_.timestamp) {
                    const double seconds=std::chrono::duration<double>(sample.timestamp-feedback_.timestamp).count();
                    feedback_gap_ms_=seconds*1000;
                    yaw_rate_=std::remainder((sample.camera_yaw_angle-feedback_.camera_yaw_angle)*180.0/PI,360.0)/seconds;
                    pitch_rate_=(sample.camera_pitch_angle-feedback_.camera_pitch_angle)*180.0/PI/seconds;
                    have_rate_=true;
                }
                ++feedback_updates_;
                feedback_=sample; have_feedback_=true;
            }
        }
        cv::Mat image;
        time_point timestamp;
        const bool new_frame = camera_.TryQuery(image, timestamp) && !image.empty();
        if (new_frame) { ++image_updates_; frame_=image; image_time_=timestamp; }
        if (!initialized_ && fresh()) {
            origin_yaw_=yaw_=measuredYaw(); pitch_=measuredPitch(); initialized_=true;
        }
        if (new_frame && error_.isEmpty()) drawFrame();
        const bool valid = initialized_ && fresh() && error_.isEmpty();
        if (!valid || ((!isActiveWindow() || !hasFocus()) && !position_mode_)) {
            if (tracking_ || position_mode_ || fire_held_ || std::any_of(held_.begin(),held_.end(),[](bool down){return down;}))
                last_event_="控制取消："+blockedReason();
            if (tracking_) { tracking_=false; resetAim(); }
            held_.fill(false);
            fire_held_=false;
            if (!valid) { position_mode_=false; predicted_direction_.reset(); }
        }
        const int dy = int(held_[2])-int(held_[3]);
        const int dp = int(held_[0])-int(held_[1]);
        const bool moving = !tracking_ && valid && isActiveWindow() && hasFocus() && (dy || dp);
        const bool position_moving = position_mode_ && terminal_target_ && valid &&
            (std::abs(std::remainder(terminal_target_->first-measuredYaw(),360.0)) > 0.2 ||
             std::abs(terminal_target_->second-measuredPitch()) > 0.2);
        if (tracking_) {
            commanded_=true;
            send(valid);
        } else if (position_mode_ && terminal_target_ && valid) {
            const double target_yaw=measuredYaw()+std::remainder(terminal_target_->first-measuredYaw(),360.0);
            const double target_pitch=terminal_target_->second;
            const double step=speed_*dt;
            yaw_ += std::clamp(target_yaw-yaw_,-step,step);
            pitch_ += std::clamp(target_pitch-pitch_,-step,step);
            proposed_yaw_=yaw_;
            yaw_=std::clamp(yaw_,measuredYaw()-1.0,measuredYaw()+1.0);
            yaw_feedback_clipped_=std::abs(yaw_-proposed_yaw_)>1e-7;
            fire_held_=false;
            send(valid);
        } else if (moving) {
            yaw_ += dy*yaw_sign_*speed_*dt;
            pitch_ += dp*pitch_sign_*speed_*dt;
            // Keep the existing manual yaw bounds; pitch has only rate control.
            proposed_yaw_=yaw_;
            yaw_=std::clamp(yaw_, origin_yaw_-10.0, origin_yaw_+10.0);
            yaw_origin_clipped_=std::abs(yaw_-proposed_yaw_)>1e-7;
            const double before_feedback_clamp=yaw_;
            yaw_=std::clamp(yaw_, measuredYaw()-1.0, measuredYaw()+1.0);
            yaw_feedback_clipped_=std::abs(yaw_-before_feedback_clamp)>1e-7;
            commanded_=true;
            send(true);
        } else {
            if (!valid || !isActiveWindow() || !hasFocus()) {
                held_.fill(false);
                fire_held_ = false; // Recovery requires a new physical press.
            }
            if (initialized_) { yaw_=measuredYaw(); pitch_=measuredPitch(); }
            if (commanded_) send(valid); // Keep a feedback-angle hold, not the previous moving setpoint.
        }
        state_=QString("Yaw %1 deg | Pitch %2 deg | Command %3 / %4 | %5 | %6 | FIRE REQUEST %7 | %8")
            .arg(measuredYaw(),0,'f',2).arg(measuredPitch(),0,'f',2)
            .arg(yaw_,0,'f',2).arg(pitch_,0,'f',2)
            .arg(valid ? "FEEDBACK OK" : "STALE / WAITING").arg(moving || position_moving ? "MOVING" : "STOPPED")
            .arg(last_fire_request_ ? 1 : 0)
            .arg(tracking_ ? (aim_ready_ ? "AUTO AIM" : "AUTO SEARCH") : (position_mode_ ? "TERMINAL TARGET" : "MANUAL"));
        if (now-panel_time_ > std::chrono::milliseconds(100)) {
            terminal_.status(measuredYaw(), measuredPitch(), valid, hasFocus(), moving || position_moving, last_fire_request_, tracking_, aim_ready_, position_mode_);
            panel_time_=now;
        }
        processing_ms_=std::chrono::duration<double,std::milli>(Clock::now()-now).count();
        update();
    }
protected:
    void keyPressEvent(QKeyEvent* e) override {
        if (e->isAutoRepeat()) { e->accept(); return; }
        const int i=index(e->key());
        if (i>=0) {
            if (tracking_ || position_mode_) stop("方向键切回手动");
            held_[i]=true;
        }
        else if (e->key()==Qt::Key_Space) {
            if (!position_mode_ && initialized_ && fresh() && error_.isEmpty() && isActiveWindow() && hasFocus()) {
                fire_held_=true;
                commanded_=true;
                send(true);
            }
        }
        else if (e->key()==Qt::Key_Q || e->key()==Qt::Key_Escape) close();
        else if (e->key()==Qt::Key_R) {
            const bool enable=!tracking_;
            position_mode_=false;
            if (!enable) fire_held_=false;
            resetAim();
            held_.fill(false);
            if (initialized_) { yaw_=measuredYaw(); pitch_=measuredPitch(); }
            tracking_=enable && initialized_ && fresh() && error_.isEmpty();
            last_event_=tracking_ ? "R：追踪启用" : (enable ? "R：未启用，"+blockedReason() : "R：追踪关闭");
            commanded_=true;
            send(fresh());
        }
        else if (e->key()==Qt::Key_B || e->key()==Qt::Key_F) {
            color_ = e->key()==Qt::Key_B ? ENEMY_BLUE : ENEMY_RED;
            resetAim();
            fire_held_=false;
            if (commanded_) send(fresh());
        }
        e->accept();
    }
    void keyReleaseEvent(QKeyEvent* e) override {
        if (e->isAutoRepeat()) { e->accept(); return; }
        const int i=index(e->key());
        if (e->key()==Qt::Key_Space) {
            fire_held_=false;
            if (commanded_) send(fresh()); // Clear fire immediately, not after another frame.
        }
        if (i>=0) {
            held_[i]=false;
            // Cancel setpoint lead immediately on release, before the next timer tick.
            if (!tracking_ && initialized_) { yaw_=measuredYaw(); pitch_=measuredPitch(); }
            if (commanded_) send(fresh());
        }
        e->accept();
    }
    void focusOutEvent(QFocusEvent* e) override { if (!position_mode_) stop("窗口失焦"); QWidget::focusOutEvent(e); }
    void changeEvent(QEvent* e) override {
        if (e->type()==QEvent::ActivationChange && !isActiveWindow() && !position_mode_) stop("窗口不再激活");
        QWidget::changeEvent(e);
    }
    void mousePressEvent(QMouseEvent* e) override { setFocus(); QWidget::mousePressEvent(e); }
    void closeEvent(QCloseEvent* e) override { timer_.stop(); stop("退出窗口"); e->accept(); }
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.fillRect(rect(), Qt::black);
        QFont text_font("Sans Serif",15);
        text_font.setBold(true); p.setFont(text_font);
        const int line=p.fontMetrics().height()+7;
        const int top=12;
        p.setPen(Qt::green);
        const QString current=have_feedback_ ? QString("%1  %2").arg(measuredYaw(),0,'f',2).arg(measuredPitch(),0,'f',2) : "* *";
        p.drawText(12,top+line,QString("当前指向 (yaw pitch / 度)：%1").arg(current));
        p.setPen(Qt::yellow);
        const QString predicted=tracking_ && fresh() && predicted_direction_ ?
            QString("%1  %2").arg(predicted_direction_->first,0,'f',2).arg(predicted_direction_->second,0,'f',2) : "* *";
        p.drawText(12,top+line*2,QString("追踪预测点方向 (度)：%1").arg(predicted));
        p.setPen(Qt::cyan);
        const QString target=terminal_target_ ? QString("%1  %2").arg(terminal_target_->first).arg(terminal_target_->second) : "* *";
        p.drawText(12,top+line*3,QString("终端目标 (度)：%1  [%2]").arg(target,position_mode_ ? "启用" : "未启用"));
        text_font.setPointSize(10); text_font.setBold(false); p.setFont(text_font);
        p.setPen(Qt::white);
        const int image_top=top+line*3+60;
        p.drawText(QRect(12,top+line*3+8,width()-24,45),Qt::TextWordWrap,state_);
        text_font.setPointSize(12); p.setFont(text_font);
        const QStringList details=diagnosticLines();
        const int detail_line=p.fontMetrics().height()+3;
        std::vector<int> detail_heights;
        int wrapped_height=detail_line+10;
        for (const auto& detail : details) {
            const int rows_height=std::max(detail_line,p.fontMetrics().boundingRect(
                QRect(0,0,std::max(1,width()-24),10000),Qt::TextWordWrap,detail).height()+3);
            detail_heights.push_back(rows_height);
            wrapped_height+=rows_height;
        }
        const int detail_height=wrapped_height;
        const int detail_top=std::max(image_top+30,height()-detail_height);
        if (!picture_.isNull()) {
            QSize size=picture_.size(); size.scale(width(),std::max(1,height()-image_top),Qt::KeepAspectRatio);
            p.drawImage(QRect(QPoint((width()-size.width())/2,image_top),size),picture_);
        }
        p.fillRect(QRect(0,detail_top,width(),height()-detail_top),QColor(24,27,32));
        p.setPen(Qt::cyan);
        p.drawText(12,detail_top+detail_line,"诊断参数（始终显示；TX仅为调用记录，无发送成功或电控执行确认）");
        int y=detail_top+detail_line+4;
        for (int i=0;i<details.size();++i) {
            p.setPen(i==details.size()-1 ? QColor(Qt::yellow) : QColor(Qt::white));
            p.drawText(QRect(12,y,width()-24,detail_heights[i]),Qt::TextWordWrap,details[i]);
            y+=detail_heights[i];
        }
        if (!error_.isEmpty()) {
            p.fillRect(QRect(0,image_top,width(),35),QColor(80,0,0));
            p.setPen(Qt::white); p.drawText(12,image_top+24,error_);
        }
    }
public:
    Preview(io::HikRobot& camera, io::SerialPort& serial, ArmorDetector& detector, AutoArmor& auto_armor,
            TerminalPanel& terminal, tools::Exiter& exiter, double speed, int ys, int ps, int color)
        : camera_(camera), serial_(serial), detector_(detector), auto_armor_(auto_armor), terminal_(terminal), exiter_(exiter),
          speed_(speed), yaw_sign_(ys), pitch_sign_(ps), color_(color) {
        setWindowTitle("Keyboard Gimbal / Armor Pixels");
        setFocusPolicy(Qt::StrongFocus);
        QSize window_size(1320,940);
        if (const auto* screen=QGuiApplication::primaryScreen()) {
            const auto available=screen->availableGeometry().size();
            window_size.setWidth(std::min(window_size.width(),available.width()*95/100));
            window_size.setHeight(std::min(window_size.height(),available.height()*95/100));
        }
        resize(window_size);
        stdin_notifier_=std::make_unique<QSocketNotifier>(STDIN_FILENO,QSocketNotifier::Read,this);
        QObject::connect(stdin_notifier_.get(),&QSocketNotifier::activated,this,[this] {
            try { readTerminal(); }
            catch (const std::exception& e) { error_=QString::fromUtf8(e.what()); stop(); update(); }
        });
        QObject::connect(&timer_, &QTimer::timeout, this, [this] {
            try { tick(); }
            catch (const std::exception& e) { error_=QString::fromUtf8(e.what()); stop(); update(); }
        });
        timer_.start(20);
    }
};

int main(int argc, char** argv) {
    if (argc<2 || std::string(argv[1])=="--help") {
        std::cout << "Usage: keyboard_gimbal_test <text.yaml> [serial=/dev/ttyACM0] [color=0] [speed_deg_s=5] [yaw_sign=1] [pitch_sign=1]\n"
                     "Terminal: integer yaw pitch + Enter; empty line cancels. Camera window: arrows to move.\n"
                     "R: toggle auto aim; arrows: manual; Space: hold fire; B/F: blue/red; Diagnostics always visible; Q/Esc: quit.\n"
                     "Manual yaw travel +/-10deg; no pitch angle clamp. AUTO uses existing tracking/aim/Shooter. Fire needs Space.\n";
        return argc<2 ? 1 : 0;
    }
    try {
        const auto path=std::filesystem::absolute(argv[1]);
        const std::string device=argc>2 ? argv[2] : "/dev/ttyACM0";
        const int color=argc>3 ? std::stoi(argv[3]) : 0;
        const double speed=argc>4 ? std::stod(argv[4]) : 5.0;
        const int ys=argc>5 ? std::stoi(argv[5]) : 1;
        const int ps=argc>6 ? std::stoi(argv[6]) : 1;
        if ((color!=0 && color!=1) || !std::isfinite(speed) || speed<=0 || speed>20 ||
            (ys!=1 && ys!=-1) || (ps!=1 && ps!=-1)) throw std::invalid_argument("Invalid color/speed/signs");
        // Only suppress this executable's console sink. Keep full file logs.
        const auto log=tools::logger();
        for (const auto& sink : log->sinks())
            if (std::dynamic_pointer_cast<spdlog::sinks::stdout_color_sink_mt>(sink))
                sink->set_level(spdlog::level::off);
        QApplication app(argc,argv);
        TerminalPanel terminal;
        const auto config=YAML::LoadFile(path.string());
        const auto models=path.parent_path()/"model";
        ArmorDetector detector((models/"mlp.onnx").string(),(models/"label.txt").string(),
                               0.7,180,std::vector<std::string>{"negative"});
        AutoArmor auto_armor(path.string(),true);
        tools::Exiter exiter;
        // Preview handles stale feedback; do not terminate the GUI from the serial thread.
        io::SerialPort serial(device.c_str(),false);
        if (serial.fd<0) throw std::runtime_error("Cannot open serial: "+device);
        io::HikRobot camera(config["Camera"]["exposure_armor"].as<double>(),0,"2bdf:0001");
        Preview window(camera,serial,detector,auto_armor,terminal,exiter,speed,ys,ps,color);
        window.show(); window.setFocus();
        return app.exec();
    } catch (const std::exception& e) {
        std::cerr << "Preview: " << e.what() << '\n'; return 1;
    }
}
