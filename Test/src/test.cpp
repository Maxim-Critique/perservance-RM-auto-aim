// Manual camera/gimbal preview. Target: review. Space holds a manual fire request.
#include "ArmorDetector.h"
#include "SerialPort.h"
#include "hikrobot.hpp"
#include "exiter.hpp"
#include "logger.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <QApplication>
#include <QCloseEvent>
#include <QFocusEvent>
#include <QImage>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

using Clock = std::chrono::steady_clock;

// A fixed terminal panel; lifecycle messages remain in the file log.
class TerminalPanel {
    bool tty_ = isatty(STDOUT_FILENO);
public:
    TerminalPanel() {
        if (tty_) std::cout << "\033[?1049h\033[?25l\033[2J";
        std::cout << "方向键：按住转动，松手保持当前反馈角度\n"
                     "先点击相机窗口，使其获得焦点；失焦自动停止\n"
                     "空格：按住请求发射，松开停止；B / R：蓝 / 红目标；Q / Esc：退出\n"
                     "像素坐标：原图左上为 (0,0)，x 向右、y 向下\n" << std::flush;
    }
    void status(double yaw, double pitch, bool fresh, bool focused, bool moving, bool fire) {
        if (!tty_) return;
        std::cout << "\033[6;1H\033[2K" << "反馈 yaw=" << yaw << "° pitch=" << pitch
                  << "° | " << (fresh ? "数据正常" : "等待数据/已过期")
                  << " | " << (focused ? "窗口有焦点" : "窗口失焦")
                  << " | " << (moving ? "移动" : "停止")
                  << " | 发射请求=" << (fire ? 1 : 0) << std::flush;
    }
    ~TerminalPanel() {
        if (tty_) std::cout << "\033[?25h\033[?1049l" << std::flush;
    }
};

class Preview : public QWidget {
    io::HikRobot& camera_;
    io::SerialPort& serial_;
    ArmorDetector& detector_;
    TerminalPanel& terminal_;
    tools::Exiter& exiter_;
    QTimer timer_;
    std::array<bool, 4> held_{};
    Data_Save feedback_{};
    time_point image_time_{}, last_tick_ = Clock::now(), panel_time_{};
    bool have_feedback_ = false, initialized_ = false, commanded_ = false;
    bool fire_held_ = false, last_fire_request_ = false;
    double yaw_ = 0, pitch_ = 0, origin_yaw_ = 0, origin_pitch_ = 0;
    double speed_, yaw_sign_, pitch_sign_;
    int color_;
    QImage picture_;
    cv::Mat frame_;
    QString state_ = "Waiting for camera and feedback";
    QString error_;
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
    void send(bool valid) {
        valid = valid && initialized_ && fresh() && error_.isEmpty();
        const bool fire = valid && isActiveWindow() && hasFocus() && fire_held_;
        Data_Get packet;
        // Keep manual turning found=2; reuse armor adapter.cpp's boolean fire=1/0.
        // Controller support for found=2 plus fire=1 must be confirmed on its side.
        packet.Vision2Control(static_cast<int32_t>(std::llround(yaw_*10000)),
                              static_cast<int32_t>(std::llround(pitch_*10000)), 0,
                              valid ? 2 : 0, static_cast<uint8_t>(fire));
        packet.Nav2Control(0, 0);
        serial_.SendData(packet);
        last_fire_request_ = fire;
    }
    void stop() {
        held_.fill(false);
        fire_held_ = false;
        if (initialized_) { yaw_ = measuredYaw(); pitch_ = measuredPitch(); }
        if (commanded_) send(fresh());
    }
    void drawFrame() {
        auto display = frame_.clone();
        const auto armors = detector_.Detect(display, color_);
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
            cv::arrowedLine(display, screen_center, candidate, cv::Scalar(255,0,255), 2);
            cv::putText(display, cv::format("Candidate pixel (%.1f, %.1f)", candidate.x, candidate.y),
                        cv::Point(15,30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,0,255), 2);
        } else {
            cv::putText(display, "No candidate", cv::Point(15,30), cv::FONT_HERSHEY_SIMPLEX,
                        0.7, cv::Scalar(0,255,255), 2);
        }
        cv::Mat rgb;
        cv::cvtColor(display, rgb, cv::COLOR_BGR2RGB);
        picture_ = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
    void tick() {
        if (exiter_.exit()) { close(); return; }
        const auto now = Clock::now();
        const double dt = std::clamp(std::chrono::duration<double>(now-last_tick_).count(), 0.0, 0.05);
        last_tick_ = now;
        Data_Save sample{};
        if (serial_.TryLatest(sample)) {
            if (std::isfinite(sample.camera_yaw_angle) && std::isfinite(sample.camera_pitch_angle)) {
                feedback_=sample; have_feedback_=true;
            }
        }
        cv::Mat image;
        time_point timestamp;
        const bool new_frame = camera_.TryQuery(image, timestamp) && !image.empty();
        if (new_frame) { frame_=image; image_time_=timestamp; }
        if (!initialized_ && fresh()) {
            origin_yaw_=yaw_=measuredYaw(); origin_pitch_=pitch_=measuredPitch(); initialized_=true;
        }
        const bool valid = initialized_ && fresh() && error_.isEmpty();
        const int dy = int(held_[2])-int(held_[3]);
        const int dp = int(held_[0])-int(held_[1]);
        const bool moving = valid && isActiveWindow() && hasFocus() && (dy || dp);
        if (moving) {
            yaw_ += dy*yaw_sign_*speed_*dt;
            pitch_ += dp*pitch_sign_*speed_*dt;
            // Bound total travel and avoid a distant setpoint accumulating behind the mechanism.
            yaw_=std::clamp(yaw_, origin_yaw_-10.0, origin_yaw_+10.0);
            pitch_=std::clamp(pitch_, origin_pitch_-10.0, origin_pitch_+10.0);
            yaw_=std::clamp(yaw_, measuredYaw()-1.0, measuredYaw()+1.0);
            pitch_=std::clamp(pitch_, measuredPitch()-1.0, measuredPitch()+1.0);
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
        state_=QString("Yaw %1 deg | Pitch %2 deg | Command %3 / %4 | %5 | %6 | FIRE REQUEST %7")
            .arg(measuredYaw(),0,'f',2).arg(measuredPitch(),0,'f',2)
            .arg(yaw_,0,'f',2).arg(pitch_,0,'f',2)
            .arg(valid ? "FEEDBACK OK" : "STALE / WAITING").arg(moving ? "MOVING" : "STOPPED")
            .arg(last_fire_request_ ? 1 : 0);
        if (new_frame && error_.isEmpty()) drawFrame();
        if (now-panel_time_ > std::chrono::milliseconds(100)) {
            terminal_.status(measuredYaw(), measuredPitch(), valid, hasFocus(), moving, last_fire_request_);
            panel_time_=now;
        }
        update();
    }
protected:
    void keyPressEvent(QKeyEvent* e) override {
        if (e->isAutoRepeat()) { e->accept(); return; }
        const int i=index(e->key());
        if (i>=0) held_[i]=true;
        else if (e->key()==Qt::Key_Space) {
            if (initialized_ && fresh() && error_.isEmpty() && isActiveWindow() && hasFocus()) {
                fire_held_=true;
                commanded_=true;
                send(true);
            }
        }
        else if (e->key()==Qt::Key_Q || e->key()==Qt::Key_Escape) close();
        else if (e->key()==Qt::Key_B) color_=ENEMY_BLUE;
        else if (e->key()==Qt::Key_R) color_=ENEMY_RED;
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
            if (initialized_) { yaw_=measuredYaw(); pitch_=measuredPitch(); }
            if (commanded_) send(fresh());
        }
        e->accept();
    }
    void focusOutEvent(QFocusEvent* e) override { stop(); QWidget::focusOutEvent(e); }
    void changeEvent(QEvent* e) override {
        if (e->type()==QEvent::ActivationChange && !isActiveWindow()) stop();
        QWidget::changeEvent(e);
    }
    void mousePressEvent(QMouseEvent* e) override { setFocus(); QWidget::mousePressEvent(e); }
    void closeEvent(QCloseEvent* e) override { timer_.stop(); stop(); e->accept(); }
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.fillRect(rect(), Qt::black);
        p.setPen(Qt::green);
        p.drawText(QRect(10,5,width()-20,50), Qt::TextWordWrap, state_);
        if (!error_.isEmpty()) { p.setPen(Qt::red); p.drawText(10,75,error_); }
        if (!picture_.isNull()) {
            QSize size=picture_.size(); size.scale(width(),std::max(1,height()-90),Qt::KeepAspectRatio);
            p.drawImage(QRect(QPoint((width()-size.width())/2,90),size),picture_);
        }
    }
public:
    Preview(io::HikRobot& camera, io::SerialPort& serial, ArmorDetector& detector,
            TerminalPanel& terminal, tools::Exiter& exiter, double speed, int ys, int ps, int color)
        : camera_(camera), serial_(serial), detector_(detector), terminal_(terminal), exiter_(exiter),
          speed_(speed), yaw_sign_(ys), pitch_sign_(ps), color_(color) {
        setWindowTitle("Keyboard Gimbal / Armor Pixels");
        setFocusPolicy(Qt::StrongFocus); resize(1100,850);
        QObject::connect(&timer_, &QTimer::timeout, this, [this] {
            try { tick(); }
            catch (const std::exception& e) { error_=QString::fromUtf8(e.what()); stop(); update(); }
        });
        timer_.start(20);
    }
};

int main(int argc, char** argv) {
    if (argc<2 || std::string(argv[1])=="--help") {
        std::cout << "Usage: review <text.yaml> [serial=/dev/ttyACM0] [color=0] [speed_deg_s=5] [yaw_sign=1] [pitch_sign=1]\n"
                     "Focus camera window. Hold arrows to move, release to hold feedback angle.\n"
                     "Space: hold to request fire, release to stop; B/R: blue/red; Q/Esc: quit.\n"
                     "Travel +/-10deg. Armor fire encoding=1/0, manual found=2; confirm controller support.\n";
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
        tools::Exiter exiter;
        // Preview handles stale feedback; do not terminate the GUI from the serial thread.
        io::SerialPort serial(device.c_str(),false);
        if (serial.fd<0) throw std::runtime_error("Cannot open serial: "+device);
        io::HikRobot camera(config["Camera"]["exposure_armor"].as<double>(),0,"2bdf:0001");
        Preview window(camera,serial,detector,terminal,exiter,speed,ys,ps,color);
        window.show(); window.setFocus();
        return app.exec();
    } catch (const std::exception& e) {
        std::cerr << "Preview: " << e.what() << '\n'; return 1;
    }
}
