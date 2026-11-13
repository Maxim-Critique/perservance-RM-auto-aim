// Read-only qualitative preview: image-space tracking, no gimbal/serial/shooting output.
// Uses the existing detector and tools::ExtendedKalmanFilter; no camera calibration is assumed.
#include "ArmorDetector.h"
#include "extended_kalman_filter.hpp"
#include "hikrobot.hpp"
#include "exiter.hpp"
#include "logger.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>

namespace {
using Clock = std::chrono::steady_clock;
constexpr const char* window_name = "Camera detection / pixel prediction (no control)";

// A single-board image-space model: [u, du/dt, v, dv/dt], pixels and seconds.
// This tests visual continuity without reusing mismatched real-robot extrinsics or intrinsics.
class PixelTrack {
    std::optional<tools::ExtendedKalmanFilter> filter_;
    time_point time_{}, observed_time_{};
public:
    std::optional<cv::Point2d> observed, predicted;
    cv::Point2d velocity{};

    void reset() {
        filter_.reset(); observed.reset(); predicted.reset(); velocity = {};
        time_ = observed_time_ = time_point{};
    }
    void update(const Armors& armors, time_point timestamp, cv::Size size, double lead) {
        observed.reset(); predicted.reset();
        if (filter_) {
            const double dt = std::chrono::duration<double>(timestamp-time_).count();
            if (dt <= 0 || dt > 0.25) reset();
            else {
                Eigen::MatrixXd f = Eigen::MatrixXd::Identity(4,4);
                f(0,1) = f(2,3) = dt;
                Eigen::MatrixXd q = Eigen::MatrixXd::Zero(4,4);
                Eigen::Matrix2d block;
                block << std::pow(dt,4)/4, std::pow(dt,3)/2,
                         std::pow(dt,3)/2, dt*dt;
                q.block<2,2>(0,0) = q.block<2,2>(2,2) = block*40000.0;
                filter_->predict(f,q);
                if (!filter_->x.allFinite() || !filter_->P.allFinite()) reset();
            }
        }
        time_ = timestamp;
        const cv::Point2d reference = filter_ ? cv::Point2d(filter_->x[0],filter_->x[2]) :
                                                              cv::Point2d(size.width/2.0,size.height/2.0);
        const Armor* closest = nullptr;
        double distance = std::numeric_limits<double>::infinity();
        for (const auto& armor : armors) {
            if (armor.points.size()!=4 || !std::isfinite(armor.center.x) || !std::isfinite(armor.center.y)) continue;
            const double candidate = cv::norm(cv::Point2d(armor.center)-reference);
            if (candidate < distance) { distance=candidate; closest=&armor; }
        }
        if (closest) {
            observed = cv::Point2d(closest->center);
            // Large discontinuities reacquire instead of interpreting a new board as a huge velocity.
            if (filter_ && distance > std::max(80.0,std::hypot(size.width,size.height)*0.2)) filter_.reset();
            if (!filter_) {
                Eigen::VectorXd x(4); x << observed->x,0,observed->y,0;
                Eigen::MatrixXd covariance=Eigen::MatrixXd::Zero(4,4);
                covariance.diagonal() << 25,10000,25,10000;
                filter_.emplace(x,covariance);
            } else {
                Eigen::MatrixXd h=Eigen::MatrixXd::Zero(2,4);
                h(0,0)=h(1,2)=1;
                Eigen::Vector2d measurement(observed->x,observed->y);
                filter_->update(measurement,h,Eigen::Matrix2d::Identity()*4.0);
            }
            observed_time_=timestamp;
        } else if (!filter_ || timestamp-observed_time_ > std::chrono::milliseconds(150)) {
            reset(); return;
        }
        if (filter_) {
            if (!filter_->x.allFinite() || !filter_->P.allFinite()) { reset(); return; }
            velocity={filter_->x[1],filter_->x[3]};
            predicted=cv::Point2d(filter_->x[0]+velocity.x*lead,filter_->x[2]+velocity.y*lead);
        }
    }
};

std::string pointText(const std::optional<cv::Point2d>& point) {
    return point ? cv::format("%.1f, %.1f",point->x,point->y) : "* *";
}

cv::Mat render(const cv::Mat& frame, const Armors& armors, const PixelTrack& track,
               double lead, int color, double process_ms, const std::string& status) {
    cv::Mat view;
    const double scale=std::min(960.0/frame.cols,600.0/frame.rows);
    cv::resize(frame,view,cv::Size(),scale,scale);
    const int panel=140, offset=(std::max(800,view.cols)-view.cols)/2;
    cv::Mat canvas(view.rows+panel,std::max(800,view.cols),CV_8UC3,cv::Scalar(20,20,20));
    view.copyTo(canvas(cv::Rect(offset,panel,view.cols,view.rows)));
    const auto pixel=[&](cv::Point2d p) { return cv::Point(cvRound(p.x*scale)+offset,cvRound(p.y*scale)+panel); };
    for (const auto& armor : armors) {
        if (armor.points.size()!=4) continue;
        for (int i=0;i<4;++i)
            cv::line(canvas,pixel(armor.points[i]),pixel(armor.points[(i+1)%4]),cv::Scalar(0,180,0),2);
    }
    if (track.observed) {
        cv::circle(canvas,pixel(*track.observed),9,cv::Scalar(0,255,0),2);
        cv::drawMarker(canvas,pixel(*track.observed),cv::Scalar(0,255,0),cv::MARKER_CROSS,20,2);
    }
    if (track.predicted && std::abs(track.predicted->x)<1e6 && std::abs(track.predicted->y)<1e6) {
        const auto p=pixel(*track.predicted);
        const cv::Rect image_bounds(offset,panel,view.cols,view.rows);
        if (image_bounds.contains(p)) cv::drawMarker(canvas,p,cv::Scalar(255,0,255),cv::MARKER_TILTED_CROSS,24,3);
        if (track.observed) {
            auto from=pixel(*track.observed), to=p;
            if (cv::clipLine(image_bounds,from,to)) cv::arrowedLine(canvas,from,to,cv::Scalar(255,0,255),2);
        }
    }
    cv::putText(canvas,"CURRENT (green, px): "+pointText(track.observed),{12,27},cv::FONT_HERSHEY_SIMPLEX,0.7,{0,255,0},2);
    cv::putText(canvas,cv::format("PREDICT +%.0fms (magenta, px): ",lead*1000)+pointText(track.predicted),
                {12,56},cv::FONT_HERSHEY_SIMPLEX,0.7,{255,0,255},2);
    cv::putText(canvas,cv::format("%s | %s | pairs:%d | process:%.1fms | v:%.0f,%.0f px/s",
                status.c_str(),color==ENEMY_BLUE ? "BLUE" : "RED",int(armors.size()),process_ms,track.velocity.x,track.velocity.y),
                {12,85},cv::FONT_HERSHEY_SIMPLEX,0.55,{255,255,255},1);
    cv::putText(canvas,"2D QUALITATIVE ONLY | B/F: blue/red | R: reset | Q/ESC: quit | NO CONTROL",
                {12,113},cv::FONT_HERSHEY_SIMPLEX,0.53,{0,220,255},1);
    return canvas;
}

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// Synthetic checks run before any camera object is constructed.
int selfTest() {
    require(ArmorDetector::kLightBarsOnly,"Self-test expects the current light-bars-only mode");
    ArmorDetector detector("/nonexistent/model.onnx","/nonexistent/labels.txt",0.7);
    PixelTrack track;
    const time_point epoch=Clock::now();
    for (int i=0;i<80;++i) {
        cv::Mat image=cv::Mat::zeros(300,700,CV_8UC3);
        const int x=100+i*3;
        cv::rectangle(image,{x,110,12,70},{255,0,0},cv::FILLED);
        cv::rectangle(image,{x+120,110,12,70},{255,0,0},cv::FILLED);
        const auto armors=detector.Detect(image,ENEMY_BLUE);
        require(!armors.empty(),"Synthetic lights must be detected without digits");
        track.update(armors,epoch+std::chrono::milliseconds(20*i),image.size(),0.15);
    }
    require(track.observed && track.predicted,"Observed and predicted points must exist");
    require(track.predicted->x>track.observed->x+10,"Prediction must lead horizontal motion");
    require(std::abs(track.predicted->y-track.observed->y)<5,"Horizontal motion must not invent vertical drift");
    const auto preview=render(cv::Mat::zeros(300,700,CV_8UC3),{},track,0.15,ENEMY_BLUE,0,"SELF-TEST");
    require(!preview.empty(),"Overlay must render without a window or camera");
    track.update({},epoch+std::chrono::milliseconds(1600),{700,300},0.15);
    require(!track.observed && track.predicted,"Brief loss may retain prediction, never a fake observation");
    track.update({},epoch+std::chrono::milliseconds(1800),{700,300},0.15);
    require(!track.observed && !track.predicted,"Long loss must clear the points");
    std::cout << "PASS synthetic lights, pixel prediction, overlay rendering and loss handling; no hardware opened\n";
    return 0;
}
}

int main(int argc, char** argv) {
    try {
        if (argc>1 && std::string(argv[1])=="--self-test") return selfTest();
        if (argc>1 && std::string(argv[1])=="--help") {
            std::cout << "Usage: camera_detection_preview [hik|usb:0] [color=0] [prediction_s=0.15] [exposure_us=2500]\n"
                         "No calibration or serial connection required. Prediction is in image pixels, not a ballistic aim point.\n";
            return 0;
        }
        const std::string source=argc>1 ? argv[1] : "hik";
        const int color_argument=argc>2 ? std::stoi(argv[2]) : ENEMY_BLUE;
        int color=color_argument;
        const double lead=argc>3 ? std::stod(argv[3]) : 0.15;
        const double exposure=argc>4 ? std::stod(argv[4]) : 2500.0;
        if ((color!=0 && color!=1) || !std::isfinite(lead) || lead<0 || lead>0.5 ||
            !std::isfinite(exposure) || exposure<=0) throw std::invalid_argument("Invalid color/horizon/exposure");
        int usb_index=-1;
        if (source!="hik") {
            const auto suffix=source.substr(std::min<size_t>(4,source.size()));
            if (source.rfind("usb:",0)!=0 || suffix.empty() || suffix.find_first_not_of("0123456789")!=std::string::npos)
                throw std::invalid_argument("Camera source must be hik or usb:N");
            usb_index=std::stoi(suffix);
        }
        for (const auto& sink : tools::logger()->sinks())
            if (std::dynamic_pointer_cast<spdlog::sinks::stdout_color_sink_mt>(sink)) sink->set_level(spdlog::level::off);
        const auto models=std::filesystem::path(PREVIEW_MODEL_DIR);
        ArmorDetector detector((models/"mlp.onnx").string(),(models/"label.txt").string(),0.7);
        tools::Exiter exiter;
        std::unique_ptr<io::HikRobot> hik;
        cv::VideoCapture usb;
        if (source=="hik") hik=std::make_unique<io::HikRobot>(exposure,0.0,"2bdf:0001");
        else if (!usb.open(usb_index)) throw std::runtime_error("Cannot open USB camera");
        cv::namedWindow(window_name,cv::WINDOW_AUTOSIZE);
        PixelTrack track;
        time_point last_frame{};
        bool first_frame=true;
        cv::Mat display=render(cv::Mat::zeros(480,640,CV_8UC3),{},track,lead,color,0,"WAITING FOR CAMERA");
        std::cout << "Preview only: green=current, magenta=future pixel. B/F switches color; R resets; Q/ESC exits.\n";
        while (!exiter.exit()) {
            cv::Mat frame;
            time_point timestamp;
            const bool got=hik ? hik->TryQuery(frame,timestamp) : usb.read(frame);
            if (got && !frame.empty()) {
                if (first_frame) {
                    std::cout << "First camera frame: " << frame.cols << "x" << frame.rows << '\n';
                    first_frame=false;
                }
                if (!hik) timestamp=Clock::now();
                last_frame=timestamp;
                const auto start=Clock::now();
                const auto armors=detector.Detect(frame,color);
                track.update(armors,timestamp,frame.size(),lead);
                const double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
                display=render(frame,armors,track,lead,color,ms,track.observed ? "OBSERVED" : (track.predicted ? "PREDICTION ONLY" : "NO TARGET"));
            } else if (last_frame==time_point{} || Clock::now()-last_frame>std::chrono::milliseconds(250)) {
                track.reset();
                display=render(cv::Mat::zeros(480,640,CV_8UC3),{},track,lead,color,0,"WAITING / CAMERA STALE");
            }
            cv::imshow(window_name,display);
            const int key=cv::waitKey(10);
            if (key==27 || key=='q' || key=='Q') {
                std::cout << "Preview stopped by Q/ESC.\n";
                break;
            }
            // OpenCV 4.5.4 GTK does not implement WND_PROP_VISIBLE (returns -1).
            // AUTOSIZE is supported by GTK and returns -1 when this window is gone.
            if (cv::getWindowProperty(window_name,cv::WND_PROP_AUTOSIZE)<0) {
                std::cout << "Preview window was closed.\n";
                break;
            }
            if (key=='b' || key=='B' || key=='f' || key=='F') {
                color=(key=='b' || key=='B') ? ENEMY_BLUE : ENEMY_RED;
                track.reset();
            }
            if (key=='r' || key=='R') track.reset();
        }
        cv::destroyAllWindows();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Camera preview: " << e.what() << '\n';
        return 1;
    }
}
