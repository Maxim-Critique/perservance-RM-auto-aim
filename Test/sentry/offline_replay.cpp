#include "AutoArmor.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>

namespace {
int32_t angle(const Data_Get& data, int offset)
{
  uint32_t bits=0;
  for(int i=0;i<4;++i) bits=(bits<<8)|data.raw_data[offset+i];
  int32_t value;std::memcpy(&value,&bits,sizeof(value));return value;
}
}

// Video-only fixture. Never constructs a camera, SerialPort, ROS2, or Buff object.
int main(int argc,char** argv) try
{
  if(argc!=5) {
    std::cerr<<"Usage: sentry_offline_replay <config.yaml> <video.avi> <frame_limit> <output.csv>\n";
    return 2;
  }
  const int limit=std::stoi(argv[3]);
  if(limit<1) throw std::invalid_argument("frame_limit must be positive");
  const auto config=YAML::LoadFile(argv[1]);
  // Deterministic single-thread fixture for comparable old/new outputs.
  cv::setNumThreads(1);
  AutoArmor pipeline(argv[1],false);
  cv::VideoCapture video(argv[2]);
  if(!video.isOpened()) throw std::runtime_error("cannot open video");
  const double fps=video.get(cv::CAP_PROP_FPS);
  if(!std::isfinite(fps) || fps<=0) throw std::runtime_error("invalid video FPS");
  std::ofstream csv(argv[4]);
  if(!csv) throw std::runtime_error("cannot create CSV output");
  csv<<"frame,time_s,mode,found,id,yaw_deg,pitch_deg,fire_output,latency_ms\n";
  Data_Save feedback{};feedback.armor_or_buff=0;feedback.color=0;
  Data_Get command;
  std::vector<double> latency;
  int count=0,found=0;
  const auto epoch=time_point{}+std::chrono::seconds(100);
  for(;count<limit;++count) {
    cv::Mat image;if(!video.read(image)) break;
    feedback.timestamp=epoch+std::chrono::duration_cast<time_point::duration>(std::chrono::duration<double>(count/fps));
    const auto begin=std::chrono::steady_clock::now();
    pipeline.AutoAim(image,feedback,command);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    latency.push_back(ms);
    // Retain diagnostic target/angles, but no fire flag leaves this fixture even in the CSV.
    command.raw_data[19]=0;
    found+=command.raw_data[18]!=0;
    const double yaw=angle(command,2)*1e-4,pitch=angle(command,6)*1e-4;
    if(!std::isfinite(yaw) || !std::isfinite(pitch)) throw std::runtime_error("nonfinite output");
    csv<<count<<','<<count/fps<<",0,"<<int(command.raw_data[18])<<','<<int(command.raw_data[17])
       <<','<<yaw<<','<<pitch<<",0,"<<ms<<'\n';
  }
  if(count==0) throw std::runtime_error("video contained no decodable frames");
  const double mean=std::accumulate(latency.begin(),latency.end(),0.)/count;
  std::sort(latency.begin(),latency.end());
  std::cout<<"frames="<<count<<"\nfound_frames="<<found<<"\nmean_pipeline_ms="<<mean
           <<"\np95_pipeline_ms="<<latency[std::min(count-1,int(count*.95))]
           <<"\nbullet_speed_m_s="<<bullet_speed
           <<"\nyaw_compensation_deg="<<config["Transmit"]["compensate_yaw"].as<double>()
           <<"\npitch_compensation_deg="<<config["Transmit"]["compensate_pitch"].as<double>()
           <<"\nprediction_delay_s="<<config["Transmit"]["predict_time"].as<double>()
           <<"\nfixture=video BGR, zero Euler attitude; no position/accuracy validation"
           <<"\nhardware_io=none\nfire_output=disabled\n" << std::flush;
  return 0;
} catch(const std::exception& e) {
  std::cerr<<"sentry_offline_replay: "<<e.what()<<'\n';return 1;
}
