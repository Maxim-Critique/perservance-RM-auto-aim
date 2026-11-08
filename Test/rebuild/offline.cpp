#include "tongji_adapter.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <sys/resource.h>
int main(int argc,char **argv) {
  try {
    if(argc<6 || argc>8)throw std::runtime_error("Usage: fusion_offline car.yaml algorithm.yaml video frame_limit output.csv [original_car_metadata.txt] [input_rgb=0|1]");
    cv::setNumThreads(1);
    fusion::TongjiAdapter adapter(argv[1],argv[2]);
    cv::VideoCapture video(argv[3]);if(!video.isOpened())throw std::runtime_error("Cannot open video");
    std::ofstream out(argv[5]);if(!out)throw std::runtime_error("Cannot open output CSV");
    std::ifstream metadata;if(argc>=7){metadata.open(argv[6]);if(!metadata)throw std::runtime_error("Cannot open metadata");}
    const bool input_rgb = argc == 8 ? std::stoi(argv[7]) != 0 : metadata.is_open();
    out<<"frame,name,type_prior,confidence,ratio,accepted_prior,latency_ms,tracker,x0,y0,x1,y1,x2,y2,x3,y3\n";
    const int limit=std::stoi(argv[4]);int count=0,raw_count=0,accepted_count=0;
    std::vector<double> times;cv::Mat image;
    while((limit<=0 || count<limit) && video.read(image)) {
      auto begin=std::chrono::steady_clock::now();auto detections=adapter.detect_only(image,input_rgb);
      Data_Save d{};int mode=0;
      if(metadata.is_open()) {
        double since;std::string line;if(!std::getline(metadata,line))throw std::runtime_error("Metadata shorter than video");
        std::istringstream stream(line);
        if(!(stream>>since>>d.camera_yaw_angle>>d.camera_pitch_angle>>d.camera_roll_angle>>d.go_back>>d.armor_or_buff>>d.color)||!std::isfinite(since)||since<0||since>1e7)
          throw std::runtime_error("Malformed original metadata");
        d.timestamp=time_point(std::chrono::duration_cast<time_point::duration>(std::chrono::duration<double>(since+1)));
        mode=static_cast<int>(d.armor_or_buff);adapter.update(detections,d,image.size());
      }
      double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();times.push_back(ms);
      const auto state=metadata.is_open()?adapter.state():"not_run_without_attitude";
      if(detections.empty())out<<count<<",none,none,0,0,0,"<<ms<<','<<state<<",,,,,,,,\n";
      for(auto armor:detections) {
        ++raw_count;bool accepted=adapter.accept_type(armor,mode);accepted_count+=accepted;
        out<<count<<','<<auto_aim::ARMOR_NAMES.at(armor.name)<<','<<auto_aim::ARMOR_TYPES.at(armor.type)<<','
           <<armor.confidence<<','<<armor.ratio<<','<<accepted<<','<<ms<<','<<state;
        for(const auto &p:armor.points)out<<','<<p.x<<','<<p.y;
        out<<'\n';
      }
      ++count;
    }
    if(times.empty())throw std::runtime_error("No video frames");
    double sum=std::accumulate(times.begin(),times.end(),0.);std::sort(times.begin(),times.end());
    rusage usage{};getrusage(RUSAGE_SELF,&usage);
    std::cout<<"frames="<<count<<" raw_detections="<<raw_count<<" accepted="<<accepted_count
             <<" mean_ms="<<sum/count<<" p95_ms="<<times[std::min(times.size()-1,size_t(times.size()*.95))]
             <<" peak_rss_kib="<<usage.ru_maxrss<<" metadata="<<metadata.is_open()<<std::endl;
    return 0;
  } catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}
}
