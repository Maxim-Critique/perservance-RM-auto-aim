#include "tongji_adapter.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "AutoArmor.h"
#include "State.h"
#include "player.hpp"
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <stdexcept>
#include <limits>
#include <cstring>
#include "tools/math_tools.hpp"

void check(bool x, const char *message) { if (!x) throw std::runtime_error(message); }
Data_Save frame(double seconds = 1) {
  Data_Save d{}; d.timestamp = time_point(std::chrono::duration_cast<time_point::duration>(std::chrono::duration<double>(seconds)));
  d.color = 0; d.armor_or_buff = 0; return d;
}
auto_aim::Armor observed(fusion::TongjiAdapter &a, auto_aim::ArmorType type = auto_aim::small,
                         auto_aim::ArmorName name = auto_aim::three, Eigen::Vector3d xyz = {4,.2,.1}) {
  auto points = a.solver().reproject_armor(xyz, 0, type, name);
  auto_aim::Armor armor(0, .99f, cv::Rect(0,0,100,100), points);
  armor.name = name; armor.type = type; armor.color = auto_aim::blue; armor.priority = auto_aim::first;
  return armor;
}
int main(int argc, char **argv) {
  try {
    check(argc == 4, "arguments"); const std::string which = argv[1];
    fusion::TongjiAdapter a(argv[2], argv[3], which == "model");
    if (which == "calibration") {
      const auto cfg=YAML::LoadFile(argv[2]);
      check(cfg["Transform"]["t_camera2gimbal"][0].as<double>() == -.05, "original translation");
      for(double yaw : {0.,.3,-2.}) for(double pitch : {0.,.4,-.6}) for(double roll : {0.,.2}) {
        auto d=frame();d.camera_yaw_angle=yaw;d.camera_pitch_angle=pitch;d.camera_roll_angle=roll;
        const double x=roll,y=-pitch,z=yaw;
        Eigen::Matrix3d R;
        R << cos(y)*cos(z), cos(z)*sin(x)*sin(y)-cos(x)*sin(z), cos(x)*cos(z)*sin(y)+sin(x)*sin(z),
             cos(y)*sin(z), cos(x)*cos(z)+sin(x)*sin(y)*sin(z), -cos(z)*sin(x)+cos(x)*sin(y)*sin(z),
             -sin(y), cos(y)*sin(x), cos(x)*cos(y);
        check((R-fusion::TongjiAdapter::original_rotation(d)).norm()<1e-12,"original rotation formula");
      }
      cv::Mat rgb(1,1,CV_8UC3,cv::Scalar(255,0,0)); auto bgr=a.to_bgr(rgb,true);
      check(bgr.at<cv::Vec3b>(0,0)==cv::Vec3b(0,0,255),"RGB to BGR");
    } else if (which == "type") {
      auto armor=observed(a); armor.points={{0,0},{150,0},{150,20},{0,20}};
      check(armor.ratio>0,"ratio initialised");
      check(a.accept_type(armor,0) && armor.type==auto_aim::small,"oblique small must not become BIG by ratio");
      armor.name=auto_aim::five;armor.type=auto_aim::big;
      check(a.accept_type(armor,0) && armor.type==auto_aim::big,"BIG retained");
      armor.name=auto_aim::one;armor.type=auto_aim::small;
      check(a.accept_type(armor,0) && armor.type==auto_aim::big,"hero identity repair");
      auto hero=observed(a,auto_aim::big,auto_aim::one);hero.type=auto_aim::small;
      check(a.resolve_type(hero,0) && hero.type==auto_aim::big &&
            (hero.xyz_in_world-Eigen::Vector3d(4,.2,.1)).norm()<.003,"hero type and solved dimensions agree");
      armor.name=auto_aim::three;armor.type=auto_aim::big;
      check(!a.accept_type(armor,0),"original A LARGE/3 filter");
      armor=observed(a,auto_aim::small,auto_aim::outpost);
      check(!a.accept_type(armor,0) && a.accept_type(armor,3),"outpost mode rule");
      armor.name=auto_aim::base;check(!a.accept_type(armor,0),"base excluded");
      for(double pitch : {0.,-.6,-.9}) {
        auto d=frame(); d.camera_pitch_angle=pitch;
        a.solver().set_original_car_rotation(a.original_rotation(d));
        const Eigen::Vector3d world=a.original_rotation(d)*Eigen::Vector3d(4,0,0);
        auto oblique=observed(a,auto_aim::small,auto_aim::five,world);
        // Start with a wrong BIG hypothesis, not a forced-SMALL detector.
        oblique.type=auto_aim::big;
        check(a.resolve_type(oblique,0) && oblique.type==auto_aim::small,"perspective hypothesis recovers SMALL");
        auto large=observed(a,auto_aim::big,auto_aim::five,world);
        check(a.resolve_type(large,0) && large.type==auto_aim::big,"perspective retains real BIG");
      }
    } else if (which == "pose") {
      const auto cfg=YAML::LoadFile(argv[2]);
      const auto k=cfg["Camera"]["camera_matrix"].as<std::vector<double>>();
      const auto d=cfg["Camera"]["distort_coeffs"].as<std::vector<double>>();
      cv::Mat K(3,3,CV_64F,const_cast<double*>(k.data())),D(1,5,CV_64F,const_cast<double*>(d.data()));
      Eigen::Matrix3d R;R<<0,0,1,-1,0,0,0,-1,0;
      const Eigen::Vector3d trans(-.05,-.05,0),world(4,.2,.1);
      for(auto type : {auto_aim::small,auto_aim::big}) for(double yaw : {-.4,0.,.5}) {
        const double width=type==auto_aim::big?.230:.130;
        std::vector<cv::Point3f> object{{0,float(width/2),.025f},{0,float(-width/2),.025f},
                                      {0,float(-width/2),-.025f},{0,float(width/2),-.025f}};
        const Eigen::Matrix3d R_aw=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(15*CV_PI/180,Eigen::Vector3d::UnitY())).toRotationMatrix();
        cv::Mat rot;cv::eigen2cv((R.transpose()*R_aw).eval(),rot);cv::Vec3d rv;cv::Rodrigues(rot,rv);
        Eigen::Vector3d pos=R.transpose()*(world-trans);std::vector<cv::Point2f> points;
        cv::projectPoints(object,rv,cv::Vec3d(pos[0],pos[1],pos[2]),K,D,points);
        auto armor=observed(a,type);armor.points=points;a.solver().solve(armor);
        check((armor.xyz_in_world-world).norm()<.003,"PnP metre/origin/dimensions roundtrip");
        check(std::abs(tongji_tools::limit_rad(armor.ypr_in_world[0]-yaw))<.03,"yaw rad roundtrip");
      }
    } else if (which == "tracker") {
      std::shared_ptr<KinematicModel> model;
      for(int i=0;i<25;i++) model=a.update({observed(a)},frame(1+i*.01),{1440,1080});
      check(model && a.state()=="tracking","stationary target acquisition");
      check(model->number==4 && std::abs(model->radius[0]-.25)<.02,"A normal radius and armor count");
      for(const auto &armor:model->GetArmors(.08))check(armor.abs_pose.allFinite(),"finite predicted pose");
      for(int i=0;i<25;i++) a.update({},frame(1.25+i*.01),{1440,1080});
      check(a.state()=="lost","target lost");
      for(int i=0;i<25;i++)model=a.update({observed(a)},frame(1.5+i*.01),{1440,1080});
      check(model && a.state()=="tracking","reacquisition");
      a.reset();
      for(int i=0;i<25;i++)model=a.update({observed(a,auto_aim::big,auto_aim::five)},frame(2+i*.01),{1440,1080});
      check(model && model->number==2,"two-armor balance association bounds");
    } else if (which == "outpost") {
      auto cfg=YAML::LoadFile(argv[3]);cfg["enemy_color"]="blue";
      auto_aim::Tracker tracker(cfg,a.solver());
      std::list<auto_aim::Armor> armors{observed(a,auto_aim::small,auto_aim::outpost)};
      auto targets=tracker.track(armors,frame().timestamp);
      check(!targets.empty() && std::abs(targets.front().ekf_x()[8]-.225)<1e-10,"A outpost radius initialisation");
      check(targets.front().armor_xyza_list().size()==3,"outpost three-armor model");
      auto d=frame();d.armor_or_buff=3;
      for(int i=0;i<25;i++){d.timestamp=frame(1+i*.01).timestamp;
        check(!a.update({observed(a,auto_aim::small,auto_aim::outpost)},d,{1440,1080}),"stationary outpost control gated");}
      a.reset();std::shared_ptr<KinematicModel> model;
      for(int i=0;i<40;i++) {
        const double yaw=2.51*i*.01;
        auto armor=observed(a,auto_aim::small,auto_aim::outpost,
          {4.225-.225*std::cos(yaw),-.225*std::sin(yaw),.1});
        armor.points=a.solver().reproject_armor({4.225-.225*std::cos(yaw),-.225*std::sin(yaw),.1},yaw,auto_aim::small,auto_aim::outpost);
        d.timestamp=frame(2+i*.01).timestamp;model=a.update({armor},d,{1440,1080});
      }
      check(model && model->number==3 && std::abs(model->palstance)>=2 && std::abs(model->palstance)<=3,"rotating outpost gate uses B omega index 7");
    } else if (which == "planner") {
      std::shared_ptr<KinematicModel> model;
      for(int i=0;i<25;i++)model=a.update({observed(a)},frame(1+i*.01),{1440,1080});
      check(bool(model),"planner target snapshot");
      Planner planner(argv[2]);const auto angle=planner.TrajectoryPlanning(model);
      check(std::isfinite(angle.manage_yaw) && std::isfinite(angle.target_pitch),"A planner accepts B model");
      check(std::abs(angle.manage_yaw-std::atan2(.2,4.))<.03,"yaw in rad");
      check(angle.target_pitch>0 && angle.target_pitch<.2,"22m/s ballistic pitch in rad");
    } else if (which == "ekf") {
      Eigen::Vector2d x(0,1);Eigen::Matrix2d P=Eigen::Matrix2d::Identity();
      tongji_tools::ExtendedKalmanFilter ekf(x,P);
      Eigen::Matrix2d F;F<<1,.01,0,1;
      Eigen::Matrix<double,1,2> H;H<<1,0;
      for(int i=0;i<200;i++) {
        ekf.predict(F,.0001*P);
        Eigen::VectorXd z(1);z<<.01*(i+1);
        ekf.update(z,H,Eigen::Matrix<double,1,1>::Constant(.01));
        check(ekf.x.allFinite() && ekf.P.allFinite(),"finite EKF");
        check((ekf.P-ekf.P.transpose()).norm()<1e-10,"covariance symmetry");
      }
      check(std::abs(ekf.x[0]-2)<.02,"EKF constant-velocity estimate");
    } else if(which == "protocol") {
      Data_Get dg;dg.Nav2Control(-123,456);unsigned char nav[4];std::memcpy(nav,dg.raw_data+10,4);
      dg.Vision2Control(-1234567,7654321,7,1,0);
      const unsigned char expected[21]={0xAA,0xBB,0xFF,0xED,0x29,0x79,0,0x74,0xCB,0xB1,0xFF,0x85,1,0xC8,0,0,0,7,1,0,0xBB};
      check(std::memcmp(expected,dg.raw_data,21)==0,"AA BB 21-byte golden packet");
      check(std::memcmp(nav,dg.raw_data+10,4)==0,"vision preserves navigation bytes");
      dg.Vision2Control();check(dg.raw_data[19]==0 && dg.raw_data[18]==0,"clear vision output");
    } else if(which == "time") {
      for(int i=0;i<10;i++)a.update({observed(a)},frame(1+i*.01),{1440,1080});
      check(a.state()=="tracking","setup time test");
      auto result=a.update({observed(a)},frame(1.05),{1440,1080});
      check(!result && a.state()=="detecting","backward timestamp resets");
      result=a.update({observed(a)},frame(3600),{1440,1080});
      check(!result && a.state()=="detecting","long interval resets");
      auto invalid=frame();invalid.camera_pitch_angle=std::numeric_limits<double>::quiet_NaN();
      check(!a.update({observed(a)},invalid,{1440,1080}) && a.state()=="lost","NaN attitude rejected");
    } else if(which == "empty") {
      check(!a.process({},frame(),false) && a.state()=="lost","empty frame");
      auto armor=observed(a);armor.points.clear();
      check(!a.update({armor},frame(1.01),{1440,1080}),"missing corners");
      armor=observed(a);armor.points[0].x=std::numeric_limits<float>::quiet_NaN();
      check(!a.update({armor},frame(1.02),{1440,1080}),"NaN corner");
      auto d=frame(1.03);d.color=8;check(!a.update({},d,{1440,1080}),"invalid color");
    } else if(which == "player") {
      const auto root=std::filesystem::temp_directory_path()/("fusion-player-fixture-"+std::to_string(getpid()));
      std::filesystem::create_directories(root);const auto prefix=(root/"sample").string();
      { cv::VideoWriter out(prefix+".avi",cv::VideoWriter::fourcc('M','J','P','G'),30,{64,48});
        check(out.isOpened(),"fixture video writer");
        for(int i=0;i<3;i++)out.write(cv::Mat(48,64,CV_8UC3,i==0?cv::Scalar(240,0,0):cv::Scalar(i*60,i*60,i*60))); }
      { std::ofstream text(prefix+".txt");text<<"0 0 0 0 0 0 0\n3600 0.1 0 0 0 0 0\nbroken"; }
      tools::Player player(prefix);cv::Mat image;Data_Save first{},later{};
      check(player.read(image,first),"read first");check(player.read(image,later),"read hour timestamp");
      check(std::abs(std::chrono::duration<double>(later.timestamp-first.timestamp).count()-3600)<1e-6,"no microsecond overflow");
      check(!player.read(image,later) && image.empty(),"bad metadata rejected");
      player.seek(1);check(player.read(image,later) && std::abs(cv::mean(image)[0]-60)<3,"forward seek image metadata alignment");
      player.seek(0);check(player.read(image,later) && std::abs(cv::mean(image)[0]-240)<5,"backward seek");
      const auto bgr=fusion::TongjiAdapter::to_bgr(image,true);
      check(cv::mean(bgr)[2]>230 && cv::mean(bgr)[0]<5,"original Recorder RGB data restored for YOLO BGR");
      bool threw=false;try{player.seek(-1);}catch(const std::out_of_range&){threw=true;}check(threw,"negative seek rejected");
    } else if(which == "hold") {
      AimStateMachine state;Data_Get dg;dg.Vision2Control(12000,34000,3,1,1);
      state.Step(frame().timestamp,dg);dg.Vision2Control();dg.Nav2Control(123,456);
      state.Step(frame(1.01).timestamp,dg);
      check(dg.raw_data[19]==0 && dg.raw_data[18]==1,"hold clears fire and keeps angle");
      check(dg.raw_data[10]==0 && dg.raw_data[11]==123 && dg.raw_data[12]==1 && dg.raw_data[13]==200,"hold preserves current nav");
      state.Reset();dg.Vision2Control();state.Step(frame(1.02).timestamp,dg);
      check(dg.raw_data[18]==0 && dg.raw_data[19]==0,"reset clears hold");
    } else if(which == "model") {
      cv::Mat blank(1080,1440,CV_8UC3,cv::Scalar(0));
      for(int i=0;i<3;i++) { auto detections=a.detect_only(blank,false);check(detections.empty(),"blank detections"); }
      check(!a.process(blank,frame(),false),"model pipeline blank");
    } else throw std::runtime_error("unknown case");
    std::cout<<"PASS "<<which<<std::endl;return 0;
  } catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<std::endl;return 1;}
}
