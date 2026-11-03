#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <map>
#include <opencv2/calib3d.hpp>

#include "sentry/adapter.hpp"
#include "sentry/aimer.hpp"
#include "sentry/shooter.hpp"
#include "sentry/tracker.hpp"
#include "ArmorSolver.h"
#include "State.h"
#include "trajectory.hpp"
#include "AutoArmor.h"

namespace sa = sentry_aim;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char * message)
{
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
Time at(double t) { return Time{} + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(t)); }
sa::Armor observation(sa::ArmorName name = sa::ArmorName::three, double x = 4, double y = 0, double angle = 0)
{
  sa::Armor a;
  a.name = name;
  a.priority = name == sa::ArmorName::one ? 0 : 1;
  a.center = {640, 360};
  a.xyz_in_world = {x, y, 0.5};
  a.ypr_in_world = {angle, 0, 0};
  a.ypd_in_world = tools::xyz2ypd(a.xyz_in_world);
  return a;
}
sa::Target target(const sa::Armor & a = observation(), int count = 4)
{
  return sa::Target(a, at(0), 0.2, count, Eigen::VectorXd::Ones(11));
}
int32_t read_angle(const Data_Get & d, int offset)
{
  uint32_t n = 0;
  for (int i = 0; i < 4; ++i) n = (n << 8) | d.raw_data[offset+i];
  int32_t result;
  std::memcpy(&result, &n, sizeof(result));
  return result;
}

void bridge()
{
  require(sa::color_from_a(0) == sa::Color::blue, "A blue conversion");
  require(sa::color_from_a(1) == sa::Color::red, "A red conversion");
  require(!sa::color_from_a(2), "unknown color rejected");
  const std::vector<std::string> names{"1","2","3","4","5","outpost","guard","base"};
  for (std::size_t i = 0; i < names.size(); ++i)
    require(sa::id_for_a(sa::name_from_a(names[i])) == i+1, "A ID mapping");
  require(sa::name_from_a("negative") == sa::ArmorName::not_armor, "unknown ID rejected");
  ::Armor a;
  a.type = ::ArmorType::SMALL;
  a.number = "guard";
  a.center = {640,360};
  a.abs_pose = {4,0,0.5};
  a.yaw = CV_PI;
  auto converted = sa::observation_from_a(a, 0, sa::Config{});
  require(converted.has_value(), "solved observation accepted");
  near(converted->ypr_in_world[0], 0, 1e-10, "opposite plate normal conventions");
  auto t = target(*converted);
  near(t.ekf_x()[0], 4.2, 1e-10, "rotation center must be behind near plate");
  near((t.armor_xyza_list()[0].head<3>() - a.abs_pose).norm(), 0, 1e-10, "plate round trip");
}

void wire()
{
  Data_Get d;
  d.Nav2Control(-1234, 2345);
  auto nav = d;
  sa::write_command({true, -CV_PI/2, CV_PI/4}, sa::ArmorName::sentry, true, d);
  require(d.size == 21 && d.raw_data[0] == 0xaa && d.raw_data[1] == 0xbb && d.raw_data[20] == 0xbb, "21-byte frame markers");
  near(read_angle(d, 2), -900000, 1, "big endian negative yaw, degree times 1e4");
  near(read_angle(d, 6), 450000, 1, "big endian positive pitch");
  require(d.raw_data[17] == 7 && d.raw_data[18] == 1 && d.raw_data[19] == 1, "A flags and sentry ID");
  require(std::memcmp(d.raw_data+10, nav.raw_data+10, 7) == 0, "vision preserves navigation");
  sa::write_command({true, 0, std::numeric_limits<double>::infinity()}, sa::ArmorName::one, true, d);
  require(!d.raw_data[18] && !d.raw_data[19], "nonfinite command cannot escape");
  sa::write_command({true, 0, 0}, sa::ArmorName::not_armor, true, d);
  require(!d.raw_data[18] && !d.raw_data[19], "invalid identity cannot escape");
}

void model()
{
  for (int count : {2,3,4}) {
    auto t = target(observation(), count);
    require(t.ekf_x().size() == 11, "11 state dimensions");
    auto plates = t.armor_xyza_list();
    require(plates.size() == static_cast<std::size_t>(count), "plate count");
    near((plates[0].head<3>() - observation().xyz_in_world).norm(), 0, 1e-9, "initial observed plate");
    t.predict(at(0.1));
    require(!t.diverged(), "stationary prediction remains finite");
    for (const auto & plate : t.armor_xyza_list())
      near(std::hypot(plate[0]-4.2, plate[1]), 0.2, 1e-9, "plate radius");
  }
}

void association()
{
  for (int count : {2,3,4}) {
    auto t = target(observation(), count);
    for (int i = 1; i <= 40; ++i) {
      t.predict(at(i * 0.01));
      const auto plate = t.armor_xyza_list()[i % count];
      auto a = observation(sa::ArmorName::three, plate[0], plate[1], plate[3]);
      a.xyz_in_world[2] = plate[2];
      a.ypd_in_world = tools::xyz2ypd(a.xyz_in_world);
      t.update(a);
      require(!t.diverged(), "plate association stays finite for 2/3/4 plates");
      require(t.last_id >= 0 && t.last_id < count, "association index in bounds");
    }
  }
  auto t = target(observation(sa::ArmorName::three, 4, 0, CV_PI - 1e-4));
  auto a = observation(sa::ArmorName::three, 4, 0, -CV_PI + 1e-4);
  t.predict(at(0.01));
  t.update(a);
  require(!t.diverged(), "orientation association wraps at pi");
}

void tracker()
{
  sa::Config c;
  c.min_detect_count = 2;
  c.max_temp_lost_count = 2;
  sa::Tracker tr(c);
  auto a = observation();
  require(tr.track({a}, at(0), sa::Color::blue, {1280,720}), "target acquired");
  require(tr.state() == sa::Tracker::State::detecting, "confirmation phase");
  require(tr.track({a}, at(.01), sa::Color::blue, {1280,720}), "target confirmed");
  require(tr.state() == sa::Tracker::State::tracking && tr.observed(), "tracking phase");
  require(tr.track({}, at(.02), sa::Color::blue, {1280,720}), "predict through first occlusion");
  require(tr.state() == sa::Tracker::State::temp_lost && !tr.observed(), "no fabricated observation");
  require(tr.track({a}, at(.03), sa::Color::blue, {1280,720}), "recover from occlusion");
  require(tr.state() == sa::Tracker::State::tracking, "recovery state");
  require(tr.track({}, at(.04), sa::Color::blue, {1280,720}), "lost frame one");
  require(tr.track({}, at(.05), sa::Color::blue, {1280,720}), "lost frame two");
  require(!tr.track({}, at(.06), sa::Color::blue, {1280,720}), "loss expiry");
  require(tr.state() == sa::Tracker::State::lost, "USB search may resume");
  tr.track({a}, at(.07), sa::Color::blue, {1280,720});
  require(!tr.track({}, at(.08), sa::Color::blue, {1280,720}), "unconfirmed detection expires immediately");
}

void timing()
{
  sa::Config c;
  c.min_detect_count = 1;
  sa::Tracker tr(c);
  auto a = observation();
  tr.track({a}, at(1), sa::Color::blue, {1280,720});
  require(!tr.track({}, at(1), sa::Color::blue, {1280,720}), "duplicate timestamp resets");
  tr.track({a}, at(2), sa::Color::blue, {1280,720});
  require(!tr.track({}, at(1), sa::Color::blue, {1280,720}), "time reversal resets");
  tr.track({a}, at(2), sa::Color::blue, {1280,720});
  require(!tr.track({}, at(2.2), sa::Color::blue, {1280,720}), "camera interruption resets");
  tr.track({a}, at(3), sa::Color::blue, {1280,720});
  require(!tr.track({a}, at(3.01), sa::Color::red, {1280,720}), "color switch resets and filters");
  a.xyz_in_world[0] = std::numeric_limits<double>::quiet_NaN();
  require(!tr.track({a}, at(4), sa::Color::blue, {1280,720}), "nonfinite observation rejected");
}

void priority()
{
  sa::Config c;
  c.min_detect_count = 1;
  sa::Tracker tr(c);
  auto a = observation();
  a.center = {720,540};
  auto b = observation(sa::ArmorName::three, 3);
  b.center = {1000,500};
  const auto * selected = tr.track({a,b}, at(0), sa::Color::blue, {2000,1000});
  near(selected->armor_xyza_list()[0][0], 3, 1e-9, "use actual image center");
  const auto generation = tr.generation();
  auto hero = observation(sa::ArmorName::one);
  selected = tr.track({a,hero}, at(.01), sa::Color::blue, {2000,1000});
  require(selected->name == sa::ArmorName::one && tr.generation() != generation, "priority replacement resets generation");
}

void shooter()
{
  auto t = target();
  sa::Shooter s(sa::Config{});
  sa::AimPoint p{true, {4,0,.5,0}};
  sa::Command cmd{true,0,.1};
  Eigen::Vector3d gimbal{0,.1,0};
  require(!s.shoot(cmd,p,&t,gimbal,true,true), "no fire on first command");
  require(s.shoot(cmd,p,&t,gimbal,true,true), "stable aligned command can fire");
  cmd.yaw = .3;
  gimbal[0] = .3;
  require(!s.shoot(cmd,p,&t,gimbal,true,true), "command yaw jump blocks fire before history update");
  require(s.shoot(cmd,p,&t,gimbal,true,true), "recover after stable jump");
  gimbal[1] = .5;
  require(!s.shoot(cmd,p,&t,gimbal,true,true), "pitch must align too");
  gimbal[1] = .1;
  require(!s.shoot(cmd,p,&t,gimbal,true,false), "predicted observation never fires");
  require(!s.shoot(cmd,p,&t,gimbal,true,true), "occlusion clears history");
  require(!s.shoot(cmd,p,&t,gimbal,false,true), "unconfirmed track never fires");
  cmd.yaw = CV_PI-.001;
  s.shoot(cmd,p,&t,{cmd.yaw,.1,0},true,true);
  cmd.yaw = -CV_PI+.001;
  require(s.shoot(cmd,p,&t,{cmd.yaw,.1,0},true,true), "command delta wraps at pi");
  sa::Config c; c.auto_fire = false;
  sa::Shooter disabled(c);
  disabled.shoot(cmd,p,&t,gimbal,true,true);
  require(!disabled.shoot(cmd,p,&t,gimbal,true,true), "auto_fire false honored");
}

void static_aim()
{
  sa::Aimer aimer(sa::Config{});
  auto t = target();
  const auto before = t.ekf_x();
  auto cmd = aimer.aim(t, at(0), 21.5);
  require(cmd.control && aimer.debug_aim_point.valid, "stationary solution");
  near(cmd.yaw, 0, 1e-10, "stationary yaw");
  tools::Trajectory expected(21.5, 4, .5);
  near(cmd.pitch, expected.pitch, 1e-9, "stationary ballistic pitch");
  near((t.ekf_x()-before).norm(), 0, 1e-12, "aiming does not mutate live filter");
  require(!aimer.aim(t, at(0), 0).control && !aimer.debug_aim_point.valid, "invalid speed clears aim point");
  require(!aimer.aim(t, at(0), 21.5, .6).control, "stale frame cannot produce command");
}

void moving_aim()
{
  auto t = target();
  for (int i = 1; i <= 150; ++i) {
    t.predict(at(i*.01));
    t.update(observation(sa::ArmorName::three, 4, i*.01));
  }
  sa::Aimer aimer(sa::Config{});
  const auto cmd = aimer.aim(t, at(1.5), 21.5);
  require(cmd.control, "moving target ballistic iteration converges");
  require(cmd.yaw > std::atan2(1.5,4), "lateral target receives lead");
  require(aimer.debug_aim_point.xyza[1] > 1.5, "aim point predicts forward in time");
}

void trajectory()
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto & values : std::vector<Eigen::Vector3d>{{0,4,0},{-1,4,0},{21.5,0,0},
        {21.5,-1,0},{nan,4,0},{21.5,4,nan},{21.5,1000,1000}}) {
    tools::Trajectory t(values[0], values[1], values[2]);
    require(t.unsolvable && std::isfinite(t.fly_time) && std::isfinite(t.pitch), "invalid ballistic inputs produce defined invalid result");
  }
  for (double height : {-1.,0.,1.}) {
    tools::Trajectory t(21.5, 4, height);
    require(!t.unsolvable, "normal ballistic input");
    near(21.5*std::cos(t.pitch)*t.fly_time, 4, 1e-10, "horizontal ballistic range");
    near(21.5*std::sin(t.pitch)*t.fly_time - .5*9.7833*t.fly_time*t.fly_time, height, 1e-10, "vertical ballistic range");
  }
}

void hold()
{
  AimStateMachine state;
  Data_Get d;
  d.Vision2Control(1234,-5678,3,1,1);
  state.Step(at(0),d);
  d.Nav2Control(42,-13);
  const auto nav = d;
  d.Vision2Control();
  state.Step(at(.01),d);
  require(read_angle(d,2) == 1234 && read_angle(d,6) == -5678, "hold keeps angles");
  require(d.raw_data[18] == 1 && !d.raw_data[19], "hold never retains fire");
  require(std::memcmp(nav.raw_data+10,d.raw_data+10,7) == 0, "hold keeps current navigation");
  d.Vision2Control();
  state.Step(at(.511),d);
  require(!d.raw_data[18] && !d.raw_data[19] && state.UsbActive(), "hold expires and USB resumes");
  d.Vision2Control(22,33,0,2,0);
  state.Step(at(.52),d);
  d.Vision2Control();
  state.Step(at(.53),d);
  require(d.raw_data[18] == 2 && !d.raw_data[19], "USB hold identity preserved");
  state.Reset();
  d.Vision2Control();
  state.Step(at(.54),d);
  require(!d.raw_data[18] && state.UsbActive(), "mode change clears stale hold");
}

void config()
{
  const auto c = sa::Config::from_yaml(YAML::LoadFile(AIM_CONFIG_PATH));
  near(c.yaw_offset, .4, 1e-10, "keep A yaw compensation");
  near(c.pitch_offset, .8, 1e-10, "keep A pitch compensation");
  near(c.predict_time, .07, 1e-10, "keep A prediction delay");
  bool rejected = false;
  try { sa::Config::from_yaml(YAML::Load("Sentry: {min_detect_count: 0}")); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "invalid tracking count rejected");
  rejected = false;
  try { sa::Config::from_yaml(YAML::Load("Sentry: {priority_ids: [1,1,3,4,5,6,7,8]}")); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected, "duplicate priority IDs rejected");
}

void solver()
{
  const auto yaml = YAML::LoadFile(AIM_CONFIG_PATH);
  const auto intrinsics = yaml["Camera"]["camera_matrix"].as<std::vector<double>>();
  const auto distortion = yaml["Camera"]["distort_coeffs"].as<std::vector<double>>();
  const cv::Mat k(3,3,CV_64F,const_cast<double *>(intrinsics.data()));
  const cv::Mat dist(1,5,CV_64F,const_cast<double *>(distortion.data()));
  // A order: bottom-left, top-left, top-right, bottom-right. Normal points toward the camera.
  const std::vector<cv::Point3f> object{{-.065f,-.025f,0},{-.065f,.025f,0},
                                      {.065f,.025f,0},{.065f,-.025f,0}};
  ::Armor a;
  a.type = ::ArmorType::SMALL; a.number = "3"; a.center = {700,525};
  cv::projectPoints(object, cv::Vec3d(165*CV_PI/180,0,0), cv::Vec3d(0,0,4), k, dist, a.points);
  Slover solver(AIM_CONFIG_PATH);
  Data_Save ds{}; ds.camera_yaw_angle = CV_PI/4;
  solver.UpdateStatus(ds);
  require(solver.Solve(a), "A solver accepts A corner order");
  const Eigen::Vector3d expected = Eigen::AngleAxisd(CV_PI/4,Eigen::Vector3d::UnitZ()) * Eigen::Vector3d(3.95,-.05,0);
  near((a.abs_pose-expected).norm(), 0, .002, "A calibration and yaw attitude retained");
  auto b = sa::observation_from_a(a,0,sa::Config{});
  near(tools::limit_rad(b->ypr_in_world[0]-CV_PI/4), 0, .02, "world plate orientation adapted");
  a.points.resize(3);
  require(!solver.Solve(a), "bad corner count rejected");
  a.points.assign(4,cv::Point2f(100,100));
  require(!solver.Solve(a), "degenerate PnP rejected");
}

void blank_pipeline()
{
  AutoArmor pipeline(AIM_CONFIG_PATH, false);
  cv::Mat frame = cv::Mat::zeros(1080,1440,CV_8UC3);
  Data_Save ds{};
  ds.color = ENEMY_BLUE;
  ds.armor_or_buff = 0;
  Data_Get output;
  for (int mode : {0,3,0}) {
    pipeline.Reset();
    ds.armor_or_buff = mode;
    output.Vision2Control(10,20,1,1,1);
    require(!pipeline.AutoAim(frame,ds,output), "blank image has no target");
    require(!output.raw_data[18] && !output.raw_data[19], "empty detector output clears command");
  }
  ds.color = 9;
  require(!pipeline.AutoAim(frame,ds,output), "invalid color returns empty command");
  cv::Mat empty;
  require(!pipeline.AutoAim(empty,ds,output), "empty image returns empty command");
}

int main(int argc, char ** argv)
{
  const std::map<std::string,std::function<void()>> checks{{"bridge",bridge},{"wire",wire},
    {"model",model},{"association",association},{"tracker",tracker},{"timing",timing},
    {"priority",priority},{"shooter",shooter},{"static_aim",static_aim},{"moving_aim",moving_aim},
    {"trajectory",trajectory},{"hold",hold},{"config",config},{"solver",solver},
    {"blank_pipeline",blank_pipeline}};
  try {
    for (const auto & [name,check] : checks) {
      if (argc > 1 && name != argv[1]) continue;
      check();
      std::cout << "PASS " << name << '\n';
    }
  } catch (const std::exception & e) {
    std::cerr << "FAIL " << e.what() << '\n';
    return 1;
  }
  return 0;
}
