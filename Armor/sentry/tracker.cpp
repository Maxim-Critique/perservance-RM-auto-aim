// Adapted from TongjiSuperPower/sp_vision_25 b2c7dc5. See LICENSE and SOURCES.md.
#include "tracker.hpp"

#include <algorithm>
#include "math_tools.hpp"

namespace sentry_aim
{
void Tracker::reset()
{
  state_ = State::lost;
  target_.reset();
  last_timestamp_.reset();
  enemy_.reset();
  detect_count_ = lost_count_ = 0;
  observed_ = false;
}

void Tracker::set_target(const Armor & armor, std::chrono::steady_clock::time_point t)
{
  const bool balance = armor.type == ArmorType::big &&
    (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);
  const bool fixed_radius = armor.name == ArmorName::outpost || armor.name == ArmorName::base;
  const int count = balance ? 2 : fixed_radius ? 3 : 4;
  const double radius = armor.name == ArmorName::outpost ? 0.2765 :
                        armor.name == ArmorName::base ? 0.3205 : 0.2;
  Eigen::VectorXd p{{1, 64, 1, 64, 1, 64, 0.4, 100,
                    fixed_radius ? 1e-4 : 1.0, fixed_radius ? 0.0 : 1.0, fixed_radius ? 0.0 : 1.0}};
  target_.emplace(armor, t, radius, count, p);
  state_ = config_.min_detect_count == 1 ? State::tracking : State::detecting;
  detect_count_ = 1;
  lost_count_ = 0;
  observed_ = true;
  ++generation_;
}

const Target * Tracker::track(std::vector<Armor> armors,
  std::chrono::steady_clock::time_point t, Color enemy, cv::Size image_size)
{
  if ((enemy_ && *enemy_ != enemy) ||
      (last_timestamp_ && (t <= *last_timestamp_ ||
       tools::delta_time(t, *last_timestamp_) > config_.max_frame_gap))) reset();
  last_timestamp_ = t;
  enemy_ = enemy;
  observed_ = false;
  armors.erase(std::remove_if(armors.begin(), armors.end(), [&](const Armor & a) {
    return a.color != enemy || a.name == ArmorName::not_armor ||
      !a.xyz_in_world.allFinite() || !a.ypr_in_world.allFinite() || !a.ypd_in_world.allFinite() ||
      a.xyz_in_world.norm() < 0.1 || a.xyz_in_world.norm() > config_.max_distance ||
      !std::isfinite(a.center.x) || !std::isfinite(a.center.y);
  }), armors.end());
  const cv::Point2f center(image_size.width * 0.5f, image_size.height * 0.5f);
  std::stable_sort(armors.begin(), armors.end(), [&](const Armor & a, const Armor & b) {
    if (a.priority != b.priority) return a.priority < b.priority;
    return cv::norm(a.center - center) < cv::norm(b.center - center);
  });

  if (!target_) {
    if (!armors.empty()) set_target(armors.front(), t);
    return target_ ? &*target_ : nullptr;
  }
  // Main-camera priority switching only; A retains ownership of USB search and navigation.
  if (state_ == State::tracking && !armors.empty() && armors.front().priority < target_->priority) {
    set_target(armors.front(), t);
    return &*target_;
  }
  target_->predict(t);
  const Armor * observation = nullptr;
  for (const auto & armor : armors) {
    if (armor.name != target_->name || armor.type != target_->armor_type) continue;
    // B updates the leftmost same-ID observation, preserving the observation/plate association.
    if (!observation || armor.center.x < observation->center.x) observation = &armor;
  }
  if (observation) {
    target_->update(*observation);
    observed_ = true;
    lost_count_ = 0;
    if (state_ == State::temp_lost) state_ = State::tracking;
    else if (state_ == State::detecting && ++detect_count_ >= config_.min_detect_count)
      state_ = State::tracking;
  } else if (state_ == State::detecting) {
    reset();
    return nullptr;
  } else {
    state_ = State::temp_lost;
    const int limit = target_->name == ArmorName::outpost ?
      config_.outpost_max_temp_lost_count : config_.max_temp_lost_count;
    if (++lost_count_ > limit) {
      reset();
      return nullptr;
    }
  }
  if (target_->diverged()) {
    reset();
    return nullptr;
  }
  return &*target_;
}
}  // namespace sentry_aim
