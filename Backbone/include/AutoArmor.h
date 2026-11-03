#pragma once

#include "ArmorSolver.h"
#include "ArmorDetector.h"
#include "sentry/adapter.hpp"
#include "sentry/aimer.hpp"
#include "sentry/shooter.hpp"
#include "sentry/tracker.hpp"

class AutoArmor
{
public:
    explicit AutoArmor(const std::string & config_path = YamlPath, bool realtime = true);
    void Reset();
    bool AutoAim(Mat & img, const Data_Save & ds, Data_Get & dg);
private:
    void ShowArmors(const Armors & armors, Mat & img);
    sentry_aim::Config config_;
    std::unique_ptr<ArmorDetector> find_ptr_;
    std::unique_ptr<Slover> solve_ptr_;
    std::unique_ptr<sentry_aim::Tracker> tracker_;
    std::unique_ptr<sentry_aim::Aimer> aimer_;
    std::unique_ptr<sentry_aim::Shooter> shooter_;
    unsigned generation_ = 0;
    int last_mode_ = -1;
    int last_color_ = -1;
    bool show_ = false;
    bool realtime_ = true;
    std::string model_dir_;
};
