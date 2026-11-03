#ifndef AIM_STATE_MACHINE_H
#define AIM_STATE_MACHINE_H

#include <chrono>
#include "Base.h"

class AimStateMachine {
public:
    AimStateMachine() : mode_(Mode::Lost) { last_dg_.Vision2Control(); }

    static AimStateMachine& Instance() {
        static AimStateMachine instance;
        return instance;
    }

    void Reset() {
        mode_ = Mode::Lost;
        hold_deadline_ = {};
        last_dg_ = Data_Get{};
    }

    void Step(const time_point& now, Data_Get& dg) {
        const uint8_t found = dg.raw_data[18];
        UpdateMode(now, found);
        if(mode_ == Mode::MainTrack || mode_ == Mode::UsbSearch) last_dg_ = dg;
        else if (mode_ == Mode::HoldMain || mode_ == Mode::HoldUsb) {
            // Hold only the vision angles and identity; keep the current navigation fields.
            std::memcpy(dg.raw_data + 2, last_dg_.raw_data + 2, 8);
            dg.raw_data[17] = last_dg_.raw_data[17];
            dg.raw_data[18] = last_dg_.raw_data[18];
            dg.raw_data[19] = 0;
        }
    }

    bool UsbActive() const {
        return mode_ == Mode::UsbSearch || mode_ == Mode::HoldUsb || mode_ == Mode::Lost;
    }

    bool NeedPerceptron(const Data_Get& dg) const {
        return !dg.raw_data[18] && UsbActive();
    }

    const char* ModeName() const {
        switch (mode_) {
            case Mode::MainTrack: return "MainTrack";
            case Mode::HoldMain: return "HoldMain";
            case Mode::UsbSearch: return "UsbSearch";
            case Mode::HoldUsb: return "HoldUsb";
            case Mode::Lost: return "Lost";
        }
        return "Unknown";
    }

private:
    enum class Mode { MainTrack, HoldMain, UsbSearch, HoldUsb, Lost };
    Mode mode_;
    time_point hold_deadline_{};
    Data_Get last_dg_{};

    void UpdateMode(const time_point& now, uint8_t found) {
        if (found == 1) { mode_ = Mode::MainTrack; return; }
        if (mode_ == Mode::HoldMain) {
            if (now >= hold_deadline_) mode_ = Mode::Lost;
            return;
        }
        if (found == 2 && mode_ != Mode::HoldUsb) { mode_ = Mode::UsbSearch; return; }
        switch (mode_) {
            case Mode::MainTrack: EnterHoldMain(now); break;
            case Mode::UsbSearch: EnterHoldUsb(now); break;
            case Mode::HoldUsb: if (now >= hold_deadline_) mode_ = Mode::Lost; break;
            case Mode::Lost: break;
            case Mode::HoldMain: break;
        }
    }

    void EnterHoldMain(const time_point& now) { mode_ = Mode::HoldMain; hold_deadline_ = now + std::chrono::milliseconds(500); }
    void EnterHoldUsb(const time_point& now) { mode_ = Mode::HoldUsb; hold_deadline_ = now + std::chrono::milliseconds(500); }

};

#endif
