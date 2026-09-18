#pragma once
#include "Kinematics.h"
#include "Parser.h"
#include <string>

namespace cnc {

enum class MotionMode { Rapid, Linear, ArcCW, ArcCCW };

struct ModalState {
    MotionMode motion{MotionMode::Rapid};
    bool absolute{true};
    int work_offset{54};
    int plane{17};
    bool coolant{false};
    int spindle_direction{0};
};

class Runtime {
    MachineState state_{};
    ModalState modal_{};
    double feed_{0}, rpm_{0};
    int tool_{0}, d_{0};
    double work_x_{0}, work_y_{0}, work_z_{0};

public:
    bool execute(const Block&, std::string&);
    const MachineState& state() const { return state_; }
    const ModalState& modal() const { return modal_; }
    double feed() const { return feed_; }
    double rpm() const { return rpm_; }
    int tool() const { return tool_; }
    int d() const { return d_; }
    void reset();
};

}
