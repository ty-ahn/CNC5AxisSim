#include "Runtime.h"
#include <cmath>

namespace cnc {

static int code(const Word& w) {
    return static_cast<int>(std::llround(w.value));
}

bool Runtime::execute(const Block& b, std::string& error) {
    auto next = state_;
    auto next_modal = modal_;
    double next_feed = feed_, next_rpm = rpm_;
    int next_tool = tool_, next_d = d_;

    // Siemens-style modal words are applied in block order.
    for (const auto& w : b.words) {
        switch (w.letter) {
        case 'G':
            switch (code(w)) {
            case 0:  next_modal.motion = MotionMode::Rapid; break;
            case 1:  next_modal.motion = MotionMode::Linear; break;
            case 2:  next_modal.motion = MotionMode::ArcCW; break;
            case 3:  next_modal.motion = MotionMode::ArcCCW; break;
            case 17: next_modal.plane = 17; break;
            case 18: next_modal.plane = 18; break;
            case 19: next_modal.plane = 19; break;
            case 54: next_modal.work_offset = 54; break;
            case 55: next_modal.work_offset = 55; break;
            case 90: next_modal.absolute = true; break;
            case 91: next_modal.absolute = false; break;
            case 94: /* feed per minute */ break;
            case 95: /* feed per revolution */ break;
            default: break;
            }
            break;

        case 'X':
            next.X = next_modal.absolute ? w.value : next.X + w.value;
            break;
        case 'Y':
            next.Y = next_modal.absolute ? w.value : next.Y + w.value;
            break;
        case 'Z':
            next.Z = next_modal.absolute ? w.value : next.Z + w.value;
            break;
        case 'A':
            next.A = next_modal.absolute ? w.value : next.A + w.value;
            break;
        case 'C':
            next.C = next_modal.absolute ? w.value : next.C + w.value;
            break;
        case 'F':
            if (w.value < 0) { error = "Negative feed is invalid"; return false; }
            next_feed = w.value;
            break;
        case 'S':
            if (w.value < 0) { error = "Negative spindle speed is invalid"; return false; }
            next_rpm = w.value;
            break;
        case 'T':
            next_tool = code(w);
            break;
        case 'D':
            next_d = code(w);
            break;
        case 'M':
            switch (code(w)) {
            case 3: next_modal.spindle_direction = 1; break;
            case 4: next_modal.spindle_direction = -1; break;
            case 5: next_modal.spindle_direction = 0; next_rpm = 0; break;
            case 8: next_modal.coolant = true; break;
            case 9: next_modal.coolant = false; break;
            default: break;
            }
            break;
        default:
            break;
        }
    }

    if (!Kinematics().validate(next, error))
        return false;

    state_ = next;
    modal_ = next_modal;
    feed_ = next_feed;
    rpm_ = next_rpm;
    tool_ = next_tool;
    d_ = next_d;
    return true;
}

void Runtime::reset() {
    state_ = {};
    modal_ = {};
    feed_ = rpm_ = 0;
    tool_ = d_ = 0;
}

}
