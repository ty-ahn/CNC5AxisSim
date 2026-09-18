#pragma once
#include "Kinematics.h"
#include "Parser.h"
namespace cnc {
class Runtime {
    MachineState state_{};
    bool absolute_=true;
    double feed_=0,rpm_=0;
    int tool_=0,d_=0;
public:
    bool execute(const Block&,std::string&);
    const MachineState& state() const{return state_;}
    double feed() const{return feed_;}
    double rpm() const{return rpm_;}
    int tool() const{return tool_;}
    int d() const{return d_;}
    void reset();
};
}