#include "cnc/Runtime.h"
#include "cnc/Kinematics.h"
#include <cassert>
#include <cmath>
int main(){cnc::Runtime r;std::string e;auto run=[&](const char*x){auto b=cnc::Parser::parse(x);assert(b);assert(r.execute(*b,e));};
run("N10 G90 G54 G0 X0 Y0 Z100");run("N20 T1 D1 S8000 M3");run("N30 G1 X100 Y50 Z20 A30 C45 F1000");assert(r.last_motion().size()==21);assert(r.modal().motion==cnc::MotionMode::Linear);
run("N40 G91 X5 C20");assert(std::abs(r.state().X-105)<1e-9&&std::abs(r.state().C-65)<1e-9);
run("N50 G90 G0 X0 Y0 Z0");run("N60 G17 G2 X20 Y0 I10 J0 Z-10");assert(r.last_motion().size()==33);assert(std::abs(r.state().X-20)<1e-9);
run("N70 G3 X0 Y0 I-10 J0");assert(std::abs(r.state().X)<1e-9&&std::abs(r.state().Y)<1e-9);
run("N80 TRAORI G90 G1 X10 Y0 Z0 I0 J0 K-1 F500"); assert(r.modal().traori); assert(std::abs(r.state().A)<1e-9); assert(std::abs(r.state().C)<1e-9); run("N85 TRAFOOF"); assert(!r.modal().traori); run("N90 G55 M5");assert(r.modal().work_offset==55&&r.rpm()==0);assert(std::abs(cnc::Kinematics::unwrap(359,1)-361)<1e-9);}