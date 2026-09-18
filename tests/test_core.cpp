#include "cnc/Parser.h"
#include "cnc/Runtime.h"
#include "cnc/Kinematics.h"
#include <cassert>
#include <cmath>
int main(){
    auto b=cnc::Parser::parse("N10 G90 G1 X10 Y20 Z30 A30 C45 F1000");
    assert(b && b->number==10);
    cnc::Runtime r; std::string e;
    assert(r.execute(*b,e));
    assert(std::abs(r.state().X-10)<1e-9 && std::abs(r.state().C-45)<1e-9);
    auto b2=cnc::Parser::parse("N20 G91 G1 X5 C20");
    assert(r.execute(*b2,e));
    assert(std::abs(r.state().X-15)<1e-9 && std::abs(r.state().C-65)<1e-9);
    assert(std::abs(cnc::Kinematics::unwrap(359,1)-361)<1e-9);
    return 0;
}