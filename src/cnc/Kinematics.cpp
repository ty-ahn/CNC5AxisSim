#include "Kinematics.h"
#include <cmath>
namespace cnc {
double Kinematics::unwrap(double p,double t) {
    while(t-p>180.0) t-=360.0;
    while(t-p<-180.0) t+=360.0;
    return t;
}
bool Kinematics::validate(const MachineState& s,std::string& e) const {
    if(!std::isfinite(s.X)||!std::isfinite(s.Y)||!std::isfinite(s.Z)||!std::isfinite(s.A)||!std::isfinite(s.C)){e="non-finite axis";return false;}
    if(s.A < -120.0 || s.A > 120.0){e="A axis limit";return false;}
    return true;
}
}