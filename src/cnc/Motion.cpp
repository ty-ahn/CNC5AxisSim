#include "Motion.h"
#include <cmath>
namespace cnc {
double MotionPlanner::rotary_distance(double a,double b){return std::abs(Kinematics::unwrap(a,b)-a);}
double MotionPlanner::linear_length(const MotionPoint&a,const MotionPoint&b){double dx=b.X-a.X,dy=b.Y-a.Y,dz=b.Z-a.Z,da=rotary_distance(a.A,b.A),dc=rotary_distance(a.C,b.C);return std::sqrt(dx*dx+dy*dy+dz*dz+da*da+dc*dc);}
bool MotionPlanner::plan(const MotionPoint&s,const MotionPoint&t,bool,double feed,std::vector<MotionPoint>&out,std::string&e,int n,const Kinematics* kin){
 if(n<1||!std::isfinite(feed)||feed<0){e="invalid motion parameters";return false;} double A=Kinematics::unwrap(s.A,t.A),C=Kinematics::unwrap(s.C,t.C); out.clear(); out.reserve(n+1);
 for(int i=0;i<=n;i++){double u=double(i)/n; MotionPoint p{s.X+(t.X-s.X)*u,s.Y+(t.Y-s.Y)*u,s.Z+(t.Z-s.Z)*u,s.A+(A-s.A)*u,s.C+(C-s.C)*u}; out.push_back(p); if(kin ? !kin->validate({p.X,p.Y,p.Z,p.A,p.C},e) : !Kinematics().validate({p.X,p.Y,p.Z,p.A,p.C},e))return false;} return true;
}
}