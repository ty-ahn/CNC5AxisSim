#include "ArcPlanner.h"
#include <cmath>
#include <algorithm>
namespace cnc {
bool ArcPlanner::plan(const MotionPoint&s,const MotionPoint&t,const ArcDefinition&o,bool cw,double feed,std::vector<MotionPoint>&out,std::string&e,int n){
 if(n<2||feed<0||!std::isfinite(feed)){e="invalid arc parameters";return false;}
 // ArcPlanner's public API retains I/J/K; Runtime selects the active plane and maps it to XY/XZ/YZ before planning.
 const double cx=s.X+o.I,cy=s.Y+o.J,sx=s.X-cx,sy=s.Y-cy,tx=t.X-cx,ty=t.Y-cy,r0=std::hypot(sx,sy),r1=std::hypot(tx,ty);
 if(r0<1e-9||std::abs(r0-r1)>1e-5){e="invalid G2/G3 arc radius";return false;}
 constexpr double pi=3.14159265358979323846; double a0=std::atan2(sy,sx),da=std::atan2(ty,tx)-a0;
 if(cw){while(da>=0)da-=2*pi;}else{while(da<=0)da+=2*pi;} if(std::abs(da)<1e-12)da=cw?-2*pi:2*pi;
 out.clear();out.reserve(n+1);for(int i=0;i<=n;i++){double u=double(i)/n,a=a0+da*u;MotionPoint p{cx+r0*std::cos(a),cy+r0*std::sin(a),s.Z+(t.Z-s.Z)*u,s.A+(Kinematics::unwrap(s.A,t.A)-s.A)*u,s.C+(Kinematics::unwrap(s.C,t.C)-s.C)*u};out.push_back(p);if(!Kinematics().validate({p.X,p.Y,p.Z,p.A,p.C},e))return false;}out.back()=t;return true;
}}
