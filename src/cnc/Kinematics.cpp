#include "Kinematics.h"
#include <cmath>
#include <algorithm>
#include <limits>
namespace cnc {
double Kinematics::unwrap(double p,double t){while(t-p>180)t-=360;while(t-p<-180)t+=360;return t;}
Vec3 Kinematics::axis_from_ac(double A,double C){constexpr double d=3.14159265358979323846/180.0;double a=A*d,c=C*d;return {std::sin(a)*std::cos(c),std::sin(a)*std::sin(c),-std::cos(a)};}
Vec3 Kinematics::tcp_from_machine(const MachineState&s,double L){auto q=axis_from_ac(s.A,s.C);double reach=config_.a_to_c+config_.c_to_tool+L; return {s.X-reach*q.x,s.Y-reach*q.y,s.Z-reach*q.z};}
bool Kinematics::validate(const MachineState&s,std::string&e)const{if(!std::isfinite(s.X)||!std::isfinite(s.Y)||!std::isfinite(s.Z)||!std::isfinite(s.A)||!std::isfinite(s.C)){e="non-finite axis";return false;}const auto check=[&](double v,const AxisLimit& l,const char* n){if(l.wrap)return true;if(v<l.minimum||v>l.maximum){e=std::string(n)+" axis limit";return false;}return true;}; if(!check(s.X,config_.X,"X")||!check(s.Y,config_.Y,"Y")||!check(s.Z,config_.Z,"Z")||!check(s.A,config_.A,"A")||!check(s.C,config_.C,"C"))return false;return true;}
bool Kinematics::inverse_kinematics(const ToolPose&p,const MachineState&seed,double L,IKResult&o,std::string&e){
 double n=std::sqrt(p.tool_axis.x*p.tool_axis.x+p.tool_axis.y*p.tool_axis.y+p.tool_axis.z*p.tool_axis.z);if(n<1e-12){e="zero tool axis";return false;}
 Vec3 d{p.tool_axis.x/n,p.tool_axis.y/n,p.tool_axis.z/n};double A=std::acos(std::clamp(-d.z,-1.0,1.0))*180.0/3.14159265358979323846;double C=std::atan2(d.y,d.x)*180.0/3.14159265358979323846;A=std::clamp(A,0.0,180.0);C=unwrap(seed.C,C);
 double reach=config_.a_to_c+config_.c_to_tool+L; MachineState q{p.tcp.x+reach*d.x,p.tcp.y+reach*d.y,p.tcp.z+reach*d.z,A,C};if(!validate(q,e))return false;
 auto fk=tcp_from_machine(q,L);double pe=std::hypot(std::hypot(fk.x-p.tcp.x,fk.y-p.tcp.y),fk.z-p.tcp.z);
 auto qa=axis_from_ac(q.A,q.C);double oe=std::sqrt((qa.x-d.x)*(qa.x-d.x)+(qa.y-d.y)*(qa.y-d.y)+(qa.z-d.z)*(qa.z-d.z));
 o={q,pe,oe,std::abs(q.A-seed.A)+std::abs(q.C-seed.C),pe<1e-7&&oe<1e-7};if(!o.valid){e="IK solution exceeds tolerance";return false;}return true;
bool Kinematics::orientation_move(const ToolPose&p,const MachineState&seed,double L,std::vector<MachineState>&out,std::string&e,int samples){ out.clear(); if(samples<1)samples=1; IKResult ik; if(!inverse_kinematics(p,seed,L,ik,e)) return false; double a0=seed.A,a1=ik.state.A,c0=seed.C,c1=unwrap(c0,ik.state.C); for(int i=0;i<=samples;++i){double t=double(i)/samples; MachineState q=seed; q.A=a0+(a1-a0)*t; q.C=c0+(c1-c0)*t; if(!validate(q,e))return false; out.push_back(q);} return true; }
bool Kinematics::apply_swivel(double x,double y,double z,double a,double c,ToolPose& out,std::string&e) const { if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(a)||!std::isfinite(c)){e="non-finite swivel input";return false;} double ar=a*3.14159265358979323846/180.0, cr=c*3.14159265358979323846/180.0; Vec3 v{x,y,z}; Vec3 rx{v.x, v.y*std::cos(ar)-v.z*std::sin(ar), v.y*std::sin(ar)+v.z*std::cos(ar)}; out.tcp={rx.x*std::cos(cr)-rx.y*std::sin(cr),rx.x*std::sin(cr)+rx.y*std::cos(cr),rx.z}; out.tool_axis={std::sin(ar)*std::cos(cr),std::sin(ar)*std::sin(cr),-std::cos(ar)}; return true; }
bool Kinematics::cycle800_reset(std::string&e){ (void)e; swivel_active_=false; swivel_pose_={}; return true; }
bool Kinematics::cycle800_swivel(double a,double c,std::string&e){ ToolPose p{}; if(!apply_swivel(0,0,-1,a,c,p,e)) return false; swivel_pose_=p; swivel_active_=true; return true; }
}
}