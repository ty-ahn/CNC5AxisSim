#include "Kinematics.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace cnc {
static constexpr double PI=3.14159265358979323846;
static Vec3 add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
static Vec3 sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
static Vec3 mul(Vec3 a,double s){return {a.x*s,a.y*s,a.z*s};}
static double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static double norm(Vec3 a){return std::sqrt(dot(a,a));}
static Vec3 normalize(Vec3 a){double n=norm(a);return n<1e-12?Vec3{0,0,0}:mul(a,1.0/n);}
static Vec3 cross(Vec3 a,Vec3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static Vec3 rotate(Vec3 v,Vec3 axis,double deg){
 axis=normalize(axis); if(norm(axis)<1e-12)return v;
 double r=deg*PI/180.0,c=std::cos(r),s=std::sin(r);
 Vec3 cr=cross(axis,v);
 return add(add(mul(v,c),mul(cr,s)),mul(axis,dot(axis,v)*(1-c)));
}
static Vec3 rotate_about(Vec3 p,Vec3 pivot,Vec3 axis,double deg){return add(pivot,rotate(sub(p,pivot),axis,deg));}
double Kinematics::unwrap(double p,double t){while(t-p>180)t-=360;while(t-p<-180)t+=360;return t;}
Vec3 Kinematics::axis_from_ac(double A,double C){double a=A*PI/180.0,c=C*PI/180.0;return {std::sin(a)*std::cos(c),std::sin(a)*std::sin(c),-std::cos(a)};}
Vec3 Kinematics::configured_axis_from_ac(double A,double C) const{
 Vec3 aa=normalize(config_.a_axis),cc=normalize(config_.c_axis);
 if(norm(aa)<1e-12||norm(cc)<1e-12)return axis_from_ac(A,C);
 Vec3 cAxisWorld=normalize(rotate(cc,aa,A));
 Vec3 v=rotate({0,0,-1},aa,A);
 return normalize(rotate(v,cAxisWorld,C));
}
Vec3 Kinematics::tcp_from_machine(const MachineState&s,double L){
 Vec3 aa=normalize(config_.a_axis),cc=normalize(config_.c_axis);
 if(norm(aa)<1e-12||norm(cc)<1e-12)return {s.X,s.Y,s.Z};
 Vec3 cLocal=sub(config_.pivot_c,config_.pivot_a);
 Vec3 cOffset=rotate(cLocal,aa,s.A);
 Vec3 cAxisWorld=normalize(rotate(cc,aa,s.A));
 Vec3 toolLocal=mul({0,0,-1},L+config_.c_to_tool);
 Vec3 toolAfterA=rotate(toolLocal,aa,s.A);
 Vec3 toolOffset=rotate(toolAfterA,cAxisWorld,s.C);
 return add({s.X,s.Y,s.Z},add(cOffset,toolOffset));
}
bool Kinematics::validate(const MachineState&s,std::string&e)const{
 if(!std::isfinite(s.X)||!std::isfinite(s.Y)||!std::isfinite(s.Z)||!std::isfinite(s.A)||!std::isfinite(s.C)){e="non-finite axis";return false;}
 const auto check=[&](double v,const AxisLimit& l,const char* n){if(l.wrap)return true;if(v<l.minimum||v>l.maximum){e=std::string(n)+" axis limit";return false;}return true;};
 return check(s.X,config_.X,"X")&&check(s.Y,config_.Y,"Y")&&check(s.Z,config_.Z,"Z")&&check(s.A,config_.A,"A")&&check(s.C,config_.C,"C");
}
bool Kinematics::inverse_kinematics(const ToolPose&p,const MachineState&seed,double L,IKResult&o,std::string&e){
 Vec3 d=normalize(p.tool_axis); if(norm(d)<1e-12){e="zero tool axis";return false;}
 double A=std::acos(std::clamp(-d.z,-1.0,1.0))*180.0/PI;
 double C=std::atan2(d.y,d.x)*180.0/PI; C=unwrap(seed.C,C);
 MachineState q=seed; q.A=A; q.C=C;
 Vec3 zero=tcp_from_machine({0,0,0,A,C},L);
 q.X=p.tcp.x-zero.x; q.Y=p.tcp.y-zero.y; q.Z=p.tcp.z-zero.z;
 if(!validate(q,e))return false;
 Vec3 fk=tcp_from_machine(q,L), qa=configured_axis_from_ac(q.A,q.C);
 double pe=std::sqrt((fk.x-p.tcp.x)*(fk.x-p.tcp.x)+(fk.y-p.tcp.y)*(fk.y-p.tcp.y)+(fk.z-p.tcp.z)*(fk.z-p.tcp.z));
 double oe=std::sqrt((qa.x-d.x)*(qa.x-d.x)+(qa.y-d.y)*(qa.y-d.y)+(qa.z-d.z)*(qa.z-d.z));
 o={q,pe,oe,std::abs(q.A-seed.A)+std::abs(q.C-seed.C),pe<1e-7&&oe<1e-7};
 if(!o.valid){e="IK solution exceeds tolerance";return false;} return true;
}
bool Kinematics::orientation_move(const ToolPose&p,const MachineState&seed,double L,std::vector<MachineState>&out,std::string&e,int samples){
 out.clear(); if(samples<1)samples=1; IKResult ik; if(!inverse_kinematics(p,seed,L,ik,e))return false;
 double a0=seed.A,a1=ik.state.A,c0=seed.C,c1=unwrap(c0,ik.state.C);
 for(int i=0;i<=samples;++i){double t=double(i)/samples; MachineState q=seed; q.A=a0+(a1-a0)*t;q.C=c0+(c1-c0)*t;Vec3 zero=tcp_from_machine({0,0,0,q.A,q.C},L);q.X=p.tcp.x-zero.x;q.Y=p.tcp.y-zero.y;q.Z=p.tcp.z-zero.z;if(!validate(q,e))return false;Vec3 fk=tcp_from_machine(q,L);double err=norm(sub(fk,p.tcp));if(err>1e-7){e="TRAORI TCP preservation error";return false;}out.push_back(q);}return true;
}
bool Kinematics::apply_swivel(double x,double y,double z,double a,double c,ToolPose&out,std::string&e) const{
 if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(a)||!std::isfinite(c)){e="non-finite swivel input";return false;}
 Vec3 aa=normalize(config_.a_axis),cc=normalize(config_.c_axis); if(norm(aa)<1e-12||norm(cc)<1e-12){e="invalid rotary axis vector";return false;}
 Vec3 v{x,y,z};
 // Treat the configured pivots as the two rotary centers in the A-head/C-head chain.
 Vec3 r=add(config_.pivot_a,rotate(sub(v,config_.pivot_c),cc,c));
 r=rotate_about(r,config_.pivot_a,aa,a);
 out.tcp=r;
 out.tool_axis=configured_axis_from_ac(a,c);
 return true;
}
bool Kinematics::cycle800_reset(std::string&e){(void)e;swivel_active_=false;swivel_pose_={};return true;}
bool Kinematics::cycle800_swivel(double a,double c,std::string&e){ToolPose p{};if(!apply_swivel(0,0,-1,a,c,p,e))return false;swivel_pose_=p;swivel_active_=true;return true;}
}
