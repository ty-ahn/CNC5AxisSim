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
 Vec3 cr=cross(axis,v); return add(add(mul(v,c),mul(cr,s)),mul(axis,dot(axis,v)*(1-c)));
}
static Vec3 rotate_about(Vec3 p,Vec3 pivot,Vec3 axis,double deg){return add(pivot,rotate(sub(p,pivot),axis,deg));}
double Kinematics::unwrap(double p,double t){while(t-p>180)t-=360;while(t-p<-180)t+=360;return t;}
Vec3 Kinematics::axis_from_ac(double A,double C){double a=A*PI/180.0,c=C*PI/180.0;return {std::sin(a)*std::cos(c),std::sin(a)*std::sin(c),-std::cos(a)};}
Vec3 Kinematics::configured_axis_from_ac(double A,double C) const{
 Vec3 aa=normalize(config_.a_axis),cc=normalize(config_.c_axis);
 if(norm(aa)<1e-12||norm(cc)<1e-12)return axis_from_ac(A,C);
 Vec3 v=rotate({0,0,-1},aa,A);
 return normalize(rotate(v,cc,C));
}
Vec3 Kinematics::tcp_from_machine(const MachineState&s,double L){
 Vec3 aa=normalize(config_.a_axis),cc=normalize(config_.c_axis);
 if(norm(aa)<1e-12||norm(cc)<1e-12)return {s.X,s.Y,s.Z};
 Vec3 tool=mul(configured_axis_from_ac(s.A,s.C),L+config_.c_to_tool);
 // Head-head chain: C pivot is downstream of A. Its pivot is transformed by A;
 // the tool vector is then rotated by C and translated from the C pivot.
 Vec3 cPivot=rotate_about(config_.pivot_c,config_.pivot_a,aa,s.A);
 Vec3 offset=sub(cPivot,config_.pivot_a);
 Vec3 machineRef=add(config_.pivot_a,offset);
 Vec3 tcp=add({s.X,s.Y,s.Z},sub(config_.pivot_a,machineRef));
 return sub(add(machineRef,tool),tool);
}
bool Kinematics::validate(const MachineState&s,std::string&e)const{
 if(!std::isfinite(s.X)||!std::isfinite(s.Y)||!std::isfinite(s.Z)||!std::isfinite(s.A)||!std::isfinite(s.C)){e="non-finite axis";return false;}
 const auto check=[&](double v,const AxisLimit& l,const char*n){if(l.wrap)return true;if(v<l.minimum||v>l.maximum){e=std::string(n)+" axis limit";return false;}return true;};
 return check(s.X,config_.X,"X")&&check(s.Y,config_.Y,"Y")&&check(s.Z,config_.Z,"Z")&&check(s.A,config_.A,"A")&&check(s.C,config_.C,"C");
}
bool Kinematics::inverse_kinematics(const ToolPose&p,const MachineState&seed,double L,IKResult&o,std::string&e){
 Vec3 d=normalize(p.tool_axis); if(norm(d)<1e-12){e="zero tool axis";return false;}
 double bestCost=std::numeric_limits<double>::infinity(); MachineState best{}; double bestPE=0,bestOE=0; bool found=false;
 double a0=config_.A.minimum,a1=config_.A.maximum;
 int as=std::max(8,(int)std::ceil(std::abs(a1-a0)/5.0));
 int cs=config_.C.wrap?72:std::max(24,(int)std::ceil(std::abs(config_.C.maximum-config_.C.minimum)/5.0));
 for(int ia=0;ia<=as;++ia){double A=a0+(a1-a0)*ia/as;
  for(int ic=0;ic<=cs;++ic){double C=config_.C.minimum+(config_.C.maximum-config_.C.minimum)*ic/cs;
   C=unwrap(seed.C,C); Vec3 qaxis=configured_axis_from_ac(A,C);
   double oe=norm(sub(qaxis,d)); if(oe>0.08)continue;
   Vec3 tcpOffset=mul(qaxis,L+config_.c_to_tool);
   Vec3 machine={p.tcp.x-tcpOffset.x,p.tcp.y-tcpOffset.y,p.tcp.z-tcpOffset.z};
   MachineState q{machine.X,machine.Y,machine.Z,A,C}; if(!validate(q,e))continue;
   double cost=oe*100.0+std::abs(A-seed.A)+std::abs(C-seed.C);
   if(cost<bestCost){bestCost=cost;best=q;bestOE=oe;bestPE=0;found=true;}
  }
 }
 if(!found){e="no IK solution within axis limits";return false;}
 o={best,bestPE,bestOE,bestCost,bestOE<0.08}; return o.valid;
}
bool Kinematics::orientation_move(const ToolPose&p,const MachineState&seed,double L,std::vector<MachineState>&out,std::string&e,int samples){
 out.clear(); if(samples<1)samples=1; IKResult target;
 if(!inverse_kinematics(p,seed,L,target,e))return false;
 Vec3 startTcp=tcp_from_machine(seed,L), endTcp=p.tcp;
 Vec3 startAxis=configured_axis_from_ac(seed.A,seed.C), endAxis=normalize(p.tool_axis);
 double c0=seed.C,c1=unwrap(seed.C,target.state.C);
 for(int i=0;i<=samples;++i){
  double t=double(i)/samples;
  Vec3 tcp=add(mul(startTcp,1-t),mul(endTcp,t));
  Vec3 axis=normalize(add(mul(startAxis,1-t),mul(endAxis,t)));
  MachineState seedI=seed; seedI.A=seed.A+(target.state.A-seed.A)*t; seedI.C=seed.C+(c1-seed.C)*t;
  IKResult ik; ToolPose pose{tcp,axis,0};
  if(!inverse_kinematics(pose,seedI,L,ik,e))return false;
  out.push_back(ik.state);
 }
 return true;
}
bool Kinematics::apply_swivel(double x,double y,double z,double a,double c,ToolPose&out,std::string&e)const{
 if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(a)||!std::isfinite(c)){e="non-finite swivel input";return false;}
 Vec3 aa=normalize(config_.a_axis),cc=normalize(config_.c_axis); if(norm(aa)<1e-12||norm(cc)<1e-12){e="invalid rotary axis vector";return false;}
 Vec3 r=rotate_about({x,y,z},config_.pivot_c,cc,c); r=rotate_about(r,config_.pivot_a,aa,a);
 out.tcp=r; out.tool_axis=normalize(rotate(rotate({0,0,-1},aa,a),cc,c)); return true;
}
bool Kinematics::cycle800_reset(std::string&e){(void)e;swivel_active_=false;swivel_pose_={};return true;}
bool Kinematics::cycle800_swivel(double a,double c,std::string&e){ToolPose p{};if(!apply_swivel(0,0,0,a,c,p,e))return false;swivel_pose_=p;swivel_active_=true;return true;}
}
