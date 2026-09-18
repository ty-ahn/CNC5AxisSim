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
 // Head-head chain: A rotates the C-axis pivot and C rotates the tool vector.
 Vec3 cLocal=sub(config_.pivot_c,config_.pivot_a);
 Vec3 cOffset=rotate(cLocal,aa,s.A);
 Vec3 toolLocal=mul({0,0,-1},L+config_.c_to_tool);
 Vec3 toolOffset=rotate(rotate(toolLocal,aa,s.A),cc,s.C);
 return add({s.X,s.Y,s.Z},add(cOffset,toolOffset));
}
