#include "Stock.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace cnc {
static double dist2_segment(const Vec3& c,const Vec3& p0,const Vec3& p1){
 Vec3 v{p1.x-p0.x,p1.y-p0.y,p1.z-p0.z},w{c.x-p0.x,c.y-p0.y,c.z-p0.z};
 double vv=v.x*v.x+v.y*v.y+v.z*v.z,t=vv>0?(w.x*v.x+w.y*v.y+w.z*v.z)/vv:0;
 t=std::clamp(t,0.0,1.0); Vec3 q{p0.x+t*v.x,p0.y+t*v.y,p0.z+t*v.z};
 double dx=c.x-q.x,dy=c.y-q.y,dz=c.z-q.z; return dx*dx+dy*dy+dz*dz;
}
bool Stock::initialize(const StockDefinition& d){
 if(!std::isfinite(d.size_x)||!std::isfinite(d.size_y)||!std::isfinite(d.size_z)||!std::isfinite(d.resolution)||
    d.size_x<=0||d.size_y<=0||d.size_z<=0||d.resolution<=0)return false;
 def_=d; nx_=std::max(1,(int)std::ceil(d.size_x/d.resolution)); ny_=std::max(1,(int)std::ceil(d.size_y/d.resolution)); nz_=std::max(1,(int)std::ceil(d.size_z/d.resolution));
 cells_.assign((size_t)nx_*ny_*nz_,{}); return true;
}
bool Stock::remove_tool_segment(const Vec3& p0,const Vec3& p1,double radius){
 if(cells_.empty()||!std::isfinite(radius)||radius<0)return false; bool changed=false; double r2=radius*radius;
 for(int z=0;z<nz_;++z)for(int y=0;y<ny_;++y)for(int x=0;x<nx_;++x)if(!cells_[index(x,y,z)].removed){
  Vec3 c{def_.origin.x+(x+.5)*def_.resolution,def_.origin.y+(y+.5)*def_.resolution,def_.origin.z+(z+.5)*def_.resolution};
  if(dist2_segment(c,p0,p1)<=r2){cells_[index(x,y,z)].removed=true;changed=true;}
 } return changed;
}
bool Stock::sweep_tool_segment(const Vec3& p0,const Vec3& p1,const ToolGeometry& tool){
 if(cells_.empty()||!std::isfinite(tool.radius)||tool.radius<0||!std::isfinite(tool.length)||tool.length<0)return false;
 double r=tool.radius;
 switch(tool.shape){
  case ToolShape::Ball: r=tool.radius; break;
  case ToolShape::BullNose: r=std::max(tool.radius,std::max(0.0,tool.corner_radius)); break;
  case ToolShape::Flat: r=tool.radius; break;
 }
 // Conservative voxelization: the centerline sweep is dilated by the effective radius.
 // Exact orientation-aware cutter/stock intersection belongs to the future SDF kernel.
 return remove_tool_segment(p0,p1,r);
}
bool Stock::intersects_segment(const Vec3& p0,const Vec3& p1,double radius,Vec3* hit) const{
 if(cells_.empty()||!std::isfinite(radius)||radius<0)return false; double r2=radius*radius;
 for(int z=0;z<nz_;++z)for(int y=0;y<ny_;++y)for(int x=0;x<nx_;++x)if(!cells_[index(x,y,z)].removed){
  Vec3 c{def_.origin.x+(x+.5)*def_.resolution,def_.origin.y+(y+.5)*def_.resolution,def_.origin.z+(z+.5)*def_.resolution};
  if(dist2_segment(c,p0,p1)<=r2){if(hit)*hit=c;return true;}
 } return false;
}
CollisionEvent Stock::check_holder_stock(const Vec3& p0,const Vec3& p1,double radius) const{
 CollisionEvent e{}; e.type=CollisionType::HolderStock; if(cells_.empty()||radius<0)return e; double r2=radius*radius;
 for(int z=0;z<nz_;++z)for(int y=0;y<ny_;++y)for(int x=0;x<nx_;++x)if(!cells_[index(x,y,z)].removed){
  Vec3 c{def_.origin.x+(x+.5)*def_.resolution,def_.origin.y+(y+.5)*def_.resolution,def_.origin.z+(z+.5)*def_.resolution};
  double d2=dist2_segment(c,p0,p1); if(d2<=r2){e.collision=true;e.position=c;e.penetration=radius-std::sqrt(d2);return e;}
 } return e;
}
size_t Stock::removed_count()const{size_t n=0;for(const auto&c:cells_)if(c.removed)++n;return n;}
bool Stock::is_removed(int x,int y,int z)const{return x>=0&&x<nx_&&y>=0&&y<ny_&&z>=0&&z<nz_&&cells_[index(x,y,z)].removed;}
double Stock::remaining_volume() const {return double(cells_.size()-removed_count())*def_.resolution*def_.resolution*def_.resolution;}
std::vector<Vec3> Stock::removed_centers() const {std::vector<Vec3> out;out.reserve(removed_count());for(int z=0;z<nz_;++z)for(int y=0;y<ny_;++y)for(int x=0;x<nx_;++x)if(cells_[index(x,y,z)].removed)out.push_back({def_.origin.x+(x+.5)*def_.resolution,def_.origin.y+(y+.5)*def_.resolution,def_.origin.z+(z+.5)*def_.resolution});return out;}
double Stock::signed_distance(const Vec3& p) const {
 if(cells_.empty()) return 0;
 const double h=def_.resolution;
 const double xmin=def_.origin.x, ymin=def_.origin.y, zmin=def_.origin.z;
 const double xmax=xmin+def_.size_x, ymax=ymin+def_.size_y, zmax=zmin+def_.size_z;
 double dx=std::max({xmin-p.x,0.0,p.x-xmax});
 double dy=std::max({ymin-p.y,0.0,p.y-ymax});
 double dz=std::max({zmin-p.z,0.0,p.z-zmax});
 double outside=std::sqrt(dx*dx+dy*dy+dz*dz);
 int ix=std::clamp((int)std::floor((p.x-xmin)/h),0,nx_-1);
 int iy=std::clamp((int)std::floor((p.y-ymin)/h),0,ny_-1);
 int iz=std::clamp((int)std::floor((p.z-zmin)/h),0,nz_-1);
 bool removed=is_removed(ix,iy,iz);
 double fx=(p.x-(xmin+ix*h))/h,fy=(p.y-(ymin+iy*h))/h,fz=(p.z-(zmin+iz*h))/h;
 double local=std::min({fx,1.0-fx,fy,1.0-fy,fz,1.0-fz})*h;
 return removed ? outside+local : -(outside+local);
}

static Vec3 norm3(Vec3 v){
 double n=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
 return n<1e-12?Vec3{0,0,-1}:Vec3{v.x/n,v.y/n,v.z/n};
}
static Vec3 lerp3(Vec3 a,Vec3 b,double t){return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t};}
static double cutter_radius(const ToolGeometry& t){
 if(t.shape==ToolShape::BullNose) return std::max(t.radius,std::max(0.0,t.corner_radius));
 return std::max(0.0,t.radius);
}
bool Stock::sweep_oriented_tool(const Vec3& tcp0,const Vec3& axis0,const Vec3& tcp1,const Vec3& axis1,const ToolGeometry& tool,int samples){
 if(cells_.empty()||samples<1||!std::isfinite(tool.radius)||tool.radius<0||!std::isfinite(tool.length)||tool.length<0)return false;
 Vec3 a0=norm3(axis0),a1=norm3(axis1); bool changed=false;
 const double r=cutter_radius(tool);
 auto tip=[&](const Vec3& tcp,const Vec3& axis){return Vec3{tcp.x-axis.x*tool.length,tcp.y-axis.y*tool.length,tcp.z-axis.z*tool.length};};
 Vec3 prev_tcp=tcp0,prev_axis=a0;
 for(int i=1;i<=samples;++i){
  double t=double(i)/samples;
  Vec3 tcp=lerp3(tcp0,tcp1,t);
  Vec3 axis=norm3(lerp3(a0,a1,t));
  Vec3 p0=tip(prev_tcp,prev_axis),p1=tip(tcp,axis);
  changed=remove_tool_segment(prev_tcp,tcp,r)||changed;
  changed=remove_tool_segment(p0,p1,r)||changed;
  changed=remove_tool_segment(tcp,p1,r)||changed;
  prev_tcp=tcp; prev_axis=axis;
 }
 double d=std::clamp(a0.x*a1.x+a0.y*a1.y+a0.z*a1.z,-1.0,1.0);
 double angle=std::acos(d);
 double envelope=r+tool.length*std::sin(0.5*angle);
 if(envelope>r) changed=remove_tool_segment(tcp0,tcp1,envelope)||changed;
 return changed;
}
}
