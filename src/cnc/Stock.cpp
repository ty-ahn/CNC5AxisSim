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
static double pose_radius_at(const ToolGeometry& t,double z){ const double R=std::max(0.0,t.radius),L=std::max(0.0,t.length); if(R<=0||z<0||z>L)return 0; if(t.shape==ToolShape::Ball){if(z<R)return std::sqrt(std::max(0.0,R*R-(R-z)*(R-z)));return R;} if(t.shape==ToolShape::BullNose){const double cr=std::clamp(t.corner_radius,0.0,R);if(cr<=1e-12)return R;if(z<cr)return R-cr+std::sqrt(std::max(0.0,cr*cr-(cr-z)*(cr-z)));return R;} return R; }
static bool remove_oriented_pose(Stock& stock,const Vec3& tip,const Vec3& axis,const ToolGeometry& tool){ const auto& d=stock.definition(); const double h=d.resolution,R=cutter_radius(tool),L=std::max(0.0,tool.length); if(R<=0||L<=0)return false; const int nx=std::max(1,(int)std::ceil(d.size_x/h)),ny=std::max(1,(int)std::ceil(d.size_y/h)),nz=std::max(1,(int)std::ceil(d.size_z/h)); const Vec3 a=norm3(axis); const double reach=R+L; const int ix0=std::clamp((int)std::floor((tip.x-reach-d.origin.x)/h)-1,0,nx-1),ix1=std::clamp((int)std::floor((tip.x+reach-d.origin.x)/h)+1,0,nx-1); const int iy0=std::clamp((int)std::floor((tip.y-reach-d.origin.y)/h)-1,0,ny-1),iy1=std::clamp((int)std::floor((tip.y+reach-d.origin.y)/h)+1,0,ny-1); const int iz0=std::clamp((int)std::floor((tip.z-reach-d.origin.z)/h)-1,0,nz-1),iz1=std::clamp((int)std::floor((tip.z+reach-d.origin.z)/h)+1,0,nz-1); bool changed=false; for(int z=iz0;z<=iz1;++z)for(int y=iy0;y<=iy1;++y)for(int x=ix0;x<=ix1;++x)if(!stock.is_removed(x,y,z)){Vec3 p{d.origin.x+(x+.5)*h,d.origin.y+(y+.5)*h,d.origin.z+(z+.5)*h};Vec3 q{p.x-tip.x,p.y-tip.y,p.z-tip.z};double s=std::clamp(q.x*a.x+q.y*a.y+q.z*a.z,0.0,L);Vec3 r{q.x-a.x*s,q.y-a.y*s,q.z-a.z*s};double rr=pose_radius_at(tool,s);if(r.x*r.x+r.y*r.y+r.z*r.z<=rr*rr){stock.remove_tool_segment(p,p,0.0);changed=true;}} return changed; }
bool Stock::sweep_oriented_tool(const Vec3& tip0,const Vec3& axis0,const Vec3& tip1,const Vec3& axis1,const ToolGeometry& tool,int samples){
 if(cells_.empty()||samples<1||!std::isfinite(tool.radius)||tool.radius<0||!std::isfinite(tool.length)||tool.length<0)return false;
 Vec3 a0=norm3(axis0),a1=norm3(axis1); bool changed=false; Vec3 prev_tip=tip0;
 for(int i=0;i<=samples;++i){double t=double(i)/samples;Vec3 tip=lerp3(tip0,tip1,t);Vec3 axis=norm3(lerp3(a0,a1,t));changed=remove_oriented_pose(*this,tip,axis,tool)||changed;if(i>0)changed=remove_tool_segment(prev_tip,tip,cutter_radius(tool))||changed;prev_tip=tip;} return changed;
}
}
