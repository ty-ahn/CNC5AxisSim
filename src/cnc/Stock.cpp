#include "Stock.h"
#include <algorithm>
#include <cmath>
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
}