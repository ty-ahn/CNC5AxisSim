#include "Stock.h"
#include <algorithm>
#include <cmath>
namespace cnc {
bool Stock::initialize(const StockDefinition& d){
 if(d.size_x<=0||d.size_y<=0||d.size_z<=0||d.resolution<=0)return false;
 def_=d; nx_=std::max(1,(int)std::ceil(d.size_x/d.resolution)); ny_=std::max(1,(int)std::ceil(d.size_y/d.resolution)); nz_=std::max(1,(int)std::ceil(d.size_z/d.resolution));
 cells_.assign((size_t)nx_*ny_*nz_,{}); return true;
}
bool Stock::remove_tool_segment(const Vec3& p0,const Vec3& p1,double radius){
 if(cells_.empty()||radius<0)return false; bool changed=false; double r2=radius*radius;
 for(int z=0;z<nz_;++z)for(int y=0;y<ny_;++y)for(int x=0;x<nx_;++x){
  if(cells_[index(x,y,z)].removed)continue;
  Vec3 c{def_.origin.x+(x+.5)*def_.resolution,def_.origin.y+(y+.5)*def_.resolution,def_.origin.z+(z+.5)*def_.resolution};
  Vec3 v{p1.x-p0.x,p1.y-p0.y,p1.z-p0.z}, w{c.x-p0.x,c.y-p0.y,c.z-p0.z};
  double vv=v.x*v.x+v.y*v.y+v.z*v.z, t=vv>0?(w.x*v.x+w.y*v.y+w.z*v.z)/vv:0; t=std::clamp(t,0.0,1.0);
  Vec3 q{p0.x+t*v.x,p0.y+t*v.y,p0.z+t*v.z}; double dx=c.x-q.x,dy=c.y-q.y,dz=c.z-q.z;
  if(dx*dx+dy*dy+dz*dz<=r2){cells_[index(x,y,z)].removed=true;changed=true;}
 } return changed;
}
bool Stock::sweep_tool_segment(const Vec3& p0,const Vec3& p1,const ToolGeometry& tool){
 double r=tool.radius;
 if(tool.shape==ToolShape::BullNose && tool.corner_radius>0) r=std::max(r,tool.corner_radius);
 if(tool.shape==ToolShape::Ball) return remove_tool_segment(p0,p1,r);
 return remove_tool_segment(p0,p1,r);
}
size_t Stock::removed_count()const{size_t n=0;for(const auto&c:cells_)if(c.removed)++n;return n;}
bool Stock::is_removed(int x,int y,int z)const{return x>=0&&x<nx_&&y>=0&&y<ny_&&z>=0&&z<nz_&&cells_[index(x,y,z)].removed;}
}