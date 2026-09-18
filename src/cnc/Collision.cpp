#include "Collision.h"
#include <cmath>
#include <algorithm>
namespace cnc {
static Vec3 sub3(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}

bool CollisionManager::segment_aabb(const Vec3&p0,const Vec3&p1,const Aabb&b,double& penetration,Vec3&hit){
 double t0=0,t1=1; Vec3 d=sub3(p1,p0);
 auto slab=[&](double p,double q,double mn,double mx){if(std::abs(q)<1e-12)return p>=mn&&p<=mx;double a=(mn-p)/q,bv=(mx-p)/q;if(a>bv)std::swap(a,bv);t0=std::max(t0,a);t1=std::min(t1,bv);return t0<=t1;};
 if(!slab(p0.x,d.x,b.min.x,b.max.x)||!slab(p0.y,d.y,b.min.y,b.max.y)||!slab(p0.z,d.z,b.min.z,b.max.z))return false;
 double t=std::clamp(t0,0.0,1.0);hit={p0.x+d.x*t,p0.y+d.y*t,p0.z+d.z*t};
 double dx=std::min(hit.x-b.min.x,b.max.x-hit.x),dy=std::min(hit.y-b.min.y,b.max.y-hit.y),dz=std::min(hit.z-b.min.z,b.max.z-hit.z);
 penetration=std::max(0.0,std::min({dx,dy,dz}));return true;
}

CollisionEvent CollisionManager::check_holder_machine(const Vec3&p0,const Vec3&p1,double radius)const{
 CollisionEvent best{CollisionType::HolderMachine,{},0,false};if(radius<0)return best;
 for(const auto&body:machine_bodies_){
  Aabb b=body.bounds;
  if(local_to_world_){Vec3 mn{1e300,1e300,1e300},mx{-1e300,-1e300,-1e300};for(int ix=0;ix<2;++ix)for(int iy=0;iy<2;++iy)for(int iz=0;iz<2;++iz){Vec3 q{ix?body.bounds.max.x:body.bounds.min.x,iy?body.bounds.max.y:body.bounds.min.y,iz?body.bounds.max.z:body.bounds.min.z};q=local_to_world_(body.name,q);mn.x=std::min(mn.x,q.x);mn.y=std::min(mn.y,q.y);mn.z=std::min(mn.z,q.z);mx.x=std::max(mx.x,q.x);mx.y=std::max(mx.y,q.y);mx.z=std::max(mx.z,q.z);}b={mn,mx};}
  b.min.x-=radius;b.min.y-=radius;b.min.z-=radius;b.max.x+=radius;b.max.y+=radius;b.max.z+=radius;double pen;Vec3 hit;if(segment_aabb(p0,p1,b,pen,hit)&&(!best.collision||pen>best.penetration))best={CollisionType::HolderMachine,hit,pen,true};}
 return best;
}

CollisionEvent CollisionManager::check_holder_mesh(const Vec3&p0,const Vec3&p1,double radius)const{
 CollisionEvent best{CollisionType::HolderMachine,{},0,false};if(radius<0)return best;
 for(const auto&body:machine_bodies_) if(body.bvh&&body.mesh&&!body.bvh->empty()){
  Vec3 q0=p0,q1=p1;if(world_to_local_){q0=world_to_local_(body.name,p0);q1=world_to_local_(body.name,p1);}
  Aabb b=body.bounds;b.min.x-=radius;b.min.y-=radius;b.max.x+=radius;b.max.y+=radius;b.min.z-=radius;b.max.z+=radius;
  double pen;Vec3 hit;
  if(!segment_aabb(q0,q1,b,pen,hit))continue;
  Vec3 exact{};
  if(body.bvh->segment_hit(q0,q1,radius,&exact)){if(local_to_world_)exact=local_to_world_(body.name,exact);
   double approxPen=radius;
   if(!best.collision||approxPen>best.penetration)best={CollisionType::HolderMachine,exact,approxPen,true};
  }
 }
 return best;
}

std::vector<MachineCollisionSnapshot> CollisionManager::check_all_machine_bodies(const Vec3&p0,const Vec3&p1,double radius)const{
 std::vector<MachineCollisionSnapshot> out;if(radius<0)return out;
 for(const auto&body:machine_bodies_){Aabb b=body.bounds;if(local_to_world_){Vec3 mn{1e300,1e300,1e300},mx{-1e300,-1e300,-1e300};for(int ix=0;ix<2;++ix)for(int iy=0;iy<2;++iy)for(int iz=0;iz<2;++iz){Vec3 q{ix?body.bounds.max.x:body.bounds.min.x,iy?body.bounds.max.y:body.bounds.min.y,iz?body.bounds.max.z:body.bounds.min.z};q=local_to_world_(body.name,q);mn.x=std::min(mn.x,q.x);mn.y=std::min(mn.y,q.y);mn.z=std::min(mn.z,q.z);mx.x=std::max(mx.x,q.x);mx.y=std::max(mx.y,q.y);mx.z=std::max(mx.z,q.z);}b={mn,mx};}b.min.x-=radius;b.min.y-=radius;b.min.z-=radius;b.max.x+=radius;b.max.y+=radius;b.max.z+=radius;double pen;Vec3 hit;if(segment_aabb(p0,p1,b,pen,hit))out.push_back({body.name,{CollisionType::HolderMachine,hit,pen,true}});}
 return out;
}
}