#pragma once
#include "Stock.h"
#include "Bvh.h"
#include <vector>
#include <string>
#include <algorithm>
#include <functional>
namespace cnc {
struct Aabb { Vec3 min{}, max{}; };
struct MachineCollisionBody { std::string name; Aabb bounds{}; const std::vector<Triangle>* mesh{nullptr}; const Bvh* bvh{nullptr}; };
struct MachineCollisionSnapshot { std::string name; CollisionEvent event{}; };
struct CollisionRecord { int block_number{-1}; CollisionEvent event{}; std::string message; };
class CollisionManager {
 std::vector<CollisionRecord> events_;
 std::vector<MachineCollisionBody> machine_bodies_;
 std::function<Vec3(const std::string&,const Vec3&)> local_to_world_;
 std::function<Vec3(const std::string&,const Vec3&)> world_to_local_;
 static bool segment_aabb(const Vec3&,const Vec3&,const Aabb&,double&,Vec3&);
public:
 void clear(){events_.clear();}
 void record(int block,const CollisionEvent&e,const std::string&msg){if(e.collision)events_.push_back({block,e,msg});}
 const std::vector<CollisionRecord>& events()const{return events_;}
 bool has_collision()const{return !events_.empty();}
 void add_machine_body(const MachineCollisionBody& b){machine_bodies_.push_back(b);}
 void clear_machine_bodies(){machine_bodies_.clear();}
 void set_machine_transformers(std::function<Vec3(const std::string&,const Vec3&)> local_to_world,std::function<Vec3(const std::string&,const Vec3&)> world_to_local){local_to_world_=std::move(local_to_world);world_to_local_=std::move(world_to_local);}
 const std::vector<MachineCollisionBody>& machine_bodies()const{return machine_bodies_;}
 CollisionEvent check_holder_machine(const Vec3& p0,const Vec3& p1,double radius) const;
 CollisionEvent check_holder_mesh(const Vec3& p0,const Vec3& p1,double radius) const;
 std::vector<MachineCollisionSnapshot> check_all_machine_bodies(const Vec3& p0,const Vec3& p1,double radius) const;
 size_t count(CollisionType t)const{size_t n=0;for(const auto&e:events_)if(e.event.type==t)++n;return n;}
 double max_penetration(CollisionType t)const{double m=0;for(const auto&e:events_)if(e.event.type==t)m=std::max(m,e.event.penetration);return m;}
};
}