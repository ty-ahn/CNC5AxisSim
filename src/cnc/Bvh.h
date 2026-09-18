#pragma once
#include "MachineGeometry.h"
#include <vector>
#include <cstdint>
namespace cnc {
struct BvhNode { MeshAabb bounds{}; int left{-1},right{-1}; std::uint32_t begin{},count{}; bool leaf()const{return left<0&&right<0;} };
class Bvh {
 std::vector<BvhNode> nodes_;
 std::vector<std::uint32_t> indices_;
 const std::vector<Triangle>* triangles_{};
 int build(std::uint32_t begin,std::uint32_t end);
 static MeshAabb tri_bounds(const Triangle&);
 static MeshAabb merge(const MeshAabb&,const MeshAabb&);
public:
 void build(const std::vector<Triangle>& triangles);
 bool empty()const{return nodes_.empty();}
 const std::vector<BvhNode>& nodes()const{return nodes_;}
 bool segment_hit(const Vec3& p0,const Vec3& p1,double radius,Vec3* hit=nullptr) const;
};
}