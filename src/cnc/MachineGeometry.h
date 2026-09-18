#pragma once
#include "Kinematics.h"
#include <string>
#include <vector>
namespace cnc {
struct Triangle { Vec3 a{},b{},c{}; };
struct MeshAabb { Vec3 min{},max{}; };
class MachineGeometry {
 std::vector<Triangle> triangles_;
 MeshAabb bounds_{};
 bool loaded_{false};
public:
 bool load_stl(const std::string& path,std::string& error);
 bool save_binary_stl(const std::string& path,std::string& error) const;
 const std::vector<Triangle>& triangles() const{return triangles_;}
 const MeshAabb& bounds() const{return bounds_;}
 bool loaded() const{return loaded_;}
 void clear(){triangles_.clear();bounds_={};loaded_=false;}
};
}