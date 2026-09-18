#pragma once
#include "Kinematics.h"
#include <vector>
namespace cnc {
enum class ToolShape { Flat, Ball, BullNose };
enum class CollisionType { ToolStock, HolderStock, HolderMachine };
struct ToolGeometry { ToolShape shape{ToolShape::Flat}; double radius{2}; double corner_radius{0}; double length{100}; double holder_radius{20}; double holder_length{80}; };
struct StockDefinition { Vec3 origin{}; double size_x{100},size_y{100},size_z{50}; double resolution{2}; };
struct StockCell { bool removed{false}; float sdf{0}; };
struct CollisionEvent { CollisionType type{}; Vec3 position{}; double penetration{}; bool collision{false}; };
class Stock {
 StockDefinition def_{}; int nx_{},ny_{},nz_{}; std::vector<StockCell> cells_;
 size_t index(int x,int y,int z) const{return static_cast<size_t>((z*ny_+y)*nx_+x);}
public:
 bool initialize(const StockDefinition& d);
 bool remove_tool_segment(const Vec3& p0,const Vec3& p1,double radius);
 bool sweep_tool_segment(const Vec3& p0,const Vec3& p1,const ToolGeometry& tool);
 bool intersects_segment(const Vec3& p0,const Vec3& p1,double radius,Vec3* hit=nullptr) const;
 const StockDefinition& definition()const{return def_;}
 size_t removed_count()const;
 CollisionEvent check_holder_stock(const Vec3& holder_start,const Vec3& holder_end,double radius) const;
 size_t cell_count()const{return cells_.size();}
 bool is_removed(int x,int y,int z)const;
 double remaining_volume() const;\n double signed_distance(const Vec3& p) const;
 std::vector<Vec3> removed_centers() const;
};
}