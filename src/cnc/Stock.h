#pragma once
#include "Kinematics.h"
#include <cstdint>
#include <vector>
namespace cnc {
struct StockDefinition { Vec3 origin{}; double size_x{100},size_y{100},size_z{50}; double resolution{2}; };
struct StockCell { bool removed{false}; };
class Stock {
 StockDefinition def_{}; int nx_{},ny_{},nz_{}; std::vector<StockCell> cells_;
 size_t index(int x,int y,int z) const{return static_cast<size_t>((z*ny_+y)*nx_+x);}
public:
 bool initialize(const StockDefinition& d);
 bool remove_tool_segment(const Vec3& p0,const Vec3& p1,double radius);
 const StockDefinition& definition()const{return def_;}
 size_t removed_count()const;
 size_t cell_count()const{return cells_.size();}
 bool is_removed(int x,int y,int z)const;
};
}