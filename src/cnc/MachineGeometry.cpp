#include "MachineGeometry.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cmath>
namespace cnc {
static float f32(const unsigned char*p){float v;std::memcpy(&v,p,4);return v;}
static void expand(MeshAabb& b,Vec3 p){b.min.x=std::min(b.min.x,p.x);b.min.y=std::min(b.min.y,p.y);b.min.z=std::min(b.min.z,p.z);b.max.x=std::max(b.max.x,p.x);b.max.y=std::max(b.max.y,p.y);b.max.z=std::max(b.max.z,p.z);}
bool MachineGeometry::load_stl(const std::string& path,std::string& error){
 clear();std::ifstream f(path,std::ios::binary);if(!f){error="cannot open STL";return false;}
 unsigned char header[80]{};f.read(reinterpret_cast<char*>(header),80);std::uint32_t n=0;f.read(reinterpret_cast<char*>(&n),4);if(!f){error="invalid STL header";return false;}
 std::streamoff expected=84+static_cast<std::streamoff>(n)*50;f.seekg(0,std::ios::end);auto size=f.tellg();if(size==expected){
  f.seekg(84);triangles_.reserve(n);for(std::uint32_t i=0;i<n;++i){unsigned char rec[50];f.read(reinterpret_cast<char*>(rec),50);if(!f){clear();error="truncated binary STL";return false;}Triangle t{{f32(rec+12),f32(rec+16),f32(rec+20)},{f32(rec+24),f32(rec+28),f32(rec+32)},{f32(rec+36),f32(rec+40),f32(rec+44)}};triangles_.push_back(t);if(i==0)bounds_.min=bounds_.max=t.a;expand(bounds_,t.b);expand(bounds_,t.c);}loaded_=!triangles_.empty();return loaded_;
 }
 f.close();std::ifstream a(path);if(!a){error="cannot reopen STL";return false;}std::string line;std::vector<Vec3> v;while(std::getline(a,line)){std::istringstream ss(line);std::string w;ss>>w;if(w=="vertex"){Vec3 p{};ss>>p.x>>p.y>>p.z;if(!ss){clear();error="invalid ASCII STL vertex";return false;}v.push_back(p);if(v.size()==3){Triangle t{v[0],v[1],v[2]};triangles_.push_back(t);if(triangles_.size()==1)bounds_.min=bounds_.max=t.a;expand(bounds_,t.b);expand(bounds_,t.c);v.clear();}}}
 if(!v.empty()||triangles_.empty()){clear();error="invalid or empty STL";return false;}loaded_=true;return true;
}
bool MachineGeometry::save_binary_stl(const std::string& path,std::string& error)const{
 if(triangles_.empty()){error="empty mesh";return false;}std::ofstream f(path,std::ios::binary);if(!f){error="cannot write STL";return false;}char header[80]{};std::memcpy(header,"CNC5AxisSim machine mesh",23);f.write(header,80);std::uint32_t n=static_cast<std::uint32_t>(triangles_.size());f.write(reinterpret_cast<const char*>(&n),4);for(const auto&t:triangles_){float rec[12]={0,0,0,(float)t.a.x,(float)t.a.y,(float)t.a.z,(float)t.b.x,(float)t.b.y,(float)t.b.z,(float)t.c.x,(float)t.c.y,(float)t.c.z};f.write(reinterpret_cast<const char*>(rec),sizeof(rec));std::uint16_t attr=0;f.write(reinterpret_cast<const char*>(&attr),2);}return !!f;
}
}