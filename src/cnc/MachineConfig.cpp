#include "MachineConfig.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
#include <algorithm>
namespace cnc {
static bool num(const std::string&s,const std::string& key,double&v){std::regex r("\""+key+"\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");std::smatch m;if(!std::regex_search(s,m,r))return false;v=std::stod(m[1]);return true;}
bool MachineConfig::load_json(const std::string& path,std::string& e){
 std::ifstream f(path);if(!f){e="cannot open machine config";return false;}std::ostringstream ss;ss<<f.rdbuf();auto s=ss.str();
 auto get=[&](const char*k,AxisLimit&a){double lo,hi;if(!num(s,std::string(k)+"_min",lo)||!num(s,std::string(k)+"_max",hi))return false;if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi)return false;a.minimum=lo;a.maximum=hi;return true;};
 if(!get("X",k_.X)||!get("Y",k_.Y)||!get("Z",k_.Z)||!get("A",k_.A)||!get("C",k_.C)){e="machine axis limits missing";return false;}
 double v;if(num(s,"a_to_c",v))k_.a_to_c=v;if(num(s,"c_to_tool",v))k_.c_to_tool=v;
 double ax,ay,az,cx,cy,cz;if(num(s,"A_axis_x",ax)&&num(s,"A_axis_y",ay)&&num(s,"A_axis_z",az))k_.a_axis={ax,ay,az};if(num(s,"C_axis_x",cx)&&num(s,"C_axis_y",cy)&&num(s,"C_axis_z",cz))k_.c_axis={cx,cy,cz};
 double px,py,pz;if(num(s,"A_pivot_x",px)&&num(s,"A_pivot_y",py)&&num(s,"A_pivot_z",pz))k_.pivot_a={px,py,pz};if(num(s,"C_pivot_x",px)&&num(s,"C_pivot_y",py)&&num(s,"C_pivot_z",pz))k_.pivot_c={px,py,pz};
 std::regex nr("\"name\"\\s*:\\s*\"([^\"]+)\"");std::smatch m;if(std::regex_search(s,m,nr))name_=m[1];
 nodes_.clear(); const char* keys[]={"X","Y","Z","A","C"};
 for(int i=0;i<5;++i){double lo=0,hi=0;num(s,std::string(keys[i])+"_min",lo);num(s,std::string(keys[i])+"_max",hi);MachineNode n;n.name=keys[i];n.type=i<3?MachineNodeType::LinearAxis:MachineNodeType::RotaryAxis;n.min=lo;n.max=hi;n.wrap=(i==4);n.parent=i==0?"":i==1?"X":i==2?"Y":i==3?"Z":"A";n.axis=i==3?k_.a_axis:k_.c_axis;n.pivot=i==3?k_.pivot_a:k_.pivot_c;nodes_.push_back(n);}
 std::regex body("\"body_([A-Za-z0-9_]+)_(min_x|max_x|min_y|max_y|min_z|max_z)\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
 struct B{MachineNode n;bool seen[6]{};}; std::vector<B> bodies;
 for(std::sregex_iterator it(s.begin(),s.end(),body),endit;it!=endit;++it){std::string name=(*it)[1],key=(*it)[2];double v=std::stod((*it)[3]);auto bi=std::find_if(bodies.begin(),bodies.end(),[&](const B&b){return b.n.name==name;});if(bi==bodies.end()){B b{};b.n.name=name;b.n.type=MachineNodeType::Fixture;bi=bodies.insert(bodies.end(),b)-1;}int k=key=="min_x"?0:key=="max_x"?1:key=="min_y"?2:key=="max_y"?3:key=="min_z"?4:5;bi->n.collision_enabled=true;bi->n.collision_bounds.min.x=k==0?v:bi->n.collision_bounds.min.x;bi->n.collision_bounds.max.x=k==1?v:bi->n.collision_bounds.max.x;bi->n.collision_bounds.min.y=k==2?v:bi->n.collision_bounds.min.y;bi->n.collision_bounds.max.y=k==3?v:bi->n.collision_bounds.max.y;bi->n.collision_bounds.min.z=k==4?v:bi->n.collision_bounds.min.z;bi->n.collision_bounds.max.z=k==5?v:bi->n.collision_bounds.max.z;}
 for(const auto& b:bodies)nodes_.push_back(b.n); return true;
}
bool MachineConfig::save_json(const std::string& path,std::string&e)const{
 std::ofstream f(path);if(!f){e="cannot write machine config";return false;}
 f<<"{\n  \"name\": \""<<name_<<"\",\n  \"X_min\": "<<k_.X.minimum<<", \"X_max\": "<<k_.X.maximum<<",\n  \"Y_min\": "<<k_.Y.minimum<<", \"Y_max\": "<<k_.Y.maximum<<",\n  \"Z_min\": "<<k_.Z.minimum<<", \"Z_max\": "<<k_.Z.maximum<<",\n  \"A_min\": "<<k_.A.minimum<<", \"A_max\": "<<k_.A.maximum<<",\n  \"C_min\": "<<k_.C.minimum<<", \"C_max\": "<<k_.C.maximum<<",\n  \"a_to_c\": "<<k_.a_to_c<<", \"c_to_tool\": "<<k_.c_to_tool<<",\n  \"A_axis_x\": "<<k_.a_axis.x<<", \"A_axis_y\": "<<k_.a_axis.y<<", \"A_axis_z\": "<<k_.a_axis.z<<",\n  \"C_axis_x\": "<<k_.c_axis.x<<", \"C_axis_y\": "<<k_.c_axis.y<<", \"C_axis_z\": "<<k_.c_axis.z<<",\n  \"A_pivot_x\": "<<k_.pivot_a.x<<", \"A_pivot_y\": "<<k_.pivot_a.y<<", \"A_pivot_z\": "<<k_.pivot_a.z<<",\n  \"C_pivot_x\": "<<k_.pivot_c.x<<", \"C_pivot_y\": "<<k_.pivot_c.y<<", \"C_pivot_z\": "<<k_.pivot_c.z<<"\n}\n";return true;
}
}