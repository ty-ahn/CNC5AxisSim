#include "MachineConfig.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
namespace cnc {
static bool num(const std::string&s,const std::string& key,double&v){std::regex r("\""+key+"\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");std::smatch m;if(!std::regex_search(s,m,r))return false;v=std::stod(m[1]);return true;}
bool MachineConfig::load_json(const std::string& path,std::string& e){
 std::ifstream f(path);if(!f){e="cannot open machine config";return false;}std::ostringstream ss;ss<<f.rdbuf();auto s=ss.str();
 auto get=[&](const char*k,AxisLimit&a){double lo,hi;if(!num(s,std::string(k)+"_min",lo)||!num(s,std::string(k)+"_max",hi))return false;if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi)return false;a.minimum=lo;a.maximum=hi;return true;};
 if(!get("X",k_.X)||!get("Y",k_.Y)||!get("Z",k_.Z)||!get("A",k_.A)||!get("C",k_.C)){e="machine axis limits missing";return false;}
 double v;if(num(s,"a_to_c",v))k_.a_to_c=v;if(num(s,"c_to_tool",v))k_.c_to_tool=v; double ax,ay,az,cx,cy,cz;
 if(num(s,"A_axis_x",ax)&&num(s,"A_axis_y",ay)&&num(s,"A_axis_z",az)){k_.a_axis={ax,ay,az};}
 if(num(s,"C_axis_x",cx)&&num(s,"C_axis_y",cy)&&num(s,"C_axis_z",cz)){k_.c_axis={cx,cy,cz};}
 double px,py,pz;
 if(num(s,"A_pivot_x",px)&&num(s,"A_pivot_y",py)&&num(s,"A_pivot_z",pz))k_.pivot_a={px,py,pz};
 if(num(s,"C_pivot_x",px)&&num(s,"C_pivot_y",py)&&num(s,"C_pivot_z",pz))k_.pivot_c={px,py,pz};
 std::regex nr("\"name\"\\s*:\\s*\"([^\"]+)\"");std::smatch m;if(std::regex_search(s,m,nr))name_=m[1];
 return true;
}
bool MachineConfig::save_json(const std::string& path,std::string&e)const{
 std::ofstream f(path);if(!f){e="cannot write machine config";return false;}
 f<<"{\n  \"name\": \""<<name_<<"\",\n  \"X_min\": "<<k_.X.minimum<<", \"X_max\": "<<k_.X.maximum<<",\n  \"Y_min\": "<<k_.Y.minimum<<", \"Y_max\": "<<k_.Y.maximum<<",\n  \"Z_min\": "<<k_.Z.minimum<<", \"Z_max\": "<<k_.Z.maximum<<",\n  \"A_min\": "<<k_.A.minimum<<", \"A_max\": "<<k_.A.maximum<<",\n  \"C_min\": "<<k_.C.minimum<<", \"C_max\": "<<k_.C.maximum<<",\n  \"a_to_c\": "<<k_.a_to_c<<", \"c_to_tool\": "<<k_.c_to_tool<<"\n}\n";return true;
}
}