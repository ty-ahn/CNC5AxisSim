#include "MachineConfig.h"
#include "MachineJson.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
#include <algorithm>
namespace cnc {
static bool num(const std::string&s,const std::string& key,double&v){std::regex r("\""+key+"\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");std::smatch m;if(!std::regex_search(s,m,r))return false;v=std::stod(m[1]);return true;}
bool MachineConfig::load_json(const std::string& path,std::string& e){
 std::ifstream f(path);
 if(!f){e="cannot open machine config";return false;}
 std::ostringstream ss; ss<<f.rdbuf(); std::string raw=ss.str();
 JsonValue root;
 if(parse_json(raw,root,e)){
  if(const auto* m=root.get("machine");m&&m->object()) root=*m;
  if(const auto* n=root.get("name");n&&n->str()) name_=*n->str();
  if(const auto* a=root.get("nodes");a&&a->arr()){
   nodes_.clear();
   for(const auto& v:*a->arr()){
    MachineNode n;
    if(const auto* x=v.get("name");x&&x->str()) n.name=*x->str(); else {e="machine node name missing";return false;}
    if(const auto* x=v.get("type");x&&x->str()){
     const std::string t=*x->str();
     if(t=="linear") n.type=MachineNodeType::LinearAxis;
     else if(t=="rotary") n.type=MachineNodeType::RotaryAxis;
     else if(t=="tool") n.type=MachineNodeType::Tool;
     else if(t=="holder") n.type=MachineNodeType::Holder;
     else n.type=MachineNodeType::Fixture;
    }
    if(const auto* x=v.get("parent");x&&x->str()) n.parent=*x->str();
    if(const auto* x=v.get("mesh");x&&x->str()) n.mesh=*x->str();
    if(const auto* x=v.get("axis");x&&x->arr()&&x->arr()->size()==3){
     const auto& a=*x->arr(); if(a[0].number()&&a[1].number()&&a[2].number()) n.axis={*a[0].number(),*a[1].number(),*a[2].number()};
    }
    if(const auto* x=v.get("pivot");x&&x->arr()&&x->arr()->size()==3){
     const auto& a=*x->arr(); if(a[0].number()&&a[1].number()&&a[2].number()) n.pivot={*a[0].number(),*a[1].number(),*a[2].number()};
    }
    if(const auto* x=v.get("min");x&&x->number()) n.min=*x->number();
    if(const auto* x=v.get("max");x&&x->number()) n.max=*x->number();
    if(const auto* x=v.get("limit");x&&x->object()){
     if(const auto* q=x->get("min");q&&q->number()) n.min=*q->number();
     if(const auto* q=x->get("max");q&&q->number()) n.max=*q->number();
     if(const auto* q=x->get("wrap");q&&std::holds_alternative<bool>(q->data)) n.wrap=std::get<bool>(q->data);
    }
    if(const auto* x=v.get("wrap");x&&std::holds_alternative<bool>(x->data)) n.wrap=std::get<bool>(x->data);
    if(const auto* x=v.get("collision");x){
     if(std::holds_alternative<bool>(x->data)) n.collision_enabled=std::get<bool>(x->data);
     else if(x->object()){
      n.collision_enabled=true;
      if(const auto* q=x->get("min");q&&q->arr()&&q->arr()->size()==3){
       const auto& a=*q->arr(); if(a[0].number()&&a[1].number()&&a[2].number()) n.collision_bounds.min={*a[0].number(),*a[1].number(),*a[2].number()};
      }
      if(const auto* q=x->get("max");q&&q->arr()&&q->arr()->size()==3){
       const auto& a=*q->arr(); if(a[0].number()&&a[1].number()&&a[2].number()) n.collision_bounds.max={*a[0].number(),*a[1].number(),*a[2].number()};
      }
     }
    }
    nodes_.push_back(std::move(n));
   }
   for(const auto& n:nodes_){
    if(n.type!=MachineNodeType::LinearAxis&&n.type!=MachineNodeType::RotaryAxis) continue;
    if(n.name=="X") k_.X={n.min,n.max,n.wrap};
    else if(n.name=="Y") k_.Y={n.min,n.max,n.wrap};
    else if(n.name=="Z") k_.Z={n.min,n.max,n.wrap};
    else if(n.name=="A"){k_.A={n.min,n.max,n.wrap};k_.a_axis=n.axis;k_.pivot_a=n.pivot;}
    else if(n.name=="C"){k_.C={n.min,n.max,n.wrap};k_.c_axis=n.axis;k_.pivot_c=n.pivot;}
   }
   return validate_tree(e);
  }
 }
 double v;
 auto get=[&](const char*k,AxisLimit&a){double lo,hi;if(!num(raw,std::string(k)+"_min",lo)||!num(raw,std::string(k)+"_max",hi))return false;if(!std::isfinite(lo)||!std::isfinite(hi)||lo>=hi)return false;a={lo,hi,false};return true;};
 if(!get("X",k_.X)||!get("Y",k_.Y)||!get("Z",k_.Z)||!get("A",k_.A)||!get("C",k_.C)){e="machine axis limits missing";return false;}
 if(num(raw,"a_to_c",v)) k_.a_to_c=v;
 if(num(raw,"c_to_tool",v)) k_.c_to_tool=v;
 double ax,ay,az,cx,cy,cz;
 if(num(raw,"A_axis_x",ax)&&num(raw,"A_axis_y",ay)&&num(raw,"A_axis_z",az)) k_.a_axis={ax,ay,az};
 if(num(raw,"C_axis_x",cx)&&num(raw,"C_axis_y",cy)&&num(raw,"C_axis_z",cz)) k_.c_axis={cx,cy,cz};
 double px,py,pz;
 if(num(raw,"A_pivot_x",px)&&num(raw,"A_pivot_y",py)&&num(raw,"A_pivot_z",pz)) k_.pivot_a={px,py,pz};
 if(num(raw,"C_pivot_x",px)&&num(raw,"C_pivot_y",py)&&num(raw,"C_pivot_z",pz)) k_.pivot_c={px,py,pz};
 nodes_.clear(); const char* keys[]={"X","Y","Z","A","C"};
 for(int i=0;i<5;++i){MachineNode n;n.name=keys[i];n.type=i<3?MachineNodeType::LinearAxis:MachineNodeType::RotaryAxis;n.min=i==0?k_.X.minimum:i==1?k_.Y.minimum:i==2?k_.Z.minimum:i==3?k_.A.minimum:k_.C.minimum;n.max=i==0?k_.X.maximum:i==1?k_.Y.maximum:i==2?k_.Z.maximum:i==3?k_.A.maximum:k_.C.maximum;n.wrap=i==4;n.parent=i==0?"":i==1?"X":i==2?"Y":i==3?"Z":"A";n.axis=i==3?k_.a_axis:k_.c_axis;n.pivot=i==3?k_.pivot_a:k_.pivot_c;nodes_.push_back(n);}
 return validate_tree(e);
}
bool MachineConfig::save_json(const std::string& path,std::string&e)const{
 std::ofstream f(path);if(!f){e="cannot write machine config";return false;}
 f<<"{\n  \"machine\": {\n    \"name\": \""<<json_escape(name_)<<"\",\n    \"nodes\": [\n";
 for(size_t i=0;i<nodes_.size();++i){const auto&n=nodes_[i];const char*t=n.type==MachineNodeType::LinearAxis?"linear":n.type==MachineNodeType::RotaryAxis?"rotary":n.type==MachineNodeType::Tool?"tool":n.type==MachineNodeType::Holder?"holder":"fixture";
  f<<"      {\"name\": \""<<json_escape(n.name)<<"\", \"type\": \""<<t<<"\", \"parent\": "<<(n.parent.empty()?"null":"\""+json_escape(n.parent)+"\"")<<", \"axis\": ["<<n.axis.x<<", "<<n.axis.y<<", "<<n.axis.z<<"], \"pivot\": ["<<n.pivot.x<<", "<<n.pivot.y<<", "<<n.pivot.z<<"], \"limit\": {\"min\": "<<n.min<<", \"max\": "<<n.max<<", \"wrap\": "<<(n.wrap?"true":"false")<<"}, \"mesh\": \""<<json_escape(n.mesh)<<"\", \"collision\": "<<(n.collision_enabled?"true":"false")<<"}"<<(i+1<nodes_.size()?",":"")<<"\n";
 }
 f<<"    ]\n  }\n}\n";return true;
}
}