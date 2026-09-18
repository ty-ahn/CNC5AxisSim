#pragma once
#include <string>
#include <vector>
namespace cnc {
struct Vec3 { double x{},y{},z{}; };
struct MachineState { double X{},Y{},Z{},A{},C{}; };
struct ToolPose { Vec3 tcp{}; Vec3 tool_axis{0,0,-1}; double tool_roll{}; };
struct AxisLimit { double minimum{}, maximum{}; bool wrap{}; };
struct MachineKinematicConfig {
 AxisLimit X{-500,500,false},Y{-500,500,false},Z{-500,200,false},A{-120,120,false},C{-360,360,true};
 Vec3 a_axis{1,0,0}, c_axis{0,0,1};
 Vec3 pivot_a{0,0,0}, pivot_c{0,0,0};
 double a_to_c{}, c_to_tool{};
};
struct IKResult { MachineState state{}; double position_error{},orientation_error{},cost{}; bool valid{}; };
class Kinematics {
 MachineKinematicConfig config_{}; bool swivel_active_{false}; ToolPose swivel_pose_{};
public:
 explicit Kinematics(MachineKinematicConfig c={}):config_(c){}
 const MachineKinematicConfig& config()const{return config_;}
 void set_config(const MachineKinematicConfig& c){config_=c;}
public:
 bool validate(const MachineState&,std::string&) const;
 static double unwrap(double previous,double target);
 static Vec3 axis_from_ac(double A,double C);
 Vec3 configured_axis_from_ac(double A,double C) const;
 static Vec3 tcp_from_machine(const MachineState&,double tool_length);
 bool inverse_kinematics(const ToolPose&,const MachineState&,double tool_length,IKResult&,std::string&);
 bool orientation_move(const ToolPose&,const MachineState&,double,std::vector<MachineState>&,std::string&,int samples=20);
 bool apply_swivel(double,double,double,double,double,ToolPose&,std::string&) const;
 bool cycle800_reset(std::string&);
 bool cycle800_swivel(double,double,std::string&);
 bool cycle800_active() const{return swivel_active_;}
};
}