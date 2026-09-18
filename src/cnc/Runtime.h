#pragma once
#include "ArcPlanner.h"
#include "Parser.h"
#include "Tooling.h"
#include "Stock.h"
#include "Collision.h"
#include "Variables.h"
#include "Kinematics.h"
#include "MachineConfig.h"
#include "MachineGeometry.h"
#include "Bvh.h"
#include <string>
#include <vector>
namespace cnc {
enum class MotionMode{Rapid,Linear,ArcCW,ArcCCW};
struct ModalState{MotionMode motion{MotionMode::Rapid};bool absolute{true};int work_offset{54};int plane{17};int feed_mode{94};bool coolant{false}; bool traori{false}; int cycle{0}; int cutter_comp{0}; int length_comp{0}; int spindle_direction{0}; int retract_mode{98}; double cycle_r{}; double cycle_z{}; double cycle_feed{}; bool cycle_has_r{false}; bool cycle_has_z{false};};
class Runtime{
 MachineState state_{};ModalState modal_{};double feed_{},rpm_{};int tool_{},d_{}; ToolTable tools_; VariableStore variables_; Stock stock_; bool stock_enabled_{false};std::vector<MotionPoint> last_motion_; bool feed_hold_{false}; std::string alarm_; CollisionManager collisions_; Kinematics kinematics_; std::vector<MachineGeometry> machine_geometry_; std::vector<Bvh> machine_bvh_;
public:bool execute(const Block&,std::string&);const MachineState& state()const{return state_;}const ModalState& modal()const{return modal_;}const std::vector<MotionPoint>& last_motion()const{return last_motion_;}double feed()const{return feed_;}double rpm()const{return rpm_;}int tool()const{return tool_;}int d()const{return d_;} const ToolTable& tool_table()const{return tools_;} void set_tool(const ToolDefinition&t){tools_.set(t);} void set_stock(const StockDefinition&s){stock_enabled_=stock_.initialize(s);} const Stock& stock()const{return stock_;} bool feed_hold()const{return feed_hold_;} void set_work_offset(int g){if(g==54||g==55)modal_.work_offset=g;} void hold(){feed_hold_=true;} void resume(){if(alarm_.empty())feed_hold_=false;} const std::string& alarm()const{return alarm_;} const CollisionManager& collisions()const{return collisions_;} const VariableStore& variables() const{return variables_;} const Kinematics& kinematics() const{return kinematics_;} void set_machine_config(const MachineKinematicConfig& c){kinematics_.set_config(c);} bool load_machine_config(const std::string& path,std::string& e){MachineConfig c;if(!c.load_json(path,e))return false;kinematics_.set_config(c.kinematics());collisions_.clear_machine_bodies();machine_geometry_.clear();machine_bvh_.clear();for(const auto& n:c.nodes())if(n.collision_enabled){machine_geometry_.emplace_back();auto& g=machine_geometry_.back();if(!n.mesh.empty()){std::string ge;if(!g.load_stl(n.mesh,ge)){e="machine mesh "+n.name+": "+ge;return false;}machine_bvh_.emplace_back();machine_bvh_.back().build(g.triangles());collisions_.add_machine_body({n.name,n.collision_bounds,&g.triangles(),&machine_bvh_.back()});}else collisions_.add_machine_body({n.name,n.collision_bounds,nullptr,nullptr});}return true;} VariableStore& variables(){return variables_;} void clear_alarm(){alarm_.clear();feed_hold_=false;}void reset();
};
}