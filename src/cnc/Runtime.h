#pragma once
#include "ArcPlanner.h"
#include "Parser.h"
#include "Tooling.h"
#include <string>
#include <vector>
namespace cnc {
enum class MotionMode{Rapid,Linear,ArcCW,ArcCCW};
struct ModalState{MotionMode motion{MotionMode::Rapid};bool absolute{true};int work_offset{54};int plane{17};int feed_mode{94};bool coolant{false}; bool traori{false};int spindle_direction{0};};
class Runtime{
 MachineState state_{};ModalState modal_{};double feed_{},rpm_{};int tool_{},d_{}; ToolTable tools_;std::vector<MotionPoint> last_motion_;
public:bool execute(const Block&,std::string&);const MachineState& state()const{return state_;}const ModalState& modal()const{return modal_;}const std::vector<MotionPoint>& last_motion()const{return last_motion_;}double feed()const{return feed_;}double rpm()const{return rpm_;}int tool()const{return tool_;}int d()const{return d_;} const ToolTable& tool_table()const{return tools_;} void set_tool(const ToolDefinition&t){tools_.set(t);}void reset();
};
}