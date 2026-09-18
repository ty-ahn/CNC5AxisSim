#pragma once
#include "Kinematics.h"
#include <string>
#include <vector>
namespace cnc {
enum class MachineNodeType { LinearAxis, RotaryAxis, Tool, Holder, Fixture };
struct MachineNode {
 std::string name;
 MachineNodeType type{MachineNodeType::Fixture};
 std::string parent;
 std::string mesh;
 Vec3 axis{0,0,1};
 Vec3 pivot{};
 double min{}, max{};
 bool wrap{};\n Aabb collision_bounds{};\n bool collision_enabled{false};
};
class MachineConfig {
 MachineKinematicConfig k_{};
 std::string name_{"XYZAC_HEAD_HEAD"};
 std::vector<MachineNode> nodes_;
public:
 bool load_json(const std::string& path,std::string& error);
 bool save_json(const std::string& path,std::string& error) const;
 const MachineKinematicConfig& kinematics() const { return k_; }
 const std::string& name() const { return name_; }
 const std::vector<MachineNode>& nodes() const { return nodes_; }
};
}