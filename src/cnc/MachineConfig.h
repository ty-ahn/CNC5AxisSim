#pragma once
#include "Kinematics.h"
#include <string>
namespace cnc {
class MachineConfig {
 MachineKinematicConfig k_{};
 std::string name_{"XYZAC_HEAD_HEAD"};
public:
 bool load_json(const std::string& path,std::string& error);
 bool save_json(const std::string& path,std::string& error) const;
 const MachineKinematicConfig& kinematics() const { return k_; }
 const std::string& name() const { return name_; }
};
}