#pragma once
#include "ProgramLoader.h"
#include "Runtime.h"
#include <unordered_map>
namespace cnc {
struct VerifyResult { bool passed{}; size_t executed{}; size_t errors{}; size_t collisions{}; size_t steps{}; std::string first_error; MachineState final_state{}; double remaining_volume{};\n size_t tool_stock_collisions{};\n size_t holder_stock_collisions{};\n size_t holder_machine_collisions{};\n double max_penetration{}; };
class VerifyEngine {
public:
 static VerifyResult run(const std::vector<Block>& blocks, Runtime runtime={});
 static VerifyResult run(const std::vector<Block>& blocks,const std::unordered_map<int,std::vector<Block>>& subprograms,Runtime runtime={});
 static bool save_json(const VerifyResult& result,const std::string& path,std::string& error);
};
}