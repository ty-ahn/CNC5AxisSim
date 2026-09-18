#pragma once
#include <unordered_map>
namespace cnc {
struct ToolDefinition { int number{}; double length{}; double diameter{}; double corner_radius{}; };
class ToolTable {
 std::unordered_map<int,ToolDefinition> tools_;
public:
 void set(const ToolDefinition& t){tools_[t.number]=t;}
 bool get(int n,ToolDefinition& t) const {auto i=tools_.find(n);if(i==tools_.end())return false;t=i->second;return true;}
};
}