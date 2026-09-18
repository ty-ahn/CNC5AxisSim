#pragma once
#include <unordered_map>
#include <string>
#include <cstddef>
namespace cnc {
struct ToolDefinition {
 int number{};
 double length{};
 double diameter{};
 double corner_radius{};
 double holder_diameter{20.0};
 double holder_length{80.0};
};
class ToolTable {
 std::unordered_map<int,ToolDefinition> tools_;
public:
 void set(const ToolDefinition& t){tools_[t.number]=t;}
 bool get(int n,ToolDefinition& t) const {auto i=tools_.find(n);if(i==tools_.end())return false;t=i->second;return true;}
 size_t size() const{return tools_.size();}
};
}