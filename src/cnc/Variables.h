#pragma once
#include <string>
#include <unordered_map>
namespace cnc {
class VariableStore {
 std::unordered_map<int,double> r_;
public:
 double get(int n) const { auto i=r_.find(n); return i==r_.end()?0.0:i->second; }
 void set(int n,double v){r_[n]=v;}
 bool has(int n) const {return r_.find(n)!=r_.end();}
 void clear(){r_.clear();}
};
}