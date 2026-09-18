#pragma once
#include "Stock.h"
#include <vector>
#include <string>
namespace cnc {
struct CollisionRecord {
 int block_number{-1};
 CollisionEvent event{};
 std::string message;
};
class CollisionManager {
 std::vector<CollisionRecord> events_;
public:
 void clear(){events_.clear();}
 void record(int block,const CollisionEvent& e,const std::string& msg){if(e.collision)events_.push_back({block,e,msg});}
 const std::vector<CollisionRecord>& events()const{return events_;}
 bool has_collision()const{return !events_.empty();}
};
}