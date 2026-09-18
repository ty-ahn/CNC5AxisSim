#pragma once
#include "Runtime.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <cctype>
namespace cnc {
enum class ExecutionState { Idle, Running, Held, Stopped, Completed, Error };
class ProgramExecutor {
 Runtime* runtime_{}; std::vector<Block> blocks_; size_t index_{0}; ExecutionState state_{ExecutionState::Idle}; std::string error_; std::unordered_map<int,size_t> labels_;
public:
 explicit ProgramExecutor(Runtime& r):runtime_(&r){}
 void load(const std::vector<Block>& b){blocks_=b;index_=0;error_.clear();labels_.clear();state_=ExecutionState::Idle; for(size_t i=0;i<blocks_.size();++i) if(blocks_[i].number>=0) labels_[blocks_[i].number]=i;}
 bool start(){if(!runtime_||blocks_.empty())return false;if(state_==ExecutionState::Completed||state_==ExecutionState::Stopped)index_=0;state_=ExecutionState::Running;return step();}
 bool step(){if(state_==ExecutionState::Held||state_==ExecutionState::Error||index_>=blocks_.size()){if(index_>=blocks_.size())state_=ExecutionState::Completed;return false;} if(!runtime_->execute(blocks_[index_],error_)){state_=runtime_->feed_hold()?ExecutionState::Held:ExecutionState::Error;return false;}
  const auto& src=blocks_[index_].source; std::string upper=src; for(char& ch:upper) ch=char(std::toupper((unsigned char)ch)); auto pos=upper.find("GOTOF"); if(pos==std::string::npos) pos=upper.find("GOTOB");
  if(pos!=std::string::npos){ size_t p=pos+5; while(p<src.size()&&src[p]==' ')++p; int n=std::atoi(src.c_str()+p); auto it=labels_.find(n); if(it==labels_.end()){error_="label not found";state_=ExecutionState::Error;return false;} index_=it->second; return true; } ++index_;if(index_>=blocks_.size())state_=ExecutionState::Completed;return true;}
 void hold(){runtime_->hold();state_=ExecutionState::Held;}
 bool resume(){if(state_!=ExecutionState::Held)return false;runtime_->resume();if(runtime_->feed_hold())return false;state_=ExecutionState::Running;return step();}
 void stop(){state_=ExecutionState::Stopped;}
 void reset(){runtime_->reset();index_=0;state_=ExecutionState::Idle;error_.clear();}
 size_t current_block()const{return index_;}
 ExecutionState state()const{return state_;}
 const std::string& error()const{return error_;}
};
}