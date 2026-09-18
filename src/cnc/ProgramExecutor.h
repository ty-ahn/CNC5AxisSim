#pragma once
#include "Runtime.h"
#include "Expression.h"
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <cctype>
namespace cnc {
enum class ExecutionState { Idle, Running, Held, Stopped, Completed, Error };
class ProgramExecutor {
 Runtime* runtime_{}; std::vector<Block> blocks_; std::vector<Block> main_blocks_; size_t index_{0}; ExecutionState state_{ExecutionState::Idle}; std::string error_; std::unordered_map<int,size_t> labels_; std::unordered_map<int,std::vector<Block>> subprograms_; struct Frame{int program{}; size_t return_index{};}; std::vector<Frame> stack_; size_t steps_{0}; size_t max_steps_{1000000}; int program_{0};
public:
 explicit ProgramExecutor(Runtime& r):runtime_(&r){}
 void load(const std::vector<Block>& b){blocks_=b; main_blocks_=b; subprograms_.clear();stack_.clear();program_=0;index_=0;steps_=0;error_.clear();labels_.clear();state_=ExecutionState::Idle; for(size_t i=0;i<blocks_.size();++i) if(blocks_[i].number>=0) labels_[blocks_[i].number]=i;}
 void add_subprogram(int number,const std::vector<Block>& b){subprograms_[number]=b;}\n void set_max_steps(size_t n){max_steps_=n;}\n bool start(){if(!runtime_||blocks_.empty())return false;if(state_==ExecutionState::Completed||state_==ExecutionState::Stopped)index_=0;state_=ExecutionState::Running;return step();}
 bool step(){if(++steps_>max_steps_){error_="execution step limit exceeded";state_=ExecutionState::Error;return false;} if(state_==ExecutionState::Held||state_==ExecutionState::Error||index_>=blocks_.size()){if(index_>=blocks_.size())state_=ExecutionState::Completed;return false;} if(!runtime_->execute(blocks_[index_],error_)){state_=runtime_->feed_hold()?ExecutionState::Held:ExecutionState::Error;return false;}
  const auto& executed=blocks_[index_].source; std::string exu=executed; for(char& ch:exu) ch=char(std::toupper((unsigned char)ch));
  if(exu.find(" RET")!=std::string::npos || exu.rfind("RET",0)==0){ if(stack_.empty()){error_="RET without CALL";state_=ExecutionState::Error;return false;} auto fr=stack_.back(); stack_.pop_back(); if(fr.program==0){program_=0; blocks_=main_blocks_;} else {program_=fr.program;} labels_.clear(); for(size_t i=0;i<blocks_.size();++i) if(blocks_[i].number>=0) labels_[blocks_[i].number]=i; index_=fr.return_index; return true; }
  auto cp=exu.find("CALL"); if(cp!=std::string::npos){ size_t p=cp+4; while(p<executed.size()&&std::isspace((unsigned char)executed[p]))++p; if(p<executed.size()&&executed[p]=='P')++p; int pn=std::atoi(executed.c_str()+p); auto si=subprograms_.find(pn); if(si==subprograms_.end()){error_="subprogram not found";state_=ExecutionState::Error;return false;} stack_.push_back({program_,index_+1}); blocks_=si->second; program_=pn; index_=0; labels_.clear(); for(size_t i=0;i<blocks_.size();++i) if(blocks_[i].number>=0) labels_[blocks_[i].number]=i; return true; }
  const auto& src=blocks_[index_].source; std::string upper=src; for(char& ch:upper) ch=char(std::toupper((unsigned char)ch));
  auto ifpos=upper.find("IF"); if(ifpos!=std::string::npos){ auto gp=upper.find("GOTOF",ifpos); if(gp==std::string::npos) gp=upper.find("GOTOB",ifpos); if(gp!=std::string::npos){ std::string cond=src.substr(ifpos+2,gp-ifpos-2); double cv=0; if(!Expression::evaluate(cond,runtime_->variables(),cv,error_)){state_=ExecutionState::Error;return false;} if(cv==0){++index_;return true;} size_t p=gp+5; while(p<src.size()&&std::isspace((unsigned char)src[p]))++p; int n=std::atoi(src.c_str()+p); auto it=labels_.find(n); if(it==labels_.end()){error_="label not found";state_=ExecutionState::Error;return false;} index_=it->second; return true; }}
  auto pos=upper.find("GOTOF"); if(pos==std::string::npos) pos=upper.find("GOTOB");
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