#include "Verify.h"
#include "ProgramExecutor.h"
#include <fstream>
#include <iomanip>
#include <algorithm>
namespace cnc {
VerifyResult VerifyEngine::run(const std::vector<Block>& blocks, Runtime runtime){ static const std::unordered_map<int,std::vector<Block>> empty; return run(blocks,empty,runtime); }
VerifyResult VerifyEngine::run(const std::vector<Block>& blocks,const std::unordered_map<int,std::vector<Block>>& subprograms,Runtime runtime){
 VerifyResult r{}; ProgramExecutor ex(runtime); ex.load(blocks); for(const auto& [n,b]:subprograms) ex.add_subprogram(n,b);
 if(blocks.empty()){r.passed=true;return r;} if(!ex.start()){r.errors=1;r.first_error=ex.error();return r;}
 while(ex.state()==ExecutionState::Running) if(!ex.step()) break;
 r.errors=(ex.state()==ExecutionState::Error)?1:0; if(r.errors)r.first_error=ex.error(); r.collisions=runtime.collisions().events().size();
 r.tool_stock_collisions=runtime.collisions().count(CollisionType::ToolStock);
 r.holder_stock_collisions=runtime.collisions().count(CollisionType::HolderStock);
 r.holder_machine_collisions=runtime.collisions().count(CollisionType::HolderMachine);
 r.max_penetration=std::max({runtime.collisions().max_penetration(CollisionType::ToolStock),runtime.collisions().max_penetration(CollisionType::HolderStock),runtime.collisions().max_penetration(CollisionType::HolderMachine)}); r.steps=ex.steps(); r.executed=ex.current_block(); if(ex.state()==ExecutionState::Completed)r.executed=blocks.size();
 r.final_state=runtime.state(); r.remaining_volume=runtime.stock().remaining_volume(); r.passed=(ex.state()==ExecutionState::Completed&&r.errors==0&&r.collisions==0); return r;
}
bool VerifyEngine::save_json(const VerifyResult&r,const std::string&path,std::string&e){
 std::ofstream f(path);
 if(!f){e="cannot write verify report";return false;}
 f<<std::setprecision(15);
 f<<"{\n"
  <<"  \"passed\": "<<(r.passed?"true":"false")<<",\n"
  <<"  \"executed\": "<<r.executed<<",\n"
  <<"  \"errors\": "<<r.errors<<",\n"
  <<"  \"collisions\": "<<r.collisions<<",\n"
  <<"  \"steps\": "<<r.steps<<",\n"
  <<"  \"first_error\": \"";
 for(char ch:r.first_error){if(ch=='\\'||ch=='\"')f<<'\\'; if(ch=='\n')f<<"\\n"; else if(ch=='\r')f<<"\\r"; else if(ch=='\t')f<<"\\t"; else f<<ch;}
 f<<"\",\n"
  <<"  \"final_state\": {\"X\": "<<r.final_state.X<<", \"Y\": "<<r.final_state.Y<<", \"Z\": "<<r.final_state.Z<<", \"A\": "<<r.final_state.A<<", \"C\": "<<r.final_state.C<<"},\n"
  <<"  \"remaining_volume\": "<<r.remaining_volume<<"\n"
  <<"}\n";
 return true;
}
