#include "Verify.h"
#include "ProgramExecutor.h"
namespace cnc {
VerifyResult VerifyEngine::run(const std::vector<Block>& blocks, Runtime runtime){
 static const std::unordered_map<int,std::vector<Block>> empty;
 return run(blocks,empty,std::move(runtime));
}
VerifyResult VerifyEngine::run(const std::vector<Block>& blocks,const std::unordered_map<int,std::vector<Block>>& subprograms,Runtime runtime){
 VerifyResult r{}; ProgramExecutor ex(runtime); ex.load(blocks);
 for(const auto& [n,b]:subprograms) ex.add_subprogram(n,b);
 if(blocks.empty()){r.passed=true;return r;}
 if(!ex.start()){r.errors=1;r.first_error=ex.error();return r;}
 while(ex.state()==ExecutionState::Running) if(!ex.step()) break;
 r.errors=(ex.state()==ExecutionState::Error)?1:0;
 if(r.errors) r.first_error=ex.error();
 r.collisions=runtime.collisions().events().size();
 r.executed=ex.current_block();
 if(ex.state()==ExecutionState::Completed) r.executed=blocks.size();
 r.passed=(ex.state()==ExecutionState::Completed&&r.errors==0&&r.collisions==0);
 return r;
}
}