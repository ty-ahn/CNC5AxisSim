#include "Verify.h"
#include "ProgramExecutor.h"
namespace cnc {
VerifyResult VerifyEngine::run(const std::vector<Block>& blocks, Runtime runtime){
 VerifyResult r{}; ProgramExecutor ex(runtime); ex.load(blocks);
 if(blocks.empty()){r.passed=true;return r;}
 if(!ex.start()){r.errors=1;r.first_error=ex.error();return r;}
 while(ex.state()==ExecutionState::Running){if(!ex.step())break;}
 r.executed=blocks.size();
 if(ex.state()==ExecutionState::Error){r.errors=1;r.first_error=ex.error();}
 r.collisions=runtime.collisions().events().size();
 r.passed=(ex.state()==ExecutionState::Completed&&r.errors==0&&r.collisions==0);
 return r;
}
}