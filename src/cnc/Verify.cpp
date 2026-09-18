#include "Verify.h"
namespace cnc {
VerifyResult VerifyEngine::run(const std::vector<Block>& blocks, Runtime runtime){
 VerifyResult r{}; std::string e;
 for(const auto& b:blocks){if(!runtime.execute(b,e)){++r.errors;if(r.first_error.empty())r.first_error=e;break;}++r.executed;}
 r.collisions=runtime.collisions().events().size(); r.passed=(r.errors==0&&r.collisions==0&&r.executed==blocks.size()); return r;
}
}
