#pragma once
#include "ProgramLoader.h"
#include "Runtime.h"
namespace cnc {
struct VerifyResult { bool passed{}; size_t executed{}; size_t errors{}; size_t collisions{}; std::string first_error; };
class VerifyEngine {
public:
 static VerifyResult run(const std::vector<Block>& blocks, Runtime runtime={});
};
}
