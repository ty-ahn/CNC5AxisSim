#pragma once
#include "ProgramExecutor.h"
namespace cnc {
class OperatorController {
 ProgramExecutor executor_;
public:
 explicit OperatorController(Runtime& r):executor_(r){}
 bool load(const std::vector<Block>& b){executor_.load(b);return true;} void add_subprogram(int number,const std::vector<Block>& b){executor_.add_subprogram(number,b);} void set_max_steps(size_t n){executor_.set_max_steps(n);}
 bool start(){return executor_.start();}
 bool step(){return executor_.step();}
 void hold(){executor_.hold();}
 bool resume(){return executor_.resume();}
 void stop(){executor_.stop();}
 void reset(){executor_.reset();}
 const ProgramExecutor& executor()const{return executor_;}
};
}