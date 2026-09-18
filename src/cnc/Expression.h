#pragma once
#include "Variables.h"
#include <string>
namespace cnc {
class Expression {
public:
 static bool evaluate(const std::string& text,const VariableStore& vars,double& value,std::string& error);
};
}