#include "Expression.h"
#include <cctype>
#include <cmath>
#include <cstdlib>
namespace cnc {
class ExprParser {
 const std::string&s; const VariableStore&v; size_t p=0; std::string&e;
 void ws(){while(p<s.size()&&std::isspace((unsigned char)s[p]))++p;}
 bool eat(char c){ws();if(p<s.size()&&s[p]==c){++p;return true;}return false;}
 bool factor(double&o){ws(); if(eat('+'))return factor(o); if(eat('-')){if(!factor(o))return false;o=-o;return true;}
  if(eat('(')){if(!expr(o)||!eat(')')){e="bad expression";return false;}return true;}
  if(p<s.size()&&(s[p]=='R'||s[p]=='r')){++p;char*b=nullptr;long n=std::strtol(s.c_str()+p,&b,10);if(b==s.c_str()+p){e="bad R parameter";return false;}p=(size_t)(b-s.c_str());o=v.get((int)n);return true;}
  char*b=nullptr; o=std::strtod(s.c_str()+p,&b); if(b==s.c_str()+p){e="number expected";return false;}p=(size_t)(b-s.c_str());return true;
 }
 bool term(double&o){if(!factor(o))return false;for(;;){if(eat('*')){double x;if(!factor(x))return false;o*=x;}else if(eat('/')){double x;if(!factor(x)||std::abs(x)<1e-15){e="division by zero";return false;}o/=x;}else return true;}}
 bool expr(double&o){if(!term(o))return false;for(;;){if(eat('+')){double x;if(!term(x))return false;o+=x;}else if(eat('-')){double x;if(!term(x))return false;o-=x;}else return true;}}
public: ExprParser(const std::string&a,const VariableStore&b,std::string&c):s(a),v(b),e(c){}
 bool run(double&o){if(!expr(o))return false;ws();if(p!=s.size()){e="unexpected token";return false;}return true;}
};
bool Expression::evaluate(const std::string&t,const VariableStore&v,double&o,std::string&e){return ExprParser(t,v,e).run(o);}
}