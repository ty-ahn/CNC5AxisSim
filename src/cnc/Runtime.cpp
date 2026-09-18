#include "Runtime.h"
namespace cnc {
bool Runtime::execute(const Block& b,std::string& error){
    auto next=state_;
    for(const auto&w:b.words){
        switch(w.letter){
        case 'G': if(static_cast<int>(w.value)==90) absolute_=true; else if(static_cast<int>(w.value)==91) absolute_=false; break;
        case 'X': next.X=absolute_?w.value:next.X+w.value; break;
        case 'Y': next.Y=absolute_?w.value:next.Y+w.value; break;
        case 'Z': next.Z=absolute_?w.value:next.Z+w.value; break;
        case 'A': next.A=absolute_?w.value:next.A+w.value; break;
        case 'C': next.C=absolute_?w.value:next.C+w.value; break;
        case 'F': feed_=w.value; break;
        case 'S': rpm_=w.value; break;
        case 'T': tool_=static_cast<int>(w.value); break;
        case 'D': d_=static_cast<int>(w.value); break;
        case 'M': if(static_cast<int>(w.value)==5) rpm_=0; break;
        default: break;
        }
    }
    if(!Kinematics().validate(next,error)) return false;
    state_=next;
    return true;
}
void Runtime::reset(){state_={};absolute_=true;feed_=rpm_=0;tool_=d_=0;}
}