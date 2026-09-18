#include "Runtime.h"
#include <cmath>
#include <algorithm>
#include <cctype>
namespace cnc{
static int code(const Word&w){return static_cast<int>(std::llround(w.value));}
bool Runtime::execute(const Block&b,std::string&e){
 auto next=state_;auto m=modal_;  std::string src=b.source; std::transform(src.begin(),src.end(),src.begin(),[](unsigned char c){return char(std::toupper(c));});  if(src.find("TRAORI")!=std::string::npos) m.traori=true;  if(src.find("TRAFOOF")!=std::string::npos) m.traori=false;double f=feed_,s=rpm_;int t=tool_,d=d_;bool move=false;ArcDefinition arc{}; Vec3 tool_axis{}; bool has_tool_axis=false;
 for(const auto&w:b.words)switch(w.letter){
 case'G':switch(code(w)){case 0:m.motion=MotionMode::Rapid;break;case 1:m.motion=MotionMode::Linear;break;case 2:m.motion=MotionMode::ArcCW;break;case 3:m.motion=MotionMode::ArcCCW;break;case 17:m.plane=17;break;case 18:m.plane=18;break;case 19:m.plane=19;break;case 54:m.work_offset=54;break;case 55:m.work_offset=55;break;case 90:m.absolute=true;break;case 91:m.absolute=false;break;case 94:m.feed_mode=94;break;case 95:m.feed_mode=95;break;default:break;}break;
 case'X':next.X=m.absolute?w.value:next.X+w.value;move=true;break;case'Y':next.Y=m.absolute?w.value:next.Y+w.value;move=true;break;case'Z':next.Z=m.absolute?w.value:next.Z+w.value;move=true;break;case'A':next.A=m.absolute?w.value:next.A+w.value;move=true;break;case'C':next.C=m.absolute?w.value:next.C+w.value;move=true;break;
 case'I':arc.I=w.value;tool_axis.x=w.value;has_tool_axis=true;break;case'J':arc.J=w.value;tool_axis.y=w.value;has_tool_axis=true;break;case'K':arc.K=w.value;tool_axis.z=w.value;has_tool_axis=true;break;
 case'F':if(w.value<0){e="Negative feed is invalid";return false;}f=w.value;break;case'S':if(w.value<0){e="Negative spindle speed is invalid";return false;}s=w.value;break;case'T':t=code(w);break;case'D':d=code(w);break;
 case'M':switch(code(w)){case 3:m.spindle_direction=1;break;case 4:m.spindle_direction=-1;break;case 5:m.spindle_direction=0;s=0;break;case 8:m.coolant=true;break;case 9:m.coolant=false;break;default:break;}break;default:break;}
 if(move && m.traori && has_tool_axis && m.motion!=MotionMode::ArcCW && m.motion!=MotionMode::ArcCCW){ ToolPose pose{{next.X,next.Y,next.Z},tool_axis}; IKResult ik; if(!Kinematics::inverse_kinematics(pose,state_,100.0,ik,e)) return false; next.A=ik.state.A; next.C=ik.state.C; } if(move){MotionPoint a{state_.X,state_.Y,state_.Z,state_.A,state_.C},z{next.X,next.Y,next.Z,next.A,next.C};bool ok;if(m.motion==MotionMode::ArcCW||m.motion==MotionMode::ArcCCW){if(m.plane!=17){e="G2/G3 currently supports G17 XY plane only";return false;}ok=ArcPlanner::plan(a,z,arc,m.motion==MotionMode::ArcCW,f,last_motion_,e);}else ok=MotionPlanner::plan(a,z,m.motion==MotionMode::Rapid,f,last_motion_,e);if(!ok)return false;}else{if(!Kinematics().validate(next,e))return false;last_motion_.clear();}
 state_=next;modal_=m;feed_=f;rpm_=s;tool_=t;d_=d;return true;
}
void Runtime::reset(){state_={};modal_={};feed_=rpm_=0;tool_=d_=0;last_motion_.clear();}
}