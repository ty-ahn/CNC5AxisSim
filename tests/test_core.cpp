#include "cnc/Runtime.h"
#include "cnc/Kinematics.h"
#include "cnc/ProgramLoader.h"
#include "cnc/Verify.h"
#include "cnc/ProgramExecutor.h"
#include <cassert>
#include <cmath>
int main(){
 cnc::Runtime r; std::string e;
 r.set_tool({1,250.0,20.0,2.0});
 auto run=[&](const char* src){auto b=cnc::Parser::parse(src);assert(b);assert(r.execute(*b,e));};
 run("N10 G90 G54 G0 X0 Y0 Z100");
 run("N20 T1 D1 S8000 M3"); assert(r.tool()==1);
 run("N30 G1 X100 Y50 Z20 A30 C45 F1000"); assert(r.last_motion().size()==21);
 run("N40 G91 X5 C20"); assert(std::abs(r.state().X-105)<1e-9&&std::abs(r.state().C-65)<1e-9);
 run("N50 G90 G0 X0 Y0 Z0");
 run("N60 G17 G2 X20 Y0 I10 J0 Z-10"); assert(r.last_motion().size()==33);
 run("N70 G3 X0 Y0 I-10 J0"); assert(std::abs(r.state().X)<1e-9&&std::abs(r.state().Y)<1e-9);
 run("N80 TRAORI G90 G1 X10 Y0 Z0 I0 J0 K-1 F500"); assert(r.modal().traori); assert(std::abs(r.state().A)<1e-9);
 run("N85 TRAFOOF"); assert(!r.modal().traori);
 run("N90 G55 M5"); run("N91 G93 G1 X10 F2"); assert(r.modal().feed_mode==93&&std::abs(r.feed()-2)<1e-9); run("N100 G81 X20 Y20 Z-10 R5 F100"); assert(std::abs(r.state().Z)<1e-9&&r.last_motion().size()>1); run("N105 G99 G81 X30 Y30 Z-10 R5 F100"); assert(std::abs(r.state().Z-5)<1e-9); run("N106 X40 Y40"); assert(std::abs(r.state().Z-5)<1e-9); run("N110 G80"); assert(r.modal().work_offset==55&&r.rpm()==0);
 assert(std::abs(cnc::Kinematics::unwrap(359,1)-361)<1e-9); cnc::Kinematics kk({{-100,100,false},{-100,100,false},{-100,100,false},{-90,90,false},{-180,180,true},10,20}); cnc::MachineState ks{0,0,0,30,40}; auto tcp=kk.tcp_from_machine(ks,50); cnc::ToolPose kp{tcp,cnc::Kinematics::axis_from_ac(30,40)}; cnc::IKResult ki; assert(kk.inverse_kinematics(kp,ks,50,ki,e)); assert(std::abs(ki.position_error)<1e-7); cnc::ToolPose target{tcp,cnc::Kinematics::axis_from_ac(70,120)}; std::vector<cnc::MachineState> path; assert(kk.orientation_move(target,ks,50,path,e,12)); assert(path.size()==13); for(const auto& q:path){auto p=kk.tcp_from_machine(q,50); assert(std::sqrt((p.x-tcp.x)*(p.x-tcp.x)+(p.y-tcp.y)*(p.y-tcp.y)+(p.z-tcp.z)*(p.z-tcp.z))<1e-7);}
 cnc::Runtime cyc; auto c1=cnc::Parser::parse("N10 CYCLE800(0,0,0,0,0,0,0,30,0,45,0,0,0,0)"); assert(c1); assert(cyc.execute(*c1,e)); assert(cyc.kinematics().cycle800_active()); auto c2=cnc::Parser::parse("N20 CYCLE800()"); assert(c2); assert(cyc.execute(*c2,e)); assert(!cyc.kinematics().cycle800_active());
 cnc::Stock stock; assert(stock.initialize({{0,0,0},20,20,20,2})); auto before=stock.removed_count(); assert(stock.sweep_tool_segment({10,10,20},{10,10,0},{cnc::ToolShape::Ball,2.1,0})); assert(stock.removed_count()>before);
 cnc::Runtime held; held.hold(); assert(held.feed_hold()); held.clear_alarm(); assert(!held.feed_hold());
 cnc::ProgramLoader loader; assert(loader.load_text("N10 G0 X0\n\nN20 G1 X10 F500\n",e)); assert(loader.lines().size()==2&&loader.lines()[1].line_index==3);
 auto vr=cnc::VerifyEngine::run(loader.blocks()); assert(vr.passed&&vr.executed==2);
 cnc::Runtime flow2; cnc::ProgramExecutor ex2(flow2); cnc::ProgramLoader mainp, subp; assert(mainp.load_text("N10 CALL P100\nN20 G0 X20\n",e)); assert(subp.load_text("N100 G0 X5\nN110 RET\n",e)); ex2.load(mainp.blocks()); ex2.add_subprogram(100,subp.blocks()); assert(ex2.start()); while(ex2.state()==cnc::ExecutionState::Running) ex2.step(); assert(ex2.state()==cnc::ExecutionState::Completed&&std::abs(flow2.state().X-20)<1e-9);
 cnc::Runtime flow; flow.variables().set(1,10); cnc::ProgramExecutor ex(flow); cnc::ProgramLoader fl; assert(fl.load_text("N10 IF R1>5 GOTOF 30\nN20 G0 X99\nN30 G0 X10\n",e)); ex.load(fl.blocks()); assert(ex.start()); assert(flow.state().X==10); cnc::Runtime bounds; cnc::ProgramExecutor bx(bounds); cnc::ProgramLoader lb; assert(lb.load_text("N10 G0 X0\n",e)); bx.load(lb.blocks()); assert(bx.start()); while(bx.state()==cnc::ExecutionState::Running) bx.step(); assert(bx.state()==cnc::ExecutionState::Completed); assert(!bx.step());
 return 0;
}
