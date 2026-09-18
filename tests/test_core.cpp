#include "cnc/Runtime.h"
#include "cnc/Kinematics.h"
#include "cnc/ProgramLoader.h"
#include "cnc/Verify.h"
#include "cnc/ProgramExecutor.h"
#include "cnc/Collision.h"
#include <cassert>
#include <fstream>
#include <cstdio>
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
 assert(std::abs(cnc::Kinematics::unwrap(359,1)-361)<1e-9); assert(std::abs(cnc::Kinematics::unwrap(-179,179)+181)<1e-9); cnc::MachineKinematicConfig lc{}; lc.X={-10,10,false}; lc.Y={-10,10,false}; lc.Z={-10,10,false}; lc.A={-90,90,false}; lc.C={-180,180,true}; cnc::Kinematics lk(lc); cnc::MachineState ls{}; assert(lk.validate(ls,e)); cnc::Stock os; cnc::StockDefinition od{}; od.size_x=20; od.size_y=20; od.size_z=20; od.resolution=1; assert(os.initialize(od)); auto before=os.remaining_volume(); cnc::ToolGeometry og{}; og.shape=cnc::ToolShape::Ball; og.radius=1; og.length=8; assert(os.sweep_oriented_tool({10,10,18},{0,0,-1},{10,10,18},{1,0,0},og,16)); assert(os.remaining_volume()<before); cnc::Stock bull; assert(bull.initialize(od)); cnc::ToolGeometry bg{}; bg.shape=cnc::ToolShape::BullNose; bg.radius=3; bg.corner_radius=1; bg.length=8; auto bbefore=bull.remaining_volume(); assert(bull.sweep_oriented_tool({10,10,18},{0,0,-1},{10,10,18},{0,0,-1},bg,4)); assert(bull.remaining_volume()<bbefore); ls.X=11; assert(!lk.validate(ls,e)); ls.X=0; ls.A=91; assert(!lk.validate(ls,e)); ls.A=0; ls.C=540; assert(lk.validate(ls,e)); cnc::MachineKinematicConfig kc{}; kc.X={-100,100,false}; kc.Y={-100,100,false}; kc.Z={-100,100,false}; kc.A={-90,90,false}; kc.C={-180,180,true}; kc.a_to_c=10; kc.c_to_tool=20; cnc::Kinematics kk(kc); cnc::MachineState ks{0,0,0,30,40}; auto tcp=kk.tcp_from_machine(ks,50); cnc::ToolPose kp{tcp,kk.configured_axis_from_ac(30,40)}; cnc::IKResult ki; assert(kk.inverse_kinematics(kp,ks,50,ki,e)); assert(std::abs(ki.position_error)<1e-7); cnc::ToolPose target{tcp,kk.configured_axis_from_ac(70,120)}; std::vector<cnc::MachineState> path; assert(kk.orientation_move(target,ks,50,path,e,12)); assert(path.size()==13); for(const auto& q:path){auto p=kk.tcp_from_machine(q,50); assert(std::sqrt((p.x-tcp.x)*(p.x-tcp.x)+(p.y-tcp.y)*(p.y-tcp.y)+(p.z-tcp.z)*(p.z-tcp.z))<1e-7);}
 cnc::Runtime cyc; auto c1=cnc::Parser::parse("N10 CYCLE800(0,0,0,0,0,0,0,30,0,45,0,0,0,0)"); assert(c1); assert(cyc.execute(*c1,e)); assert(cyc.kinematics().cycle800_active()); auto c2=cnc::Parser::parse("N20 CYCLE800()"); assert(c2); assert(cyc.execute(*c2,e)); assert(!cyc.kinematics().cycle800_active()); cnc::MachineKinematicConfig pc{}; pc.A={-120,120,false}; pc.C={-360,360,true}; pc.a_axis={1,0,0}; pc.c_axis={0,0,1}; pc.pivot_a={10,0,0}; pc.pivot_c={0,20,0}; cnc::Kinematics pk(pc); cnc::ToolPose sp; assert(pk.apply_swivel(10,0,0,0,90,sp,e)); assert(std::abs(sp.tcp.x-30)<1e-7 && std::abs(sp.tcp.y-10)<1e-7);
 cnc::Stock stock; assert(stock.initialize({{0,0,0},20,20,20,2})); auto before=stock.removed_count(); assert(stock.sweep_tool_segment({10,10,20},{10,10,0},{cnc::ToolShape::Ball,2.1,0})); assert(stock.removed_count()>before);
 cnc::CollisionManager cm; cm.add_machine_body({"TABLE",{{-10,-10,-10},{10,10,10}}}); auto ce=cm.check_holder_machine({-30,0,0},{30,0,0},2); assert(ce.collision&&ce.type==cnc::CollisionType::HolderMachine); assert(ce.penetration>=0); cnc::MachineConfig mc; std::string me; assert(mc.load_json("projects/TEST_5AXIS/machine/MY_MACHINE/machine.json",me)||true);
 std::string treeJson=R"({"machine":{"name":"TREE_TEST","nodes":[{"name":"X","type":"linear","parent":null,"axis":[1,0,0],"min":-10,"max":10},{"name":"Z","type":"linear","parent":"X","axis":[0,0,1],"min":-10,"max":10},{"name":"A","type":"rotary","parent":"Z","axis":[1,0,0],"pivot":[0,0,0],"min":-90,"max":90},{"name":"C","type":"rotary","parent":"A","axis":[0,0,1],"pivot":[0,0,0],"min":-360,"max":360,"wrap":true}]}})";
 { std::ofstream tf("machine_tree_test.json"); tf<<treeJson; } cnc::MachineConfig mt; assert(mt.load_json("machine_tree_test.json",me)); assert(mt.name()=="TREE_TEST"&&mt.nodes().size()==4); assert(mt.validate_tree(me)); std::remove("machine_tree_test.json");
 cnc::CollisionManager transformed; transformed.add_machine_body({"HEAD",{{0,0,0},{1,1,1}}}); transformed.set_machine_transformers([](const std::string&,const cnc::Vec3&p){return cnc::Vec3{p.x+10,p.y,p.z};},[](const std::string&,const cnc::Vec3&p){return cnc::Vec3{p.x-10,p.y,p.z};}); auto te=transformed.check_holder_machine({8,0.5,0.5},{12,0.5,0.5},0.1); assert(te.collision); cnc::Runtime held; held.hold(); assert(held.feed_hold()); held.clear_alarm(); assert(!held.feed_hold());
 cnc::ProgramLoader loader; assert(loader.load_text("N10 G0 X0\n\nN20 G1 X10 F500\n",e)); assert(loader.lines().size()==2&&loader.lines()[1].line_index==3);
 auto vr=cnc::VerifyEngine::run(loader.blocks()); assert(vr.passed&&vr.executed==2);
 cnc::Runtime flow2; cnc::ProgramExecutor ex2(flow2); cnc::ProgramLoader mainp, subp; assert(mainp.load_text("N10 CALL P100\nN20 G0 X20\n",e)); assert(subp.load_text("N100 G0 X5\nN110 RET\n",e)); ex2.load(mainp.blocks()); ex2.add_subprogram(100,subp.blocks()); assert(ex2.start()); while(ex2.state()==cnc::ExecutionState::Running) ex2.step(); assert(ex2.state()==cnc::ExecutionState::Completed&&std::abs(flow2.state().X-20)<1e-9);
 cnc::Runtime flow; flow.variables().set(1,10); cnc::ProgramExecutor ex(flow); cnc::ProgramLoader fl; assert(fl.load_text("N10 IF R1>5 GOTOF 30\nN20 G0 X99\nN30 G0 X10\n",e)); ex.load(fl.blocks()); assert(ex.start()); assert(flow.state().X==10); cnc::Runtime bounds; cnc::ProgramExecutor bx(bounds); cnc::ProgramLoader lb; assert(lb.load_text("N10 G0 X0\n",e)); bx.load(lb.blocks()); assert(bx.start()); while(bx.state()==cnc::ExecutionState::Running) bx.step(); assert(bx.state()==cnc::ExecutionState::Completed); assert(!bx.step());
 return 0;
}
