#include "cnc/Parser.h"
#include "cnc/Runtime.h"
#include <iostream>
#include <fstream>
#include <string>
int main(int argc,char**argv){
    cnc::Runtime rt; std::string error;
    if(argc>1){
        std::ifstream in(argv[1]);
        if(!in){std::cerr<<"Cannot open MPF: "<<argv[1]<<"\n";return 2;}
        std::string line;
        while(std::getline(in,line)){
            auto b=cnc::Parser::parse(line); if(!b) continue;
            if(!rt.execute(*b,error)){std::cerr<<"ALARM N"<<b->number<<": "<<error<<"\n";return 3;}
        }
    } else {
        for(const char* line: {"N10 G90 G0 X0 Y0 Z100","N20 T1 D1 S8000 M3","N30 G1 X100 Y50 Z20 A30 C45 F1000"}){
            auto b=cnc::Parser::parse(line); if(b && !rt.execute(*b,error)){std::cerr<<error<<"\n";return 3;}
        }
    }
    const auto&s=rt.state();
    std::cout<<"CNC5AxisSim CORE PASS\n";
    std::cout<<"XYZAC = "<<s.X<<" "<<s.Y<<" "<<s.Z<<" "<<s.A<<" "<<s.C<<"\n";
    std::cout<<"F="<<rt.feed()<<" S="<<rt.rpm()<<" T="<<rt.tool()<<" D="<<rt.d()<<"\n";
    return 0;
}