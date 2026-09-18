#include "cnc/Parser.h"
#include "cnc/Runtime.h"
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    cnc::Runtime rt;
    std::string error;

    if (argc > 1) {
        std::ifstream in(argv[1]);
        if (!in) {
            std::cerr << "Cannot open MPF: " << argv[1] << "\n";
            return 2;
        }
        std::string line;
        while (std::getline(in, line)) {
            auto block = cnc::Parser::parse(line);
            if (!block) continue;
            if (!rt.execute(*block, error)) {
                std::cerr << "ALARM N" << block->number << ": " << error << "\n";
                return 3;
            }
        }
    } else {
        const char* demo[] = {
            "N10 G90 G54 G0 X0 Y0 Z100",
            "N20 T1 D1 S8000 M3",
            "N30 G1 X100 Y50 Z20 A30 C45 F1000",
            "N40 G91 X5 C20",
            "N50 G90 G55 G0 X0 Y0 Z100",
            "N60 M5"
        };
        for (const char* line : demo) {
            auto block = cnc::Parser::parse(line);
            if (block && !rt.execute(*block, error)) {
                std::cerr << error << "\n";
                return 3;
            }
        }
    }

    const auto& s = rt.state();
    std::cout << "CNC5AxisSim CORE PASS\n";
    std::cout << "XYZAC = " << s.X << " " << s.Y << " " << s.Z
              << " " << s.A << " " << s.C << "\n";
    std::cout << "MOTION = "
              << (rt.modal().motion == cnc::MotionMode::Rapid ? "G0" :
                  rt.modal().motion == cnc::MotionMode::Linear ? "G1" :
                  rt.modal().motion == cnc::MotionMode::ArcCW ? "G2" : "G3")
              << " G" << rt.modal().work_offset
              << " " << (rt.modal().absolute ? "G90" : "G91") << "\n";
    std::cout << "F=" << rt.feed() << " S=" << rt.rpm()
              << " T=" << rt.tool() << " D=" << rt.d() << "\n";
    return 0;
}
