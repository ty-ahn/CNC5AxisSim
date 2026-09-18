#pragma once
#include <string>
namespace cnc {
struct MachineState { double X{},Y{},Z{},A{},C{}; };
class Kinematics {
public:
    bool validate(const MachineState&, std::string&) const;
    static double unwrap(double previous,double target);
};
}