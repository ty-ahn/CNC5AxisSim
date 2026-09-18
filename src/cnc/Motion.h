#pragma once
#include "Kinematics.h"
#include <string>
#include <vector>
namespace cnc {
struct MotionPoint { double X{},Y{},Z{},A{},C{}; };
class MotionPlanner {
public:
 static bool plan(const MotionPoint&,const MotionPoint&,bool,double,std::vector<MotionPoint>&,std::string&,int=20,const Kinematics* kinematics=nullptr);
 static double rotary_distance(double,double);
 static double linear_length(const MotionPoint&,const MotionPoint&);
};
}