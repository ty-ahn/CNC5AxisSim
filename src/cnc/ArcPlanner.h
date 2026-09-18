#pragma once
#include "Motion.h"
namespace cnc { struct ArcDefinition{double I{},J{},K{};}; class ArcPlanner{public:static bool plan(const MotionPoint&,const MotionPoint&,const ArcDefinition&,bool,double,std::vector<MotionPoint>&,std::string&,int=32);}; }