cmake_minimum_required(VERSION 3.20)
project(CNC5AxisSim VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(cnc_core
    src/cnc/Parser.cpp
    src/cnc/Kinematics.cpp
    src/cnc/Runtime.cpp
)
target_include_directories(cnc_core PUBLIC src)

add_executable(CNC5AxisSim src/main.cpp)
target_link_libraries(CNC5AxisSim PRIVATE cnc_core)

enable_testing()
add_executable(CNC5AxisSimTests tests/test_core.cpp)
target_link_libraries(CNC5AxisSimTests PRIVATE cnc_core)
add_test(NAME CNC5AxisSimTests COMMAND CNC5AxisSimTests)
