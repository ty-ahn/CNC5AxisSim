# CNC5AxisSim

Windows x64 CNC simulator foundation for Siemens SINUMERIK 840D-style MPF execution.

## Current build target
- C++20
- CMake
- MPF-style lexer/parser
- G90/G91
- XYZAC state
- F/S/T/D
- A-axis limit validation
- rotary unwrap utility
- CTest

APT → MPF conversion is intentionally not part of this project.

## CI
GitHub Actions builds the project on a Windows runner, runs CTest, performs a smoke test, and uploads the executable artifact.
