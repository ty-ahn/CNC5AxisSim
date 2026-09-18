#pragma once
#include "Parser.h"
#include <string>
#include <vector>
namespace cnc {
struct ProgramLine { size_t line_index{}; Block block{}; };
class ProgramLoader {
 std::vector<ProgramLine> lines_;
public:
 bool load_text(const std::string& text,std::string& error);
 bool load_file(const std::string& path,std::string& error);
 const std::vector<ProgramLine>& lines()const{return lines_;}
 std::vector<Block> blocks()const;
};
}