#pragma once
#include "ProgramLoader.h"
#include <filesystem>
#include <unordered_map>
namespace cnc {
class ProgramCatalog {
 std::vector<Block> main_;
 std::unordered_map<int,std::vector<Block>> spf_;
public:
 bool load_directory(const std::string& dir,std::string& error);
 bool load_main(const std::string& path,std::string& error);
 const std::vector<Block>& main_program() const{return main_;}
 const std::unordered_map<int,std::vector<Block>>& subprograms() const{return spf_;}
};
}
