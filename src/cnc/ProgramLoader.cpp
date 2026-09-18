#include "ProgramLoader.h"
#include <fstream>
#include <sstream>
namespace cnc {
bool ProgramLoader::load_text(const std::string& text,std::string& error){
 lines_.clear(); std::istringstream in(text); std::string line; size_t n=0;
 while(std::getline(in,line)){++n; if(line.find_first_not_of(" \t\r\n")==std::string::npos)continue; auto b=Parser::parse(line); if(!b){error="Parse error at line "+std::to_string(n);lines_.clear();return false;} lines_.push_back({n,*b});}
 error.clear(); return true;
}
bool ProgramLoader::load_file(const std::string& path,std::string& error){std::ifstream f(path);if(!f){error="Cannot open program: "+path;return false;}std::ostringstream s;s<<f.rdbuf();return load_text(s.str(),error);}
std::vector<Block> ProgramLoader::blocks()const{std::vector<Block> b;b.reserve(lines_.size());for(auto&l:lines_)b.push_back(l.block);return b;}
}