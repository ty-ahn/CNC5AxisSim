#include "ProgramCatalog.h"
#include <fstream>
#include <cctype>
namespace cnc {
static int program_number(const std::filesystem::path& p){
 std::string s=p.stem().string(); size_t i=0; while(i<s.size()&&!std::isdigit((unsigned char)s[i]))++i; if(i==s.size())return -1; return std::atoi(s.c_str()+i);
}
bool ProgramCatalog::load_main(const std::string& path,std::string& e){ProgramLoader l;if(!l.load_file(path,e))return false;main_=l.blocks();return true;}
bool ProgramCatalog::load_directory(const std::string& dir,std::string& e){
 spf_.clear(); main_.clear(); std::filesystem::path d(dir); if(!std::filesystem::exists(d)){e="program directory not found";return false;}
 for(auto& x:std::filesystem::directory_iterator(d)){if(!x.is_regular_file())continue;auto ext=x.path().extension().string();for(char& c:ext)c=char(std::toupper((unsigned char)c));if(ext!=".SPF"&&ext!=".MPF")continue;ProgramLoader l;if(!l.load_file(x.path().string(),e))return false;auto b=l.blocks();if(ext==".MPF"&&main_.empty())main_=b;else if(ext==".SPF"){int n=program_number(x.path());if(n>=0)spf_[n]=b;}}
 e.clear();return !main_.empty();
}
}