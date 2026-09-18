#include "ProgramCatalog.h"
#include <fstream>
#include <cctype>
#include <cstdlib>
#include <algorithm>
namespace cnc {
static int program_number(const std::filesystem::path& p){
 std::string s=p.stem().string(); size_t i=0;
 while(i<s.size()&&!std::isdigit((unsigned char)s[i]))++i;
 if(i==s.size())return -1;
 return std::atoi(s.c_str()+i);
}
bool ProgramCatalog::load_main(const std::string& path,std::string& e){ProgramLoader l;if(!l.load_file(path,e))return false;main_=l.blocks();return true;}
bool ProgramCatalog::load_directory(const std::string& dir,std::string& e){
 spf_.clear(); main_.clear(); std::filesystem::path d(dir);
 if(!std::filesystem::exists(d)){e="program directory not found";return false;}
 std::vector<std::filesystem::path> mpf,spf;
 for(const auto& x:std::filesystem::directory_iterator(d)){
  if(!x.is_regular_file())continue;
  std::string ext=x.path().extension().string(); for(char& c:ext)c=char(std::toupper((unsigned char)c));
  if(ext==".MPF")mpf.push_back(x.path()); else if(ext==".SPF")spf.push_back(x.path());
 }
 auto by_name=[](const auto&a,const auto&b){return a.filename().string()<b.filename().string();};
 std::sort(mpf.begin(),mpf.end(),by_name); std::sort(spf.begin(),spf.end(),by_name);
 auto it=std::find_if(mpf.begin(),mpf.end(),[](const auto& p){std::string n=p.filename().string();for(char& c:n)c=char(std::toupper((unsigned char)c));return n=="MAIN.MPF";});
 if(it==mpf.end()&&!mpf.empty())it=mpf.begin();
 if(it!=mpf.end()){ProgramLoader l;if(!l.load_file(it->string(),e))return false;main_=l.blocks();}
 for(const auto& p:spf){int n=program_number(p);if(n<0)continue;ProgramLoader l;if(!l.load_file(p.string(),e))return false;spf_[n]=l.blocks();}
 e.clear();return !main_.empty();
}
}