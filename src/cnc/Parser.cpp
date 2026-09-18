#include "Parser.h"
#include <cctype>
#include <cstdlib>
namespace cnc {
std::optional<Block> Parser::parse(const std::string& line) {
    Block b; b.source=line;
    std::string s=line;
    if (auto p=s.find(';'); p!=std::string::npos) s=s.substr(0,p);
    for (size_t i=0;i<s.size();) {
        while(i<s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        if(i>=s.size()) break;
        char c=static_cast<char>(std::toupper(static_cast<unsigned char>(s[i++])));
        if(!std::isalpha(static_cast<unsigned char>(c))) continue;
        size_t j=i;
        while(j<s.size() && (std::isdigit(static_cast<unsigned char>(s[j]))||s[j]=='+'||s[j]=='-'||s[j]=='.'||s[j]=='e'||s[j]=='E')) ++j;
        if(j==i) continue;
        try {
            double v=std::stod(s.substr(i,j-i));
            b.words.push_back({c,v});
            if(c=='N') b.number=static_cast<int>(v);
        } catch(...) {}
        i=j;
    }
    return b.words.empty()?std::nullopt:std::optional<Block>(b);
}
}