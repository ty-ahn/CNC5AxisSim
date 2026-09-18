#pragma once
#include <optional>
#include <string>
#include <vector>
namespace cnc {
struct Word { char letter{}; double value{}; };
struct Block { int number{-1}; std::vector<Word> words; std::string source; };
class Parser { public: static std::optional<Block> parse(const std::string& line); };
}