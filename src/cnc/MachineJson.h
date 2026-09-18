#pragma once
#include <map>
#include <string>
#include <vector>
#include <variant>
namespace cnc {
class JsonValue {
public:
 using Object=std::map<std::string,JsonValue>; using Array=std::vector<JsonValue>;
 using Data=std::variant<std::nullptr_t,bool,double,std::string,Array,Object>;
 Data data;
 JsonValue():data(nullptr){} JsonValue(Data d):data(std::move(d)){}
 bool object()const{return std::holds_alternative<Object>(data);} bool array()const{return std::holds_alternative<Array>(data);}
 const Object* obj()const{return std::get_if<Object>(&data);} const Array* arr()const{return std::get_if<Array>(&data);}
 const JsonValue* get(const std::string&k)const{auto o=obj();if(!o)return nullptr;auto i=o->find(k);return i==o->end()?nullptr:&i->second;}
 const std::string* str()const{return std::get_if<std::string>(&data);} const double* number()const{return std::get_if<double>(&data);}
};
bool parse_json(const std::string&text,JsonValue&out,std::string&error);
std::string json_escape(const std::string&s);
}
