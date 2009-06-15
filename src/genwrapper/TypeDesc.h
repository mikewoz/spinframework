#ifndef TYPEDESC_H_
#define TYPEDESC_H_

#include <vector>
#include <string>

struct ParameterDesc
{
    std::string type_specifier;
    std::string name;
    std::string default_value;
};

typedef std::vector<ParameterDesc> ParameterList;

struct TypeDesc;

struct PropertyDesc
{
    enum Type
    {
        SIMPLE  = 0,
        ARRAY   = 1,
        INDEXED = 2
    };

    std::string get_method;
    std::string set_method;
    std::string add_method;
    std::string insert_method;
    std::string remove_method;
    std::string count_method;

    Type type;
    std::string name;
    std::string type_name;
};

typedef std::vector<PropertyDesc> PropertyList;

struct FunctionDesc
{
    std::string name_signature;
    
    std::string return_type_specifier;
    std::string name;
    ParameterList params;
    bool is_const;
    bool is_static;
    bool is_virtual;
    bool is_pure_virtual;
    bool is_explicit;
    
    std::string briefHelp;
    std::string detailedHelp;
    
    bool is_constructor(const TypeDesc &decltype) const;
    bool is_default_constructor(const TypeDesc &decltype) const;
    bool is_destructor() const { return name[0] == '~'; }
    std::string get_signature() const;
    void gen_name_signature();
    void gen_type_name_signature(std::string &str) const;
    bool valid() const { return !name.empty(); }
    FunctionDesc() :
        is_const(false),
        is_static(false),
        is_virtual(false),
        is_pure_virtual(false),
        is_explicit(false)
    {
    }
};

typedef std::vector<FunctionDesc> FunctionList;


struct AttributDesc
{
  std::string type;
  std::string name;
  bool is_const;
  bool is_static;
  bool valid() const { return ((!name.empty()) && (!type.empty())); }
  AttributDesc() : is_static(false) {}
  AttributDesc(std::string t, std::string n, bool s = false) : type(t), name(n), is_static(s) {}
};

typedef std::vector<AttributDesc> AttributList;


typedef std::vector<std::string> StringList;

struct BaseTypeDesc
{
    std::string name;
    bool is_virtual;
    BaseTypeDesc(): is_virtual(false) {}
    BaseTypeDesc(const std::string &n, bool v = false): name(n), is_virtual(v) {}
};

typedef std::vector<BaseTypeDesc> BaseTypeList;

struct TypeDesc
{
    bool undefined;
    std::string alias;    // for typedef's
    std::string declaring_file_name;
    std::string type_name;
    BaseTypeList base_types;
    StringList enum_labels;
    StringList needed_headers;
    FunctionList methods;
    FunctionList prot_methods;
    FunctionList priv_methods;
    AttributList public_attrib;
    PropertyList properties;
    ParameterList template_params;
    bool is_abstract;
    bool default_constructor_private;
    bool has_custom_constructors;
    bool destructor_private;
    bool is_namespace;

    std::string briefHelp;
    std::string detailedHelp;

    TypeDesc() 
    :    undefined(false),
        is_abstract(false), 
        default_constructor_private(false), 
        has_custom_constructors(false),
        destructor_private(false),
        is_namespace(false)
    {
    }
};

#endif
