#include "TypeDesc.h"
#include "TypeNameUtils.h"

#include <sstream>

#include <boost/algorithm/string.hpp>

bool FunctionDesc::is_constructor(const TypeDesc &dtype) const
{
    return (TypeNameUtils::getUnqualifiedIdentifier(dtype.type_name) == TypeNameUtils::getUnqualifiedIdentifier(name)) && 
        return_type_specifier.empty() && !is_destructor(); 
}

bool FunctionDesc::is_default_constructor(const TypeDesc &dtype) const
{
    if (!is_constructor(dtype)) return false;

    int na = 0;
    for (ParameterList::const_iterator i=params.begin(); i!=params.end(); ++i)
    {
        if (i->default_value.empty())
            ++na;
    }

    return na == 0;
}

std::string FunctionDesc::get_signature() const
{
    std::ostringstream oss;
    oss << return_type_specifier << "/" << name;
    for (ParameterList::const_iterator i=params.begin(); i!=params.end(); ++i)
    {
        oss << "/" << i->type_specifier;
    }
    if (is_static)
        oss << "/S";
    return oss.str();
}

void FunctionDesc::gen_type_name_signature(std::string &str) const
{
    boost::algorithm::replace_all(str, " ", "_");
    boost::algorithm::replace_all(str, "::", "_");
    boost::algorithm::replace_all(str, "const", "C5");
    boost::algorithm::replace_all(str, "&", "R1");
    boost::algorithm::replace_all(str, "*", "P1");
    boost::algorithm::replace_all(str, "<", "T1");
    boost::algorithm::replace_all(str, ">", "");
    boost::algorithm::replace_all(str, ",", "Comma");
}

void FunctionDesc::gen_name_signature()
{

    std::string str(return_type_specifier);
    gen_type_name_signature(str);
    std::ostringstream oss;
    oss << "__" << str << "__" << name;
    for (ParameterList::const_iterator i=params.begin(); i!=params.end(); ++i)
    {
        str = i->type_specifier;
        gen_type_name_signature(str);
        oss << "__" << str;
    }
    if (is_static)
        oss << "_S";
    name_signature = oss.str();
}
