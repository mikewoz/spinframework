#ifndef TYPENAMEUTILS_H_
#define TYPENAMEUTILS_H_

#include "TypeRegistry.h"
#include "TypeDesc.h"

#include <vector>
#include <string>
#include <locale>

struct TypeNameUtils
{
    struct Token
    {
        enum Type
        {
            IDENTIFIER = 1,
            STRING     = 2,
            NUMBER     = 3,
            KEYWORD    = 4,
            OTHER      = 5
        };

        Type type;
        std::string::size_type beg, len;

        bool valid() const { return (len>0 || len==std::string::npos) && beg!=std::string::npos; }

        Token(): type(OTHER), beg(std::string::npos), len(std::string::npos) {}
    };

    typedef std::vector<Token> TokenList;

    static std::string trim(const std::string &s);
    static void tokenize(const std::string &s, TokenList &tokens);
    static std::string qualifyIdentifier(const std::string &id, const TypeDesc &scope, const TypeRegistry &reg);
    static void qualifyAllIdentifiers(std::string &s, const TypeDesc &scope, const TypeRegistry &reg);
    static std::string removeGarbageFromSpecifier(const std::string &type_spec);
    static std::string removeModifiersFromSpecifier(const std::string &type_spec);
    static std::string getUnqualifiedIdentifier(const std::string &id);
    static std::string getNamespace(const std::string &id);
    static bool hasDefaults(const ParameterList &pl);
    static bool splitTemplateDeclaration(const std::string &decl, std::string &name, StringList &params);
    static void translateIdentifiers(std::string &decl, const std::string &src, const std::string &dest);
};

#endif
