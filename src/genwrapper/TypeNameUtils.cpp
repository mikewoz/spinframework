#include "TypeNameUtils.h"

#include <string>
#include <locale>
#include <algorithm>

namespace
{

    class Tokenizer
    {
    public:
        void reset()
        {
            tokens_.clear();
        }

        const TypeNameUtils::TokenList &getTokens() const
        {
            return tokens_;
        }

        TypeNameUtils::TokenList &getTokens()
        {
            return tokens_;
        }

        void tokenize(const std::string &s);

    protected:
        bool is_id_leading(char c) const;
        bool is_identifier(char c) const;
        bool is_num_leading(char c) const;
        bool is_number(char c) const;
        void change_state(int newstate);
        bool test_state(char c);
        bool is_keyword(const std::string &k) const;

    private:
        int state_;
        TypeNameUtils::TokenList tokens_;
        std::locale loc_;
        TypeNameUtils::Token last_token_;
        std::string::size_type pos_;
        bool escape_;
    };

    std::string trim(const std::string &s)
    {
        std::string r(s);
        std::string::size_type p = r.find_first_not_of(" \t\n");
        if (p != std::string::npos)
            r.erase(0, p);
        p = r.find_last_not_of(" \t\n");
        if (p != std::string::npos)
            r.erase(p+1);
        return r;
    }

}

std::string TypeNameUtils::removeGarbageFromSpecifier(const std::string &type_spec)
{
    std::locale loc;
    std::string last_word;
    std::string res;

    for (std::string::const_iterator i=type_spec.begin(); i!=type_spec.end(); ++i)
    {
        if (std::isspace(*i, loc) && !last_word.empty())
        {
            if (last_word != "virtual" && last_word != "static")
            {
                if (!res.empty()) 
                    res.push_back(*i);
                res += last_word;
            }
            last_word.clear();
        }
        else
        {
            last_word.push_back(*i);
        }
    }

    if (!res.empty()) 
        res.push_back(' ');
    res += last_word;

    return res;
}

bool TypeNameUtils::hasDefaults(const ParameterList &pl)
{
    for (ParameterList::const_iterator i=pl.begin(); i!=pl.end(); ++i)
        if (!i->default_value.empty()) return true;
    return false;
}

std::string TypeNameUtils::getUnqualifiedIdentifier(const std::string &id)
{
    std::string::size_type p = id.rfind("::");
    if (p == std::string::npos) return id;
    return id.substr(p+2);
}

bool Tokenizer::is_id_leading(char c) const
{
    return std::isalpha(c, loc_) || c == '_' || c == ':';
}

bool Tokenizer::is_identifier(char c) const
{
    return is_id_leading(c) || c == '.' || std::isdigit(c, loc_);
}

bool Tokenizer::is_num_leading(char c) const
{
    return std::isdigit(c, loc_);
}

bool Tokenizer::is_number(char c) const
{
    return is_num_leading(c) || c == '.' || c == 'e';
}


void Tokenizer::tokenize(const std::string &s)
{
    std::locale loc;

    TypeNameUtils::TokenList tokens;

    last_token_ = TypeNameUtils::Token();    
    state_ = 0;
    escape_ = false;

    pos_ = 0;
    for (std::string::const_iterator i=s.begin(); i!=s.end(); ++i, ++pos_)
    {
        if (*i == '\\' && !escape_)
        {
            escape_ = true;
            continue;
        }

        switch (state_)
        {
        case 0:
            test_state(*i);
            break;

        case TypeNameUtils::Token::IDENTIFIER:
            if (!is_identifier(*i) && !test_state(*i))
                change_state(0);
            break;

        case TypeNameUtils::Token::NUMBER:
            if (!is_number(*i) && !test_state(*i))
                change_state(0);
            break;

        case TypeNameUtils::Token::STRING:
            test_state(*i);
            break;

        case TypeNameUtils::Token::OTHER:
            test_state(*i);
            if (std::isspace(*i, loc_))
                change_state(0);
            break;
        }

        if (escape_)
            escape_ = false;
    }

    change_state(0);

    for (TypeNameUtils::TokenList::iterator i=tokens_.begin(); i!=tokens_.end(); ++i)
    {
        if (i->type == TypeNameUtils::Token::IDENTIFIER)
        {
            if (is_keyword(s.substr(i->beg, i->len)))
                i->type = TypeNameUtils::Token::KEYWORD;
        }
    }
}

void Tokenizer::change_state(int newstate)
{
    if (state_ != 0)
    {
        last_token_.type = static_cast<TypeNameUtils::Token::Type>(state_);
        last_token_.len = pos_ - last_token_.beg;
        if (last_token_.valid())
        {
            tokens_.push_back(last_token_);
        }
    }
    state_ = newstate;
    last_token_ = TypeNameUtils::Token();
    last_token_.beg = pos_;
}

bool Tokenizer::test_state(char c)
{
    if (state_ != TypeNameUtils::Token::STRING)
    {
        if (c == '"' && !escape_)
        {
            change_state(TypeNameUtils::Token::STRING);
            return true;
        }
    }
    else
    {
        if (c == '"' && !escape_)
        {
            ++pos_;
            change_state(0);
            --pos_;
            return true;
        }
        return false;
    }

    if (state_ != TypeNameUtils::Token::NUMBER)
    {
        if (is_num_leading(c))
        {
            change_state(TypeNameUtils::Token::NUMBER);
            return true;
        }
    }

    if (state_ != TypeNameUtils::Token::IDENTIFIER)
    {
        if (is_id_leading(c))
        {
            change_state(TypeNameUtils::Token::IDENTIFIER);
            return true;
        }
    }

    if (state_ != TypeNameUtils::Token::OTHER)
    {
        if (!std::isspace(c, loc_))
        {
            change_state(TypeNameUtils::Token::OTHER);
            return true;
        }
    }

    return false;
}

bool Tokenizer::is_keyword(const std::string &k) const
{
    return
        k == "asm" ||
        k == "auto" ||
        k == "bool" ||
        k == "break" ||
        k == "case" ||
        k == "catch" ||
        k == "char" ||
        k == "class" ||
        k == "const" ||
        k == "const_cast" ||
        k == "continue" ||
        k == "default" ||
        k == "delete" ||
        k == "do" ||
        k == "double" ||
        k == "dynamic_cast" ||
        k == "else" ||
        k == "enum" ||
        k == "explicit" ||
        k == "export" ||
        k == "extern" ||
        k == "false" ||
        k == "float" ||
        k == "for" ||
        k == "friend" ||
        k == "goto" ||
        k == "if" ||
        k == "int" ||
        k == "long" ||
        k == "mutable" ||
        k == "namespace" ||
        k == "new" ||
        k == "operator" ||
        k == "private" ||
        k == "public" ||
        k == "register" ||
        k == "reinterpret_cast" ||
        k == "return" ||
        k == "short" ||
        k == "signed" ||
        k == "sizeof" ||
        k == "static" ||
        k == "static_cast" ||
        k == "struct" ||
        k == "switch" ||
        k == "template" ||
        k == "this" ||
        k == "throw" ||
        k == "true" ||
        k == "try" ||
        k == "typedef" ||
        k == "typeid" ||
        k == "typename" ||
        k == "union" ||
        k == "unsigned" ||
        k == "using" ||
        k == "virtual" ||
        k == "void" ||
        k == "volatile" ||
        k == "wchar_t" ||
        k == "while";
}

std::string TypeNameUtils::qualifyIdentifier(const std::string &pqid, const TypeDesc &scope, const TypeRegistry &reg)
{
    std::string id = pqid;
    std::string::size_type p = pqid.find("::");
    if (p != std::string::npos)
    {
        id = pqid.substr(0, p);
    }

    // unwind namespace
    std::string nspath(scope.type_name);
    
    do
    {
        p = nspath.rfind("::");
        if (p != std::string::npos)
        {
            std::string qname = nspath + "::" + id;
            if (reg.symbolExists(qname)) 
                return nspath + "::" + pqid;
            nspath.erase(p);
        }        
    } while (p != std::string::npos);

    std::string qname = nspath + "::" + id;
    if (reg.symbolExists(qname)) 
        return nspath + "::" + pqid;


    // recurse base types
    for (BaseTypeList::const_iterator i=scope.base_types.begin(); i!=scope.base_types.end(); ++i)
    {
        const TypeDesc *td = reg.findTypeDescription(i->name);
        if (!td)
        {
            Notify::warn("could not find description of type `" + i->name + "' during TypeNameUtils::qualifyIdentifier()");
        }
        else
        {
            std::string res = qualifyIdentifier(pqid, *td, reg);
            if (res != pqid)
                return res;
        }
    }

    return pqid;
}

void TypeNameUtils::qualifyAllIdentifiers(std::string &s, const TypeDesc &scope, const TypeRegistry &reg)
{
    Tokenizer tk;
    tk.tokenize(s);
    TokenList tokens = tk.getTokens();
    for (TokenList::iterator i=tokens.begin(); i!=tokens.end(); ++i)
    {
        if (i->type == Token::IDENTIFIER)
        {
            std::string qid = qualifyIdentifier(s.substr(i->beg, i->len), scope, reg);
            s.replace(i->beg, i->len, qid);
            int delta = static_cast<int>(qid.length()) - static_cast<int>(i->len);
            for (TokenList::iterator j=i; j!=tokens.end(); ++j)
            {
                j->beg += delta;
            }
        }
    }
}

std::string TypeNameUtils::getNamespace(const std::string &id)
{
    std::string::size_type p = id.rfind("::");
    if (p == std::string::npos) return id;
    return id.substr(0, p);
}

std::string::size_type eat_until(const std::string &s, std::string::size_type pos, char delim)
{
    while (pos<s.length())
    {
        char c = s[pos];

        if (c == delim)
            break;

        ++pos;

        switch (c)
        {
        case '<':
            pos = eat_until(s, pos, '>');
            break;
        case '"':
            pos = eat_until(s, pos, '"');
            break;
        case '\'':
            pos = eat_until(s, pos, '\'');
            break;
        default: ;
        }
    }

    return pos;
}

bool TypeNameUtils::splitTemplateDeclaration(const std::string &decl, std::string &name, StringList &params)
{
    std::string::size_type p1 = decl.find('<');
    std::string::size_type p2 = decl.rfind('>');

    if (p1 == std::string::npos || p2 == std::string::npos)
        return false;

    name = trim(decl.substr(0, p1));
    std::string plist = trim(decl.substr(p1+1, p2-p1-1));
    std::string::size_type pos = 0;    

    do
    {
        std::string::size_type newpos = eat_until(plist, pos, ',');        
        params.push_back(trim(plist.substr(pos, newpos-pos)));
        pos = newpos+1;

    } while (pos<plist.length());

    return true;
}

std::string TypeNameUtils::trim(const std::string &s)
{
    std::string r(s);
    std::string::size_type p = r.find_first_not_of(" \t\n");
    if (p != std::string::npos)
        r.erase(0, p);
    p = r.find_last_not_of(" \t\n");
    if (p != std::string::npos)
        r.erase(p+1);
    return r;
}

void TypeNameUtils::translateIdentifiers(std::string &decl, const std::string &src, const std::string &dest)
{
    Tokenizer tk;
    tk.tokenize(decl);
    TokenList tokens = tk.getTokens();
    for (TokenList::iterator i=tokens.begin(); i!=tokens.end(); ++i)
    {
        if (i->type == Token::IDENTIFIER)
        {
            if (decl.substr(i->beg, i->len) == src)
            {
                decl.replace(i->beg, i->len, dest);
                int delta = static_cast<int>(dest.length()) - static_cast<int>(i->len);
                for (TokenList::iterator j=i; j!=tokens.end(); ++j)
                {
                    j->beg += delta;
                }
            }
        }
    }
}

void TypeNameUtils::tokenize(const std::string &s, TokenList &tokens)
{
    Tokenizer tk;
    tk.tokenize(s);
    tokens.swap(tk.getTokens());
}


std::string TypeNameUtils::removeModifiersFromSpecifier(const std::string &type_spec)
{
    std::string res(type_spec);
    TokenList tokens;
    tokenize(type_spec, tokens);
    int templ = 0;
    std::size_t delta = 0;
    for (TokenList::const_iterator i=tokens.begin(); i!=tokens.end(); ++i)
    {
        std::string tok = type_spec.substr(i->beg, i->len);
        if (tok == "<") ++templ;
        if (tok == ">") --templ;

        if (templ == 0 && (i->type == Token::KEYWORD || tok == "*" || tok == "&"))
        {
            res.erase(i->beg-delta, i->len);
            delta += i->len;            
        }
    }
    return trim(res);
}
