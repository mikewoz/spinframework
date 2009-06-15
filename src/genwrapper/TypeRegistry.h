#ifndef TYPEREGISTRY_H_
#define TYPEREGISTRY_H_

#include "TypeDesc.h"
#include "Notify.h"
#include "Configuration.h"

#include <map>
#include <set>

class TypeRegistry
{
public:
    typedef std::vector<TypeDesc> TypeList;
    typedef std::map<std::string, TypeList> TypeMap;
    typedef std::set<std::string> SymbolSet;

    typedef TypeMap::value_type value_type;
    typedef TypeMap::mapped_type mapped_type;
    typedef TypeMap::key_type key_type;
    typedef TypeMap::const_iterator const_iterator;

    TypeRegistry(const Configuration &cfg): cfg_(cfg) {}

    inline void registerSymbol(const std::string &s);
    void registerTypeDescription(const TypeDesc &td);

    void registerTemplate(const TypeDesc &td);
    const TypeDesc *findTemplate(const std::string &type_name) const;

    const TypeDesc *findTypeDescription(const std::string &type_name) const;
    std::string getDeclaringFileName(const std::string &type_name) const;
    inline bool symbolExists(const std::string &s) const;

    inline const_iterator begin() const;
    inline const_iterator end() const;
#if 0
    template<typename Func>
    bool traverseBaseTypes(const TypeDesc &start, Func functor) const;
#endif
    void consolidate();

protected:
    void fill_attributes();
    void qualify_type_names();
    void qualify_single_type(TypeDesc &td);
    bool has_pure_methods(const TypeDesc &td) const;
    void collect_pure_methods(const TypeDesc &td, FunctionList &fl) const;
    void define_properties();
    void instantiate_templates();
    bool create_template_instance(const std::string &decl, TypeDesc &newtd);
    bool translate_template(TypeDesc &td, const std::string &decl, const StringList &params);
    void translate_identifier(TypeDesc &td, const std::string &src, const std::string &dest);
    void collect_template_instances();
    void recurse_template_instances(const std::string &declfile, const std::string &decl, std::set<std::string> &tempset);
    
    bool same_parameters(const ParameterList &p1, const ParameterList &p2) const;

    void fill_needed_headers();
    void add_referenced_types(const std::string &decl, StringList &out) const;

private:
    TypeMap tfmap_;
    SymbolSet symbols_;
    TypeList templates_;

    typedef std::set<std::string> StringSet;
    typedef std::map<std::string, StringSet> StringSetMap;
    StringSetMap temp_inst_;

    const Configuration &cfg_;
};

// INLINE METHODS

inline TypeRegistry::const_iterator TypeRegistry::begin() const
{
    return tfmap_.begin();
}

inline TypeRegistry::const_iterator TypeRegistry::end() const
{
    return tfmap_.end();
}

#if 0
template<typename Func>
bool TypeRegistry::traverseBaseTypes(const TypeDesc &start, Func functor) const
{
    for (StringList::const_iterator i=start->base_type_names.begin(); i!=start->base_type_names.end(); ++i)
    {
        const TypeDesc *td = findTypeDescription(*i);
        if (!td)
        {
            Notify::warn("could not find description of type `" + *i + "' during traverseBaseTypes()");
        }
        else
        {
            if (!functor(*td))
                return false;
        }
    }
    return true;
}
#endif

inline void TypeRegistry::registerSymbol(const std::string &s)
{
    symbols_.insert(s);
}

inline bool TypeRegistry::symbolExists(const std::string &s) const
{
    return symbols_.find(s) != symbols_.end();
}

#endif
