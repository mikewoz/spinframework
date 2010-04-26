#include "TypeRegistry.h"
#include "TypeNameUtils.h"
#include "Notify.h"

#include <algorithm>

#ifdef WIN32
#pragma warning (disable: 4503)
#endif

namespace
{
    bool begins_with(const std::string &s1, const std::string &s2)
    {
        return s1.length() >= s2.length() && (s1.substr(0, s2.length()) == s2);
    }

    int getPropertyPower(const PropertyDesc &p)
    {
        int power = 0;
        if (!p.add_method.empty()) ++power;
        if (!p.insert_method.empty()) ++power;
        if (!p.remove_method.empty()) ++power;
        if (!p.get_method.empty()) ++power;
        if (!p.set_method.empty()) ++power;
        if (!p.count_method.empty()) ++power;
        return power;
    }

    struct PropertySorter
    {
        bool operator()(const PropertyDesc &p1, const PropertyDesc &p2) const
        {
            if (p1.type > p2.type) return true;
            if (p1.type < p2.type) return false;
            if (getPropertyPower(p1) > getPropertyPower(p2)) return true;
            if (getPropertyPower(p1) < getPropertyPower(p2)) return false;
            if (p2.type_name.find("const") != std::string::npos) return true;
            return false;
        }
    };
    

    struct PropertyFunctionCountFilter
    {
        bool operator()(const FunctionDesc& f) const
        {
          return (!f.params.size());
        }
    };

    struct ParameterHaveNotDefaultValue
    {
        bool operator()(const ParameterDesc &p) const
        {
            return (p.default_value.empty());
        }
    };

    template < unsigned int nbParam >
    struct PropertyFunctionFirstParameterNameFilter
    {
        PropertyFunctionFirstParameterNameFilter(std::string & type_name)
        :   _type_name(type_name)
        {}
        bool operator()(const FunctionDesc& f) const
        {
            if (f.params.size() < nbParam) return (true);
            if (f.params.front().type_specifier != _type_name) return (true);
            if (f.params.size() == nbParam) return (false);

            ParameterList::const_iterator it = f.params.begin();
            std::advance(it, nbParam);
            return (std::find_if(it, f.params.end(), ParameterHaveNotDefaultValue()) != f.params.end());
            
        }
        std::string & _type_name;
    };

    template < unsigned int nbParam >
    struct PropertyFunctionNumParameterWithDefaultFilter
    {
        bool operator()(const FunctionDesc& f) const
        {
          if (f.params.size() < nbParam) return (true);
          if (f.params.size() == nbParam) return (false);
          
          ParameterList::const_iterator it = f.params.begin();
          std::advance(it, nbParam - 1);
          return (std::find_if(it, f.params.end(), ParameterHaveNotDefaultValue()) != f.params.end());
        }
    };
    template < unsigned int nbParam >
    struct PropertyFunctionNumParameterFilter
    {
      bool operator()(const FunctionDesc& f) const
      {
            return (f.params.size() == nbParam);
      }
    };
  
}

void TypeRegistry::registerTypeDescription(const TypeDesc &td)
{
    // !!! HACK !!!
    if (td.type_name == "std::vector")
        return;

    tfmap_[td.declaring_file_name].push_back(td);
    registerSymbol(td.type_name);
    for (StringList::const_iterator i=td.enum_labels.begin(); i!=td.enum_labels.end(); ++i)
        registerSymbol(*i);
}
void TypeRegistry::registerTemplate(const TypeDesc &td)
{
    templates_.push_back(td);
    registerSymbol(td.type_name);
}

const TypeDesc *TypeRegistry::findTypeDescription(const std::string &type_name) const
{
    for (TypeMap::const_iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
        for (TypeList::const_iterator i=j->second.begin(); i!=j->second.end(); ++i)
            if (i->type_name == type_name) return &*i;
    return 0;
}

void TypeRegistry::consolidate()
{
	Notify::notice("qualify_type_names...");
    qualify_type_names();
	Notify::notice("collect_template_instances...");
    collect_template_instances();
	Notify::notice("instantiate_templates...");
    instantiate_templates();
	Notify::notice("fill_attributes...");
    fill_attributes();
	Notify::notice("define_properties...");
    define_properties();
	Notify::notice("define_properties...");
    fill_needed_headers();
}

void TypeRegistry::qualify_single_type(TypeDesc &td)
{
    if (!td.alias.empty())
    {
        std::string declname = TypeNameUtils::getNamespace(td.type_name);
        const TypeDesc *declaring = findTypeDescription(declname);
        if (!declaring)
        {
            Notify::warn("could not find description of type `" + declname + "' during TypeRegistry::qualify_type_names()");
        }
        else
        {
            TypeNameUtils::qualifyAllIdentifiers(td.alias, *declaring, *this);
        }
    }

    for (FunctionList::iterator k=td.methods.begin(); k!=td.methods.end(); ++k)
    {
        FunctionDesc &fd = *k;
        TypeNameUtils::qualifyAllIdentifiers(fd.name, td, *this);
        TypeNameUtils::qualifyAllIdentifiers(fd.return_type_specifier, td, *this);
        for (ParameterList::iterator k=fd.params.begin(); k!=fd.params.end(); ++k)
        {
            TypeNameUtils::qualifyAllIdentifiers(k->type_specifier, td, *this);
            TypeNameUtils::qualifyAllIdentifiers(k->default_value, td, *this);
        }
    }

    for (FunctionList::iterator k=td.prot_methods.begin(); k!=td.prot_methods.end(); ++k)
    {
        FunctionDesc &fd = *k;
                
        TypeNameUtils::qualifyAllIdentifiers(fd.name, td, *this);
        TypeNameUtils::qualifyAllIdentifiers(fd.return_type_specifier, td, *this);
        for (ParameterList::iterator k=fd.params.begin(); k!=fd.params.end(); ++k)
        {
            TypeNameUtils::qualifyAllIdentifiers(k->type_specifier, td, *this);
            TypeNameUtils::qualifyAllIdentifiers(k->default_value, td, *this);
        }
    }

    for (AttributList::iterator k=td.public_attrib.begin(); k!=td.public_attrib.end(); ++k)
    {
        AttributDesc &ad = *k;
        TypeNameUtils::qualifyAllIdentifiers(ad.name, td, *this);
        TypeNameUtils::qualifyAllIdentifiers(ad.type, td, *this);
    }
}

void TypeRegistry::qualify_type_names()
{
    for (TypeMap::iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
    {
        for (TypeList::iterator i=j->second.begin(); i!=j->second.end(); ++i)
            qualify_single_type(*i);
    }
}

void TypeRegistry::fill_attributes()
{
    for (TypeMap::iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
    {
        for (TypeList::iterator i=j->second.begin(); i!=j->second.end(); ++i)
        {
            if (!i->is_abstract && has_pure_methods(*i))
            {
                i->is_abstract = true;
            }

            const ReflectorOptions *opt = cfg_.getReflectorOptions(i->type_name);
            if (opt)
            {
                for (std::set<std::string>::const_iterator k=opt->added_base_types.begin(); k!=opt->added_base_types.end(); ++k)
                {
                    Notify::info("adding base type `" + *k + "' to type `" + i->type_name + "' on user request");
                    // ** FIXME david callu 2006/09/29
                    // ** why comment this test ??
                    // ** it avoid duplicate base_type in reflector when the user add one explicitly in the conf file 
                    //if (std::find(i->base_type_names.begin(), i->base_type_names.end(), *k) == i->base_type_names.end())
                        i->base_types.push_back(*k);
                }
            }
        }
    }
}

bool TypeRegistry::has_pure_methods(const TypeDesc &td) const
{
    FunctionList pure_methods;
    collect_pure_methods(td, pure_methods);
    return !pure_methods.empty();
}

void TypeRegistry::collect_pure_methods(const TypeDesc &td, FunctionList &fl) const
{
    for (BaseTypeList::const_iterator i=td.base_types.begin(); i!=td.base_types.end(); ++i)
    {
        const TypeDesc *base = findTypeDescription(i->name);
        if (base)
            collect_pure_methods(*base, fl);
    }

    FunctionList methods;
    std::copy(td.methods.begin(), td.methods.end(), std::back_inserter(methods));
    std::copy(td.prot_methods.begin(), td.prot_methods.end(), std::back_inserter(methods));
    std::copy(td.priv_methods.begin(), td.priv_methods.end(), std::back_inserter(methods));

    for (FunctionList::const_iterator i=methods.begin(); i!=methods.end(); ++i)
    {
        if (i->is_pure_virtual)
        {
            fl.push_back(*i);
        }
        else
        {
            for (FunctionList::iterator j=fl.begin(); j!=fl.end();)
            {
                if (j->get_signature() == i->get_signature())
                {
                    fl.erase(j);
                }
                else
                    ++j;
            }
        }
    }
}

void TypeRegistry::define_properties()
{
    for (TypeMap::iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
    {
        for (TypeList::iterator i=j->second.begin(); i!=j->second.end(); ++i)
        {
            typedef std::map<std::string, FunctionList> NameFunctionMap;
            typedef std::map<std::string, NameFunctionMap> FunctionMap;
            typedef std::map<std::size_t, FunctionMap> IndexFunctionMap;

            typedef std::map<std::string, PropertyList> PropertyMap;
            PropertyMap newprops;

            NameFunctionMap count_candidates;
            FunctionMap add_candidates;
            FunctionMap insert_candidates;
            IndexFunctionMap remove_candidates;
            IndexFunctionMap get_candidates;
            IndexFunctionMap set_candidates;

            typedef std::map<std::string, StringSet>    NameTypeMap;
            NameTypeMap nameTypeMap;
            std::size_t max_indices = 0;

            for (FunctionList::const_iterator k=i->methods.begin(); k!=i->methods.end(); ++k)
            {
                if (!k->is_constructor(*i) && !k->is_destructor() && !k->is_static)
                {
                    if (begins_with(k->name, "getOr") && k->name.size() > 5 && std::isupper(k->name[5], std::locale()))
                        continue;

                    if (begins_with(k->name, "getNum") && k->name.size() > 6 && std::isupper(k->name[6], std::locale()))
                    {
                        if (k->params.empty())
                        {
                            std::string name(k->name.substr(6));
                            count_candidates[name].push_back(*k);
                            continue;
                        }
                    }

                    if (begins_with(k->name, "get") && ((k->name.size() > 3 && std::isupper(k->name[3], std::locale())) || (k->name.size() == 3)))
                    {
                        std::string name(k->name.substr(3));
                        std::size_t indices = k->params.size();
                        get_candidates[indices][k->return_type_specifier][name].push_back(*k);
                        if (indices > max_indices) max_indices = indices;
                        nameTypeMap[name].insert(k->return_type_specifier);
                        continue;
                    }

                    if (begins_with(k->name, "set") && k->name.size() > 3 && std::isupper(k->name[3], std::locale()))
                    {
                        if (!k->params.empty())
                        {
                            std::string name(k->name.substr(3));
                            std::size_t indices = k->params.size() - 1;
                            set_candidates[indices][k->params.back().type_specifier][name].push_back(*k);
                            if (indices > max_indices) max_indices = indices;
                            nameTypeMap[name].insert(k->params.back().type_specifier);
                        }
                        continue;
                    }

                    if (begins_with(k->name, "add") && k->name.size() > 3 && std::isupper(k->name[3], std::locale()))
                    {
                        if (k->params.size() == 1)
                        {
                            std::string name(k->name.substr(3));
                            add_candidates[k->params.front().type_specifier][name].push_back(*k);
                            nameTypeMap[name].insert(k->params.front().type_specifier);
                        }
                        continue;
                    }

                    if (begins_with(k->name, "remove") && k->name.size() > 6 && std::isupper(k->name[6], std::locale()))
                    {
                        if (!k->params.empty())
                        {
                            std::string name(k->name.substr(6));
                            std::size_t indices = k->params.size();
                            remove_candidates[indices][k->params.front().type_specifier][name].push_back(*k);
                        }
                        continue;
                    }
                    if (begins_with(k->name, "insert") && k->name.size() > 6 && std::isupper(k->name[6], std::locale()))
                    {
                        if (k->params.size() >= 2)
                        {
                            std::string name(k->name.substr(6));
                            
                            insert_candidates[k->params.front().type_specifier][name].push_back(*k);
                        }
                        continue;
                    }
                }
            }


            for (NameTypeMap::const_iterator k=nameTypeMap.begin(); k!=nameTypeMap.end(); ++k)
            {
                StringSet::const_iterator endIt = k->second.end();
                for (StringSet::const_iterator h=k->second.begin(); h!=endIt; ++h)
                {

                    PropertyDesc pd;
                    pd.name = k->first;
                    pd.type_name = *h;

                    // simple property
                    {
                        const FunctionList &fl_get  = get_candidates[0][*h][k->first];
                        FunctionList fl_set;
                        fl_set.insert(fl_set.end(), set_candidates[0][*h][k->first].begin(), set_candidates[0][*h][k->first].end());
                        fl_set.insert(fl_set.end(), set_candidates[0]["const " + *h + " &"][k->first].begin(), set_candidates[0]["const " + *h + " &"][k->first].end());

                        if (!fl_get.empty())    pd.get_method = fl_get.front().name_signature;
                        if (!fl_set.empty())    pd.set_method = fl_set.front().name_signature;
                        
                        if (!pd.get_method.empty() || !pd.set_method.empty())
                        {
                            pd.type = PropertyDesc::SIMPLE;
                            newprops[pd.name].push_back(pd);
                            continue;
                        }
                
                    }

                    // array property 
                    {
                        FunctionList fl_count;
                        fl_count.insert(fl_count.end(), count_candidates[k->first].begin(), count_candidates[k->first].end());
                        fl_count.insert(fl_count.end(), count_candidates[k->first + "s"].begin(), count_candidates[k->first + "s"].end());
                        fl_count.insert(fl_count.end(), count_candidates[k->first + "es"].begin(), count_candidates[k->first + "es"].end());
                        fl_count.insert(fl_count.end(), count_candidates[k->first + "ren"].begin(), count_candidates[k->first + "ren"].end());

                        if (fl_count.size())
                        {                

                            std::string& return_type_specifier = fl_count.front().return_type_specifier;

                            const FunctionList &fl_get = get_candidates[1][*h][k->first];
                            FunctionList fl_set;
                            fl_set.insert(fl_set.end(), set_candidates[1][*h][k->first].begin(), set_candidates[1][*h][k->first].end());
                            fl_set.insert(fl_set.end(), set_candidates[1]["const " + *h + " &"][k->first].begin(), set_candidates[1]["const " + *h + " &"][k->first].end());
                            const FunctionList &fl_add = add_candidates[*h][k->first];
                            FunctionList fl_remove;
                            fl_remove.insert(fl_remove.end(), remove_candidates[1][return_type_specifier][k->first].begin(), remove_candidates[1][return_type_specifier][k->first].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[1][return_type_specifier][k->first + "s"].begin(), remove_candidates[1][return_type_specifier][k->first + "s"].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[1][return_type_specifier][k->first + "es"].begin(), remove_candidates[1][return_type_specifier][k->first + "es"].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[1][return_type_specifier][k->first + "ren"].begin(), remove_candidates[1][return_type_specifier][k->first + "ren"].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[2][return_type_specifier][k->first].begin(), remove_candidates[2][return_type_specifier][k->first].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[2][return_type_specifier][k->first + "s"].begin(), remove_candidates[2][return_type_specifier][k->first + "s"].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[2][return_type_specifier][k->first + "es"].begin(), remove_candidates[2][return_type_specifier][k->first + "es"].end());
                            fl_remove.insert(fl_remove.end(), remove_candidates[2][return_type_specifier][k->first + "ren"].begin(), remove_candidates[2][return_type_specifier][k->first + "ren"].end());
                            FunctionList fl_insert = insert_candidates[return_type_specifier][k->first];

                            
                            fl_insert.erase(std::remove_if(fl_insert.begin(), fl_insert.end(), PropertyFunctionFirstParameterNameFilter<2>(fl_count.front().return_type_specifier)), fl_insert.end());
                            fl_remove.erase(std::remove_if(fl_remove.begin(), fl_remove.end(), PropertyFunctionFirstParameterNameFilter<1>(fl_count.front().return_type_specifier)), fl_remove.end());

                            if (!fl_get.empty())        pd.get_method     = fl_get.front().name_signature;
                            if (!fl_set.empty())        pd.set_method     = fl_set.front().name_signature;
                            if (!fl_add.empty())        pd.add_method     = fl_add.front().name_signature;
                            if (!fl_insert.empty())     pd.insert_method  = fl_insert.front().name_signature;
                            if (!fl_remove.empty())     pd.remove_method  = fl_remove.front().name_signature;
                            if (!fl_count.empty())      pd.count_method   = fl_count.front().name_signature;
                            
                            if (!pd.get_method.empty() || !pd.set_method.empty())
                            {
                                pd.type = PropertyDesc::ARRAY;
                                newprops[pd.name].push_back(pd);
                                continue;
                            }
                        }
                    }

                    // indexed property
                    for (std::size_t u=1; u<=max_indices; ++u)
                    {
                        const FunctionList &fl_get = get_candidates[u][*h][k->first];
                        FunctionList fl_set;
                        fl_set.insert(fl_set.end(), set_candidates[u][*h][k->first].begin(), set_candidates[u][*h][k->first].end());
                        fl_set.insert(fl_set.end(), set_candidates[u]["const " + *h + " &"][k->first].begin(), set_candidates[u]["const " + *h + " &"][k->first].end());
                     
                        if (!fl_get.empty())        pd.get_method     = fl_get.front().name_signature;
                        if (!fl_set.empty())        pd.set_method     = fl_set.front().name_signature;


                        for (FunctionList::const_iterator x=fl_get.begin(); x!=fl_get.end(); ++x)
                        {
                            ParameterList get_indices(x->params.begin(), x->params.end());
                            for (FunctionList::const_iterator y=fl_set.begin(); y!=fl_set.end(); ++y)
                            {
                                ParameterList set_indices(y->params.begin(), y->params.end());
                                set_indices.pop_back();
                                if (same_parameters(get_indices, set_indices))
                                {
                                    pd.type = PropertyDesc::INDEXED;
                                    pd.get_method = x->name_signature;
                                    pd.set_method = y->name_signature;
                                    newprops[pd.name].push_back(pd);
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            for (PropertyMap::iterator k=newprops.begin(); k!=newprops.end(); ++k)
            {
                if (!k->second.empty())
                {
                    std::sort(k->second.begin(), k->second.end(), PropertySorter());

                    PropertyDesc & pd = k->second.front();
                    const PropertyOptions *opt = cfg_.getPropertyOptions(i->type_name, pd.name);

                    if (opt)
                    {
                        if (!opt->get_method.empty())       pd.get_method       = opt->get_method;
                        if (!opt->set_method.empty())       pd.set_method       = opt->set_method;
                        if (!opt->remove_method.empty())    pd.remove_method    = opt->remove_method;
                        if (!opt->add_method.empty())       pd.add_method       = opt->add_method;
                        if (!opt->insert_method.empty())    pd.insert_method    = opt->insert_method;
                        if (!opt->count_method.empty())
                        {
                            Notify::info("define count method of property `" + pd.name + "' in type `" + i->type_name + "' on user request");
                            pd.type = PropertyDesc::ARRAY;
                            pd.count_method = opt->count_method;
                        }
                    }

                    i->properties.push_back(k->second.front());
                }
            }
        }
    }
}

void TypeRegistry::instantiate_templates()
{
    for (StringSetMap::const_iterator j=temp_inst_.begin(); j!=temp_inst_.end(); ++j)
    {
        for (StringSet::const_iterator i=j->second.begin(); i!=j->second.end(); ++i)
        {
            TypeDesc newtd;
            if (create_template_instance(*i, newtd))
                tfmap_[j->first].push_back(newtd);
        }
    }
}

const TypeDesc *TypeRegistry::findTemplate(const std::string &type_name) const
{
    for (TypeList::const_iterator i=templates_.begin(); i!=templates_.end(); ++i)
        if (i->type_name == type_name) return &*i;
    return 0;
}

bool TypeRegistry::create_template_instance(const std::string &decl, TypeDesc &newtd)
{
    std::string name;
    StringList params;
    if (TypeNameUtils::splitTemplateDeclaration(decl, name, params))
    {
        const TypeDesc *td = findTemplate(name);
        if (td)
        {
            // unwind typedefs
            if (!td->alias.empty())
                return create_template_instance(td->alias, newtd);

            std::string qdecl(decl);
            TypeNameUtils::qualifyAllIdentifiers(qdecl, *td, *this);
            
            newtd = *td;
            qualify_single_type(newtd);
            if (translate_template(newtd, qdecl, params))
            {
                newtd.type_name = qdecl;                
                //Notify::info("qualifying `" + newtd.type_name + "'");
                qualify_single_type(newtd);
                return true;
            }
            return false;
        }
        newtd.type_name = decl;
        newtd.undefined = true;
        return true;
    }
    return false;
}

bool TypeRegistry::translate_template(TypeDesc &td, const std::string &decl, const StringList &params)
{
    StringList values;
    ParameterList::const_iterator i = td.template_params.begin();
    StringList::const_iterator j = params.begin();

    translate_identifier(td, td.type_name, decl);

    while (i!=td.template_params.end())
    {
        std::string name;

        if (j!=params.end())
            name = *j;
        else
            name = i->default_value;

        if (name.empty())
            return false;

        translate_identifier(td, i->name, name);

        ++i;
        if (j!=params.end())
            ++j;
    }

    return true;
}

void TypeRegistry::translate_identifier(TypeDesc &td, const std::string &src, const std::string &dest)
{
    for (FunctionList::iterator i=td.methods.begin(); i!=td.methods.end(); ++i)
    {
        TypeNameUtils::translateIdentifiers(i->name, src, dest);
        TypeNameUtils::translateIdentifiers(i->return_type_specifier, src, dest);
        for (ParameterList::iterator j=i->params.begin(); j!=i->params.end(); ++j)
        {
            TypeNameUtils::translateIdentifiers(j->type_specifier, src, dest);
            TypeNameUtils::translateIdentifiers(j->default_value, src, dest);
        }
    }
    for (FunctionList::iterator i=td.prot_methods.begin(); i!=td.prot_methods.end(); ++i)
    {
        TypeNameUtils::translateIdentifiers(i->name, src, dest);
        TypeNameUtils::translateIdentifiers(i->return_type_specifier, src, dest);
        for (ParameterList::iterator j=i->params.begin(); j!=i->params.end(); ++j)
        {
            TypeNameUtils::translateIdentifiers(j->type_specifier, src, dest);
            TypeNameUtils::translateIdentifiers(j->default_value, src, dest);
        }
    }
    for (AttributList::iterator i=td.public_attrib.begin(); i!=td.public_attrib.end(); ++i)
        TypeNameUtils::translateIdentifiers(i->type, src, dest);
}

void TypeRegistry::collect_template_instances()
{
    typedef std::set<std::string> StringSet;
    StringSet decls;

    for (TypeMap::iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
    {
    	//Notify::debug("doing: " + j->first);

        // search for user-defined instances
        const FileOptions *fo = cfg_.getFileOptions(j->first);

        //Notify::debug("AAA");

        if (fo)
        {
            for (StringSet::const_iterator i=fo->templ_instances.begin(); i!=fo->templ_instances.end(); ++i)
            {
                Notify::info("instantiating template `" + *i + "' on user request");
                recurse_template_instances(j->first, *i, decls);
            }
        }

        //Notify::debug("BBB");

        for (TypeList::const_iterator i=j->second.begin(); i!=j->second.end(); ++i)
        {

            // examine typedef declaration
            if (!i->alias.empty())
            {
                recurse_template_instances(j->first, i->alias, decls);
                continue;
            }

            // examine base types
            for (BaseTypeList::const_iterator k=i->base_types.begin(); k!=i->base_types.end(); ++k)
                recurse_template_instances(j->first, k->name, decls);

            // iterate through methods
            for (FunctionList::const_iterator k=i->methods.begin(); k!=i->methods.end(); ++k)
            {
                // examine method's return type declaration
                if (!k->return_type_specifier.empty())
                    recurse_template_instances(j->first, k->return_type_specifier, decls);

                // examine parameters' type declarations
                for (ParameterList::const_iterator h=k->params.begin(); h!=k->params.end(); ++h)
                {
                    if (!h->type_specifier.empty())
                        recurse_template_instances(j->first, h->type_specifier, decls);
                }
            }
        }
    }
}

void TypeRegistry::recurse_template_instances(const std::string &declfile, const std::string &tdecl, std::set<std::string> &tempset)
{
    std::string decl = TypeNameUtils::removeModifiersFromSpecifier(tdecl);

    if (tempset.find(decl) != tempset.end())
        return;

    StringList params;
    std::string name;
    if (TypeNameUtils::splitTemplateDeclaration(decl, name, params))
    {
        temp_inst_[declfile].insert(decl);
        tempset.insert(decl);
        for (StringList::const_iterator i=params.begin(); i!=params.end(); ++i)
        {
            recurse_template_instances(declfile, *i, tempset);
        }
    }
}

bool TypeRegistry::same_parameters(const ParameterList &p1, const ParameterList &p2) const
{
    if (p1.size() != p2.size()) return false;
    ParameterList::const_iterator j=p2.begin();
    for (ParameterList::const_iterator i=p1.begin(); i!=p1.end(); ++i, ++j)
    {
        if (TypeNameUtils::trim(i->type_specifier) != TypeNameUtils::trim(j->type_specifier))
            return false;
    }
    return true;
}


void TypeRegistry::fill_needed_headers()
{
    for (TypeMap::iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
    {
        for (TypeList::iterator i=j->second.begin(); i!=j->second.end(); ++i)
        {
            for (FunctionList::iterator k=i->methods.begin(); k!=i->methods.end(); ++k)
            {
                add_referenced_types(k->return_type_specifier, i->needed_headers);
                for (ParameterList::const_iterator h=k->params.begin(); h!=k->params.end(); ++h)
                {
                    add_referenced_types(h->type_specifier, i->needed_headers);
                }
            }
        }
    }
}

void TypeRegistry::add_referenced_types(const std::string &decl, StringList &out) const
{
    TypeNameUtils::TokenList tokens;
    TypeNameUtils::tokenize(decl, tokens);
    for (TypeNameUtils::TokenList::const_iterator i=tokens.begin(); i!=tokens.end(); ++i)
    {
        if (i->type == TypeNameUtils::Token::IDENTIFIER)
        {
            std::string fn = getDeclaringFileName(decl.substr(i->beg, i->len));
            if (!fn.empty())
            {
                if (std::find(out.begin(), out.end(), fn) == out.end())
                    out.push_back(fn);
            }
        }
    }
}

std::string TypeRegistry::getDeclaringFileName(const std::string &type_name) const
{
    for (TypeMap::const_iterator j=tfmap_.begin(); j!=tfmap_.end(); ++j)
        for (TypeList::const_iterator i=j->second.begin(); i!=j->second.end(); ++i)
            if (i->type_name == type_name) return j->first;
    return std::string();
}
