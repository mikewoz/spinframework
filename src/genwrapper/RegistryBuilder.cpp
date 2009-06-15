#include "RegistryBuilder.h"
#include "TypeNameUtils.h"
#include "Notify.h"

#include <boost/algorithm/string.hpp>

RegistryBuilder::RegistryBuilder(NodeMap &nmap, TypeRegistry &reg, const Configuration &cfg)
:    nmap_(nmap),
    reg_(reg),
    cfg_(cfg)
{
}

void RegistryBuilder::build()
{
    const XML::Node *index = nmap_.getIndexNode();
    if (!index)
        Notify::error("invalid XML index");

    for (XML::Node::Element_map::const_iterator i=index->elements().begin(); i!=index->elements().end(); ++i)
    {
        process_index_element(i->first, i->second);
    }
}

void RegistryBuilder::process_index_element(const std::string &name, const XML::Node *node)
{
    if (name == "compound")
    {
        std::string refid = node->get_attribute("refid");
        if (refid.empty())
        {
            Notify::warn("invalid reference ID in compound index element");
        }
        else
        {
            const XML::Node *ref = nmap_.findNode(refid);
            if (!ref)
            {
                Notify::warn("could not find element `" + refid + "'");
            }
            else
            {
                std::string prot = ref->get_attribute("prot");
                if (prot.empty() || prot == "public" /* || prot == "protected" */)
                    process_compound(refid, ref);
            }
        }
    }
}

void RegistryBuilder::process_compound(const std::string &id, const XML::Node *node)
{
    // early reject
    std::string kind = node->get_attribute("kind");
    if (kind != "struct" &&
        kind != "class" &&
        kind != "namespace")
        return;

    TypeDesc desc;

    desc.type_name = node->get_first_element_content("compoundname");
    if (desc.type_name.empty())
    {
        Notify::warn("compound element `" + id + "' has no name, skipping");
        return;
    }
    
    const XML::Node *locnode = node->get_first_element("location");
    if (!locnode)
    {
        Notify::warn("compound element `" + id + "' has no location, skipping");
        return;
    }

    desc.declaring_file_name = PathNameUtils::getRelativePathAfter(locnode->get_attribute("file"), cfg_.getRelativePathDelimiter());
    if (desc.declaring_file_name.empty())
    {
        Notify::warn("compound element `" + id + "' has an empty location string, skipping");
        return;
    }

    if (cfg_.isFileIgnored(desc.declaring_file_name))
    {
        Notify::info("ignoring file `" + desc.declaring_file_name + "' on user request");
        return;
    }

    const XML::Node *tparamnode = node->get_first_element("templateparamlist");
    if (tparamnode)
    {
        if (cfg_.isTemplateIgnored(desc.type_name))
        {
            Notify::info("ignoring template `" + desc.type_name + "' on user request");
            return;
        }
        process_template_parameters(tparamnode, desc);
    }

    if (cfg_.isTypeIgnored(desc.type_name)) 
    {
        Notify::info("ignoring type `" + desc.type_name + "' on user request");
        return;
    }

    bool sreg = false;
    if (kind == "struct")
        sreg = process_struct(node, desc, tparamnode != 0);
    if (kind == "class")
        sreg = process_class(node, desc, tparamnode != 0);
    if (kind == "namespace")
        sreg = process_namespace(node, desc, tparamnode != 0);

    if (sreg)
    {
        if (tparamnode)
            reg_.registerTemplate(desc);
        else
            reg_.registerTypeDescription(desc);
    }
}

void RegistryBuilder::process_help(const XML::Node *node, std::string& help)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "para")
        {
            if (i->second->has_elements("parameterlist"))
            {
                for (XML::Node::Element_map::const_iterator j=i->second->elements().begin(); j!=i->second->elements().end(); ++j)
                {
                    if (j->first == "parameterlist")
                    {
                        help += " " + j->second->get_attribute("kind") + " ";
                        for (XML::Node::Element_map::const_iterator k=j->second->elements().begin(); k!=j->second->elements().end(); ++k)
                        {
                            if (k->first == "parameteritem")
                            {
                                std::string itemName;
                                std::string itemDesc;
                                for (XML::Node::Element_map::const_iterator l=k->second->elements().begin(); l!=k->second->elements().end(); ++l)
                                {
                                    if (l->first == "parameternamelist")
                                    {
                                        for (XML::Node::Element_map::const_iterator m=l->second->elements().begin(); m!=l->second->elements().end(); ++m)
                                        {
                                            if (m->first == "parametername")
                                                itemName = m->second->get_content() + " ";
                                        }
                                    }
                                    if (l->first == "parameterdescription")
                                    {
                                        for (XML::Node::Element_map::const_iterator m=l->second->elements().begin(); m!=l->second->elements().end(); ++m)
                                        {
                                            if (m->first == "para")
                                                itemDesc = m->second->get_content() + " ";
                                        }
                                    }
                                }
                                help += itemName + itemDesc;
                            }
                        }
                    }
                    if (j->first == "simplesect")
                    {
                        help += " " + j->second->get_attribute("kind") + " ";
                        for (XML::Node::Element_map::const_iterator k=j->second->elements().begin(); k!=j->second->elements().end(); ++k)
                        {
                            if (k->first == "para")
                                help += k->second->get_content();
                        }
                    }
                }
            }
            else
                help += i->second->get_content();
        }
    }
    boost::algorithm::replace_all(help, "\"", "\\\"");
    boost::algorithm::replace_all(help, "\n", " ");
}

bool RegistryBuilder::process_class(const XML::Node *node, TypeDesc &desc, bool istemplate)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "basecompoundref" && i->second->get_attribute("prot") == "public")
        {
            std::string refid = i->second->get_attribute("refid");
            if (refid.empty())
            {
                Notify::warn("invalid reference ID in base compound element");
            }
            else
            {
                const XML::Node *ref = nmap_.findNode(refid);
                if (!ref)
                {
                    Notify::warn("could not find base compound element `" + refid + "'");
                }
                else
                {
                    desc.base_types.push_back(BaseTypeDesc(ref->get_first_element_content("compoundname"), i->second->get_attribute("virt") == "virtual"));
                }
            }
        }
    
        if (i->first == "briefdescription")
            process_help(i->second, desc.briefHelp);
    
        if (i->first == "detaileddescription")
            process_help(i->second, desc.detailedHelp);
        
        if (i->first == "sectiondef")
        {
            std::string kind = i->second->get_attribute("kind");
            if (kind == "var")
                process_var_section(i->second, desc);
            if (kind == "public-type" || kind == "typedef" || kind == "enum")
                process_type_section(i->second, desc, istemplate);
            if (kind == "public-func")
                process_func_section(i->second, desc);
            if (kind == "public-static-func")
                process_func_section(i->second, desc);
            if (kind == "protected-func")
                process_prot_func_section(i->second, desc);
            if (kind == "private-func")
                process_priv_func_section(i->second, desc);
            if (kind == "public-attrib")
                process_public_attrib_section(i->second, desc);
        }
    }

    return true;
}

bool RegistryBuilder::process_struct(const XML::Node *node, TypeDesc &desc, bool istemplate)
{
    return process_class(node, desc, istemplate);
}

bool RegistryBuilder::process_enum(const XML::Node *node, TypeDesc &desc, const std::string &declname)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "enumvalue")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty())
            {
                Notify::warn("skipping unnamed enum value in `" + desc.type_name + "'");
            }
            else
            {
                desc.enum_labels.push_back(declname + "::" + name);
            }
        }
    }
    return !desc.enum_labels.empty();
}

void RegistryBuilder::process_type_section(const XML::Node *node, TypeDesc &desc, bool istemplate)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "memberdef")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty() || name.find("@") != std::string::npos)
            {
                Notify::warn("skipping unnamed compound member defined in `" + desc.type_name + "'");
                continue;
            }

            TypeDesc newdesc;
            newdesc.type_name = desc.type_name + "::" + name;
            newdesc.template_params = desc.template_params;

            const XML::Node *locnode = i->second->get_first_element("location");
            if (!locnode)
            {
                Notify::warn("member `" + newdesc.type_name + "' has no location, skipping");
                continue;
            }

            newdesc.declaring_file_name = PathNameUtils::getRelativePathAfter(locnode->get_attribute("file"), cfg_.getRelativePathDelimiter());
            if (newdesc.declaring_file_name.empty())
            {
                Notify::warn("member `" + newdesc.type_name + "' has an empty location string, skipping");
                continue;
            }

            std::string kind = i->second->get_attribute("kind");
            bool sreg = false;
            if (kind == "enum")
                sreg = process_enum(i->second, newdesc, desc.type_name);
            if (kind == "typedef")
                sreg = process_typedef(i->second, newdesc);
            if (sreg)
            {
                if (istemplate)
                    reg_.registerTemplate(newdesc);
                else
                    reg_.registerTypeDescription(newdesc);
            }
        }
    }
}

void RegistryBuilder::process_func_section(const XML::Node *node, TypeDesc &desc)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "memberdef")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty())
            {
                Notify::warn("skipping unnamed compound member defined in `" + desc.type_name + "'");
            }
            else
            {
                std::string kind = i->second->get_attribute("kind");
                if (kind == "function")
                {
                    FunctionDesc fd;
                    fd.name = TypeNameUtils::getUnqualifiedIdentifier(name);    // some OSG functions are declared with namespace!
                    if (process_function(i->second, fd))
                    {
                        if (fd.is_constructor(desc))
                            desc.has_custom_constructors = true;
                        desc.methods.push_back(fd);
                    }
                }
            }
        }
    }
}

bool RegistryBuilder::process_function(const XML::Node *node, FunctionDesc &desc)
{
    // no templates for now
    if (node->get_first_element("templateparamlist") != 0)
        return false;

    desc.is_const = node->get_attribute("const") == "yes";
    desc.is_pure_virtual = node->get_attribute("virt") == "pure-virtual";
    desc.is_virtual = desc.is_pure_virtual || node->get_attribute("virt") == "virtual";
    desc.is_static = node->get_attribute("static") == "yes";
    desc.is_explicit = node->get_attribute("explicit") == "yes";
    desc.return_type_specifier = TypeNameUtils::trim(TypeNameUtils::removeGarbageFromSpecifier(node->get_first_element_content("type")));

    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        
        if (i->first == "briefdescription")
            process_help(i->second, desc.briefHelp);
    
        if (i->first == "detaileddescription")
            process_help(i->second, desc.detailedHelp);
    
        if (i->first == "param")
        {
            ParameterDesc pd;

            pd.default_value = i->second->get_first_element_content("defval");

            pd.name = i->second->get_first_element_content("declname");
            if (pd.name.empty())
                pd.name = "x";

            pd.type_specifier = TypeNameUtils::trim(i->second->get_first_element_content("type"));
            if (pd.type_specifier.empty())
            {
                Notify::warn("type of parameter `" + pd.name + "' in function `" + desc.name + "' is undefined");
            }
            else
            {
                if (pd.type_specifier != "void")
                    desc.params.push_back(pd);
            }
        }
    }

    desc.gen_name_signature();

    return true;
}

bool RegistryBuilder::process_typedef(const XML::Node *node, TypeDesc &desc)
{
    desc.alias = node->get_first_element_content("type");

    std::string::size_type apientry = desc.alias.find("APIENTRY");
    if (apientry != std::string::npos)
        desc.alias.erase(apientry, 8);

    if (desc.alias.empty())
    {
        Notify::warn("typedef `" + desc.type_name + "' has no type, skipping");
        return false;
    }
    return true;
}

void RegistryBuilder::process_priv_func_section(const XML::Node *node, TypeDesc &desc)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "memberdef")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty())
            {
                Notify::warn("skipping unnamed compound member defined in `" + desc.type_name + "'");
            }
            else
            {
                std::string kind = i->second->get_attribute("kind");
                if (kind == "function")
                {
                    FunctionDesc fd;
                    fd.name = TypeNameUtils::getUnqualifiedIdentifier(name);
                    if (process_function(i->second, fd))
                    {
                        if (fd.is_constructor(desc))
                            desc.has_custom_constructors = true;
                        if (fd.is_default_constructor(desc))
                            desc.default_constructor_private = true;
                        if (fd.is_destructor())
                            desc.destructor_private = true;
                        desc.priv_methods.push_back(fd);
                    }
                }
            }
        }
    }
}

void RegistryBuilder::process_prot_func_section(const XML::Node *node, TypeDesc &desc)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "memberdef")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty())
            {
                Notify::warn("skipping unnamed compound member defined in `" + desc.type_name + "'");
            }
            else
            {
                std::string kind = i->second->get_attribute("kind");
                if (kind == "function")
                {
                    FunctionDesc fd;
                    fd.name = TypeNameUtils::getUnqualifiedIdentifier(name);
                    if (process_function(i->second, fd))
                    {
                        if (fd.is_constructor(desc))
                            desc.has_custom_constructors = true;
                        if (fd.is_default_constructor(desc))
                            desc.default_constructor_private = true;
                        if (fd.is_destructor())
                            desc.destructor_private = true;
                        desc.prot_methods.push_back(fd);
                    }
                }
            }
        }
    }
}

bool RegistryBuilder::process_namespace(const XML::Node *node, TypeDesc &desc, bool istemplate)
{
    desc.is_namespace = true;
    return process_struct(node, desc, istemplate);
}

void RegistryBuilder::process_var_section(const XML::Node *node, TypeDesc &desc)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "memberdef")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty())
            {
                Notify::warn("skipping unnamed compound member defined in `" + desc.type_name + "'");
            }
            else
            {
                std::string kind = i->second->get_attribute("kind");
                if (kind == "variable")
                {
                    reg_.registerSymbol(desc.type_name + "::" + name);
                }
            }
        }
    }
}

void RegistryBuilder::process_public_attrib_section(const XML::Node *node, TypeDesc &desc)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "memberdef")
        {
            std::string name = i->second->get_first_element_content("name");
            if (name.empty())
            {
                Notify::warn("skipping unnamed compound member defined in `" + desc.type_name + "'");
            }
            else
            {
                std::string kind = i->second->get_attribute("kind");
                if (kind == "variable")
                {
                    
                    // ** pointer-to-reference-member is not supported in GCC 4.1.1
                    // **   fixe in GCC 4.1.2
                    // **   and for other compilator ???
                    // **   -->> pointer-to-reference-member not supported in osgIntrospection at this time
                    
                    // ** pointer to array member not yet implemented in osgIntrospection
                    // **   argsstring specify if there are a "[nb]" after the variable
                    // **   and so if the variable is an array

                    // ** pointer to const member not yet implemented in osgIntrospection
                    // **   
                    std::string type = i->second->get_first_element_content("type");
                    std::string arg  = i->second->get_first_element_content("argsstring");
                    if ((type.size()) && (type.find_first_of('&') == std::string::npos) && (type.find("const") == std::string::npos) && (arg.size() == 0))
                    {
                        AttributDesc ad(type, name);
                        desc.public_attrib.push_back(ad);
                    }
                    else if (type.find_first_of('&') != std::string::npos)
                        Notify::warn("skipping the public reference member `" + i->second->get_first_element_content("definition") + "'");
                    else if (type.find("const") != std::string::npos)
                        Notify::warn("skipping the public const member `" + i->second->get_first_element_content("definition") + "'");
                    else if (arg.size() == 0)
                        Notify::warn("skipping the public array member `" + i->second->get_first_element_content("definition") + "'");
                    else if (type.size() == 0)
                        Notify::warn("skipping the unknow public member `" + i->second->get_first_element_content("definition") + "'");
                }
            }
        }
    }
}

void RegistryBuilder::process_template_parameters(const XML::Node *node, TypeDesc &desc)
{
    for (XML::Node::Element_map::const_iterator i=node->elements().begin(); i!=node->elements().end(); ++i)
    {
        if (i->first == "param")
        {
            ParameterDesc pd;
            pd.type_specifier = i->second->get_first_element_content("type");
            pd.name = i->second->get_first_element_content("defname");
            pd.default_value= i->second->get_first_element_content("defval");
            if (pd.name.empty())
            {
                Notify::warn("skipping unnamed template parameter in `" + desc.type_name + "'");
                continue;
            }
            if (pd.type_specifier.empty())
            {
                Notify::warn("type of template parameter `" + pd.name + "' in compound `" + desc.type_name + "' is undefined");
                continue;
            }
            desc.template_params.push_back(pd);
        }
    }
}
