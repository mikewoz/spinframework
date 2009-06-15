#ifndef REGISTRYBUILDER_H_
#define REGISTRYBUILDER_H_

#include "NodeMap.h"
#include "TypeRegistry.h"
#include "Configuration.h"

class RegistryBuilder
{
public:
    RegistryBuilder(NodeMap &nmap, TypeRegistry &reg, const Configuration &cfg);

    void build();

protected:
    void process_index_element(const std::string &name, const XML::Node *node);
    void process_compound(const std::string &id, const XML::Node *node);
    void process_help(const XML::Node *node, std::string& help);
    bool process_class(const XML::Node *node, TypeDesc &desc, bool istemplate);
    bool process_struct(const XML::Node *node, TypeDesc &desc, bool istemplate);
    bool process_enum(const XML::Node *node, TypeDesc &desc, const std::string &declname);
    bool process_typedef(const XML::Node *node, TypeDesc &desc);
    bool process_function(const XML::Node *node, FunctionDesc &desc);
    void process_type_section(const XML::Node *node, TypeDesc &desc, bool istemplate);
    void process_func_section(const XML::Node *node, TypeDesc &desc);
    void process_prot_func_section(const XML::Node *node, TypeDesc &desc);
    void process_priv_func_section(const XML::Node *node, TypeDesc &desc);
    void process_public_attrib_section(const XML::Node *node, TypeDesc &desc);
    bool process_namespace(const XML::Node *node, TypeDesc &desc, bool istemplate);
    void process_var_section(const XML::Node *node, TypeDesc &desc);
    void process_template_parameters(const XML::Node *node, TypeDesc &desc);

private:
    NodeMap &nmap_;
    TypeRegistry &reg_;
    const Configuration &cfg_;
};

#endif
