#ifndef WRAPPERGENERATOR_H_
#define WRAPPERGENERATOR_H_

#include "TypeRegistry.h"
#include "Configuration.h"

#include <string>
#include <set>
#include <iosfwd>

class WrapperGenerator
{
public:
    WrapperGenerator(const TypeRegistry &reg, const std::string &out_dir, const std::string &dst_dir, const std::string &appdir, bool create_lists, const Configuration &cfg);

    struct Statistics
    {
        int total_reflectors;
        int num_custom;
        int num_typedefs;
        int num_stdcontainers;
        int num_value_types;
        int num_object_types;
        int num_abstract_object_types;
        int num_enums;
        int total_methods;
        int total_constructors;
        int total_static;
        int total_protected;
        int total_properties;
        int num_public_properties;
        int num_simple_properties;
        int num_array_properties;
        int num_indexed_properties;

        Statistics()
        :    total_reflectors(0),
            num_custom(0),
            num_typedefs(0),
            num_stdcontainers(0),
            num_value_types(0),
            num_object_types(0),
            num_abstract_object_types(0),
            num_enums(0),
            total_methods(0),
            total_constructors(0),
            total_static(0),
            total_protected(0),
            total_properties(0),
            num_public_properties(0),
            num_simple_properties(0),
            num_array_properties(0),
            num_indexed_properties(0)
        {
        }
    };

    void generate() ;
    inline const Statistics &getStatistics() const   { return stats_; }

protected:
    void write_header(const std::set<std::string> &incl, std::ostream &os) const;
    void write_reflector(const TypeDesc &td, std::ostream &os) ;
    void write_method(const FunctionDesc &fd, const TypeDesc &td, bool is_protected, std::ostream &os) ;
    void write_property(const PropertyDesc &fd, const TypeDesc &td, std::ostream &os) ;
    void write_public_attribut(const AttributDesc &ad, const TypeDesc &td, std::ostream &os);
    void handle_undefined_type(const TypeDesc &td, std::ostream &os) ;
    std::string replace_commas(const std::string &s) const;
    void write_user_header(const OptionBase &opt, std::ostream &os);
    void write_user_footer(const OptionBase &opt, std::ostream &os);

    bool read_text_file(const std::string &filename, std::string &text) const;

    typedef std::map<std::string, std::vector<std::string> > FileMap;
    int compare_text_files(const std::string &filename, const std::string &content) const;
    void create_lists(const FileMap &file_map, const FileMap &created_files, const FileMap &modified_files);

private:
    const TypeRegistry &reg_;
    std::string dst_dir_;
    std::string app_dir_;
    const Configuration &cfg_;
    Statistics stats_;
    bool create_lists_;
};

#endif
