#ifndef CONFIGURATION_
#define CONFIGURATION_

#include <string>
#include <vector>
#include <iterator>
#include <set>
#include <map>
#include <stack>

#include <boost/regex.hpp>

#include "PathNameUtils.h"

class StringMatcher
{
public:
    StringMatcher(const std::string &s);
    bool matches(const std::string &s) const;
    bool operator==(const std::string &s) const { return matches(s); }

private:
    std::string literal;
    boost::regex re;
};

struct OptionBase
{
    std::string pattern;

    bool entirely_replaced;
    std::string replace_text;

    std::string header_text;
    std::string footer_text;

    std::vector<std::string> attributes;

    OptionBase(): entirely_replaced(false) {}
};

struct LibraryOptions: OptionBase
{
    std::set<std::string> win32_debug_dependencies;
    std::set<std::string> win32_release_dependencies;
    std::set<std::string> unix_dependencies;
};

struct FileOptions: OptionBase
{
    std::set<std::string> templ_instances;
};

struct MethodOptions: OptionBase
{
};

struct ConstructorOptions: OptionBase
{
};

struct PropertyOptions: OptionBase
{
    std::string get_method;
    std::string set_method;
    std::string count_method;
    std::string add_method;
    std::string insert_method;
    std::string remove_method;
};

struct ReflectorOptions: OptionBase
{
    enum TypeKind
    {
        DEFAULT_TYPE,
        OBJECT_TYPE,
        ABSTRACT_OBJECT_TYPE,
        ENUM_TYPE,
        VALUE_TYPE,
        STD_VALUE,
        STD_MAP,
        STD_VECTOR,
        STD_SET,
        STD_LIST,
        STD_PAIR
    };

    TypeKind type_kind;

    std::string comparator;
    bool comparator_specified;

    std::string readerwriter;
    bool readerwriter_specified;

    typedef std::vector<std::pair<StringMatcher, ConstructorOptions> > ConstructorOptionList;
    ConstructorOptionList constructor_opts;

    typedef std::vector<std::pair<StringMatcher, MethodOptions> > MethodOptionList;
    MethodOptionList method_opts;

    typedef std::vector<std::pair<StringMatcher, PropertyOptions> > PropertyOptionList;
    PropertyOptionList property_opts;

    std::set<std::string> added_text;
    std::set<std::string> suppressed_text;

    std::set<std::string> added_base_types;

    ReflectorOptions()
    :    type_kind(DEFAULT_TYPE),
        comparator_specified(false),
        readerwriter_specified(false)
    {
    }
};


class Configuration
{
public:
    Configuration();

    inline const std::string &getRelativePathDelimiter() const;
    inline void setRelativePathDelimiter(const std::string &s);

    bool load(const std::string &filename);

    bool isFileIgnored(const std::string &s) const;
    bool isTypeIgnored(const std::string &s) const;
    bool isTemplateIgnored(const std::string &s) const;
    bool isFileSuppressed(const std::string &s) const;
    bool isReflectorSuppressed(const std::string &s) const;
    inline const LibraryOptions *getLibraryOptions(const std::string &s) const;
    inline const FileOptions *getFileOptions(const std::string &s) const;
    inline const ReflectorOptions *getReflectorOptions(const std::string &s) const;
    inline const MethodOptions *getMethodOptions(const std::string &t, const std::string &s) const;
    inline const PropertyOptions *getPropertyOptions(const std::string &t, const std::string &s) const;

    void path_delimiter(const std::string &s);
    void ignore_file(const std::string &s);
    void ignore_type(const std::string &s);
    void ignore_template(const std::string &s);
    void suppress_file(const std::string &s);
    void suppress_reflector(const std::string &s);
    void configure_library(const std::string &s);
    void configure_file(const std::string &s);
    void configure_reflector(const std::string &s);
    void configure_constructor(const std::string &s);
    void configure_method(const std::string &s);
    void configure_property(const std::string &s);
    void replace_with(const std::string &s);
    void emit_before(const std::string &s);
    void emit_after(const std::string &s);
    void instantiate_template(const std::string &s);
    void add_base_type(const std::string &s);
    void set_rw(const std::string &s);
    void set_cmp(const std::string &s);
    void add_attribute(const std::string &s);
    void end_block(const std::string &);    
    void value_type(const std::string &);
    void object_type(const std::string &);
    void abstract_object_type(const std::string &);
    void dependency_unix(const std::string &);
    void dependency_win32(const std::string &);
    void dependency_win32d(const std::string &);
    void get_method(const std::string & s);
    void set_method(const std::string & s);
    void count_method(const std::string & s);
    void add_method(const std::string & s);
    void insert_method(const std::string & s);
    void remove_method(const std::string & s);

private:
    typedef std::vector<StringMatcher> MatcherList;

    MatcherList suppressed_files_;
    MatcherList suppressed_refs_;
    MatcherList ignored_files_;
    MatcherList ignored_types_;
    MatcherList ignored_templ_;

    typedef std::vector<std::pair<StringMatcher, ReflectorOptions> > ReflectorOptionList;
    ReflectorOptionList ref_opts_;

    typedef std::vector<std::pair<StringMatcher, FileOptions> > FileOptionList;
    FileOptionList file_opts_;

    typedef std::vector<std::pair<StringMatcher, LibraryOptions> > LibraryOptionList;
    LibraryOptionList lib_opts_;

    enum Context
    {
        GLOBAL,
        LIBRARY,
        FILE,
        REFLECTOR,
        CONSTRUCTOR,
        METHOD,
        PROPERTY
    };

    void error(const std::string &msg) const;

    inline Context &context()
    {
        return cstack_.top();
    }

    typedef std::stack<Context> ContextStack;
    ContextStack cstack_;

    std::string path_delim_;
};

// INLINE METHODS

inline const std::string &Configuration::getRelativePathDelimiter() const
{
    return path_delim_;
}

inline void Configuration::setRelativePathDelimiter(const std::string &s)
{
    path_delim_ = PathNameUtils::consolidateDelimiters(s, false);
}

inline const LibraryOptions *Configuration::getLibraryOptions(const std::string &s) const
{
    for (LibraryOptionList::const_iterator i=lib_opts_.begin(); i!=lib_opts_.end(); ++i)
        if (i->first.matches(s)) return &i->second;
    return 0;
}

inline const FileOptions *Configuration::getFileOptions(const std::string &s) const
{
    for (FileOptionList::const_iterator i=file_opts_.begin(); i!=file_opts_.end(); ++i)
        if (i->first.matches(s)) return &i->second;
    return 0;
}

inline const ReflectorOptions *Configuration::getReflectorOptions(const std::string &s) const
{
    for (ReflectorOptionList::const_iterator i=ref_opts_.begin(); i!=ref_opts_.end(); ++i)
        if (i->first.matches(s)) return &i->second;
    return 0;
}

inline const MethodOptions *Configuration::getMethodOptions(const std::string &t, const std::string &s) const
{
    for (ReflectorOptionList::const_iterator i=ref_opts_.begin(); i!=ref_opts_.end(); ++i)
    {
        if (i->first.matches(t)) 
        {
            for (ReflectorOptions::MethodOptionList::const_iterator j=i->second.method_opts.begin(); j!=i->second.method_opts.end(); ++j)
            {
                if (j->first.matches(s))
                    return &j->second;
            }
        }
    }
    return 0;
}

inline const PropertyOptions *Configuration::getPropertyOptions(const std::string &t, const std::string &s) const
{
    for (ReflectorOptionList::const_iterator i=ref_opts_.begin(); i!=ref_opts_.end(); ++i)
    {
        if (i->first.matches(t)) 
        {
            for (ReflectorOptions::PropertyOptionList::const_iterator j=i->second.property_opts.begin(); j!=i->second.property_opts.end(); ++j)
            {
                if (j->first.matches(s))
                    return &j->second;
            }
        }
    }
    return 0;
}

#endif
