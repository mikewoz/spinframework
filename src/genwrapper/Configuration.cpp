#include "Configuration.h"
#include "Notify.h"

#include <boost/spirit/include/classic.hpp>
#include <boost/regex.hpp>

#include <algorithm>
#include <fstream>
#include <vector>
#include <iterator>

using namespace boost::spirit::classic;

struct config_grammar: public grammar<config_grammar>
{
    template<typename S>
    struct definition
    {        
        definition(const config_grammar &self)
        {
            delim = eol_p;

            comment = (ch_p('#') >> *(anychar_p - delim) >> +delim);

            quote = ch_p('"');
            re_quote = ch_p('/');

            escaped_char = ch_p('\\') >> anychar_p;

            literal = (quote >> *(escaped_char | (anychar_p - quote)) >> quote);
            regular_expr = (re_quote >> *(escaped_char | (anychar_p - re_quote)) >> re_quote);

            pattern = 
                    literal
                |   regular_expr
                ;            

            global_level_statement =
                    (str_p("path")          >> str_p("delimiter")    >> literal [proxy(self, &Configuration::path_delimiter)])
                |   (str_p("ignore")        >> str_p("file")         >> pattern [proxy(self, &Configuration::ignore_file)])
                |   (str_p("ignore")        >> str_p("type")         >> pattern [proxy(self, &Configuration::ignore_type)])
                |   (str_p("ignore")        >> str_p("template")     >> pattern [proxy(self, &Configuration::ignore_template)])
                |   (str_p("suppress")      >> str_p("file")         >> pattern [proxy(self, &Configuration::suppress_file)])
                |   (str_p("suppress")      >> str_p("reflector")    >> pattern [proxy(self, &Configuration::suppress_reflector)])
                ;

            library_level_statement =
                    (str_p("dependency")    >> str_p("unix")         >> literal [proxy(self, &Configuration::dependency_unix)])
                |   (str_p("dependency")    >> str_p("win32-release")>> literal [proxy(self, &Configuration::dependency_win32)])
                |   (str_p("dependency")    >> str_p("win32-debug")  >> literal [proxy(self, &Configuration::dependency_win32d)])
                ;

            file_level_statement =
                    (str_p("instantiate")   >> str_p("template")     >> literal [proxy(self, &Configuration::instantiate_template)])
                ;

            reflector_level_statement =
                     (str_p("add")           >> str_p("base")         >> literal [proxy(self, &Configuration::add_base_type)])
                |   (str_p("readerwriter")                           >> literal [proxy(self, &Configuration::set_rw)])
                |   (str_p("comparator")                             >> literal [proxy(self, &Configuration::set_cmp)])
                |   (str_p("value-type")                             [proxy(self, &Configuration::value_type)])
                |   (str_p("object-type")                            [proxy(self, &Configuration::object_type)])
                |   (str_p("abstract-object-type")                   [proxy(self, &Configuration::abstract_object_type)])
                ;

            property_level_statement =
                    (str_p("get_method")   >> literal [proxy(self, &Configuration::get_method)])
                |   (str_p("set_method")   >> literal [proxy(self, &Configuration::set_method)])
                |   (str_p("count_method")   >> literal [proxy(self, &Configuration::count_method)])
                |   (str_p("add_method")   >> literal [proxy(self, &Configuration::add_method)])
                |   (str_p("insert_method")   >> literal [proxy(self, &Configuration::insert_method)])
                |   (str_p("remove_method")   >> literal [proxy(self, &Configuration::remove_method)])
                ;

            line_statement = 
                (
                    global_level_statement
                |   library_level_statement
                |   file_level_statement
                |   reflector_level_statement
                |   property_level_statement

                    // multiple levels
                |   (str_p("replace")       >> str_p("with")         >> literal [proxy(self, &Configuration::replace_with)])
                |   (str_p("emit")          >> str_p("before")       >> literal [proxy(self, &Configuration::emit_before)])
                |   (str_p("emit")          >> str_p("after")        >> literal [proxy(self, &Configuration::emit_after)])
                |   (str_p("attribute")                              >> literal [proxy(self, &Configuration::add_attribute)])
                ) >> +delim
                ;

            block_statement =
                (
                    (str_p("configure")     >> str_p("library")      >> pattern [proxy(self, &Configuration::configure_library)])
                |    (str_p("configure")     >> str_p("file")         >> pattern [proxy(self, &Configuration::configure_file)])
                |   (str_p("configure")     >> str_p("reflector")    >> pattern [proxy(self, &Configuration::configure_reflector)])
                |   (str_p("configure")     >> str_p("constructor")  >> pattern [proxy(self, &Configuration::configure_constructor)])
                |   (str_p("configure")     >> str_p("method")       >> pattern [proxy(self, &Configuration::configure_method)])
                |   (str_p("configure")     >> str_p("property")     >> pattern [proxy(self, &Configuration::configure_property)])
                ) >> +delim
                ;

            block_end = 
                    str_p("end") [proxy(self, &Configuration::end_block)]
                >>  +delim;

            body =
                *(
                    comment
                |   line_statement
                |   block_statement
                |   block_end
                |   delim
                )
                ;
        }

        struct proxy
        {
            typedef void (Configuration::*function_type)(const std::string &);            
            typedef typename S::iterator_t iterator_type;            
            
            proxy(const config_grammar &inst, function_type ptr)
            :    inst_(inst),
                ptr_(ptr)
            {
            }

            void operator()(iterator_type beg, iterator_type end) const
            {
                (inst_.config()->*ptr_)(std::string(beg, end));
            }

            const config_grammar &inst_;
            function_type ptr_;
        };

        const rule<S> &start() const  { return body; }

        rule<S> body, delim, comment, line_statement;
        rule<S> block_statement, pattern, quote;
        rule<S> literal, re_quote, block_end;
        rule<S> escaped_char, regular_expr;
        rule<S> global_level_statement, library_level_statement;
        rule<S> file_level_statement, reflector_level_statement;
        rule<S> property_level_statement;
    };

    config_grammar(Configuration *conf)
    :    conf_(conf)
    {
    }

    inline Configuration *config() const { return conf_; }

    Configuration *conf_;
};

StringMatcher::StringMatcher(const std::string &s)
{
    if ((s.size() < 2) || (*s.begin() != *s.rbegin()))
    {
        literal = s;
    }
    else
    {
        if (*s.begin() == '/')
            re = s.substr(1, s.length()-2);
        else
            literal = s.substr(1, s.length()-2);            
    }
}

bool StringMatcher::matches(const std::string &s) const
{
	//Notify::debug("literal="+literal);
    if (!literal.empty()) return s == literal;
    return regex_match(s.begin(), s.end(), re);
}

Configuration::Configuration()
:    path_delim_("/include/")
{
    cstack_.push(Context());
}

bool Configuration::load(const std::string &filename)
{
    typedef std::ifstream stream_type;    

    // open configuration file
    stream_type ifs(filename.c_str());
    if (!ifs.is_open()) 
        return false;

    ifs.unsetf(std::ios_base::skipws);

    // load file content
    std::vector<stream_type::char_type> buf;
    buf.assign(
        std::istream_iterator<stream_type::char_type>(ifs), 
        std::istream_iterator<stream_type::char_type>());

    // clear context stack
    while (!cstack_.empty()) cstack_.pop();
    cstack_.push(Context());

    // parse text
    if (!parse(buf.begin(), buf.end(), config_grammar(this), space_p - eol_p).full)
        Notify::error("parse error in configuration file `" + filename + "'");

    return true;
}

void Configuration::error(const std::string &msg) const
{
    Notify::error("configuration file: " + msg);
}

bool Configuration::isFileIgnored(const std::string &s) const
{
    return std::find(ignored_files_.begin(), ignored_files_.end(), s) 
        != ignored_files_.end();
}

bool Configuration::isTypeIgnored(const std::string &s) const
{
    return std::find(ignored_types_.begin(), ignored_types_.end(), s) 
        != ignored_types_.end();
}

bool Configuration::isTemplateIgnored(const std::string &s) const
{
    return std::find(ignored_templ_.begin(), ignored_templ_.end(), s) 
        != ignored_templ_.end();
}

bool Configuration::isFileSuppressed(const std::string &s) const
{
    return std::find(suppressed_files_.begin(), suppressed_files_.end(), s)
        != suppressed_files_.end();
}

bool Configuration::isReflectorSuppressed(const std::string &s) const
{
    return std::find(suppressed_refs_.begin(), suppressed_refs_.end(), s)
        != suppressed_refs_.end();
}

void Configuration::ignore_file(const std::string &s)
{
    if (context() != GLOBAL)
        error("`ignore file' only allowed at global level");
    ignored_files_.push_back(s);
}

void Configuration::ignore_type(const std::string &s)
{
    if (context() != GLOBAL)
        error("`ignore type' only allowed at global level");
    ignored_types_.push_back(s);
}

void Configuration::ignore_template(const std::string &s)
{
    if (context() != GLOBAL)
        error("`ignore template' only allowed at global level");
    ignored_templ_.push_back(s);
}

void Configuration::suppress_file(const std::string &s)
{
    if (context() != GLOBAL)
        error("`suppress file' only allowed at global level");
    suppressed_files_.push_back(s);
}

void Configuration::suppress_reflector(const std::string &s)
{
    if (context() != GLOBAL)
        error("`suppress reflector' only allowed at global level");
    suppressed_refs_.push_back(s);
}

void Configuration::end_block(const std::string &)
{
    if (context() == GLOBAL)
        error("unmatched `end'");
    cstack_.pop();
}

void Configuration::configure_library(const std::string &s)
{
    if (context() != GLOBAL)
        error("`configure library' only allowed in global context");
    cstack_.push(LIBRARY);
    lib_opts_.push_back(std::make_pair(s, LibraryOptions()));
}

void Configuration::configure_file(const std::string &s)
{
    if (context() != GLOBAL)
        error("`configure file' only allowed in global context");
    cstack_.push(FILE);
    file_opts_.push_back(std::make_pair(s, FileOptions()));
}

void Configuration::configure_reflector(const std::string &s)
{
    if (context() != GLOBAL)
        error("`configure reflector' only allowed in global context");
    cstack_.push(REFLECTOR);
    ref_opts_.push_back(std::make_pair(s, ReflectorOptions()));
}

void Configuration::configure_constructor(const std::string &s)
{
    if (context() != REFLECTOR)
        error("`configure constructor' only allowed in reflector context");
    cstack_.push(CONSTRUCTOR);
    ref_opts_.back().second.constructor_opts.push_back(std::make_pair(s, ConstructorOptions()));
}

void Configuration::configure_method(const std::string &s)
{
    if (context() != REFLECTOR)
        error("`configure method' only allowed in reflector context");
    cstack_.push(METHOD);
    ref_opts_.back().second.method_opts.push_back(std::make_pair(s, MethodOptions()));
}

void Configuration::configure_property(const std::string &s)
{
    if (context() != REFLECTOR)
        error("`configure property' only allowed at reflector level");
    cstack_.push(PROPERTY);
    ref_opts_.back().second.property_opts.push_back(std::make_pair(s, PropertyOptions()));
}

void Configuration::instantiate_template(const std::string &s)
{
    if (context() != FILE)
        error("`instantiate template' only allowed at file level");
    file_opts_.back().second.templ_instances.insert(s.substr(1, s.length()-2));
}

void Configuration::replace_with(const std::string &s)
{
    std::string t(s.substr(1, s.length()-2));
    switch (context())
    {
    case FILE:
        file_opts_.back().second.replace_text = t;
        file_opts_.back().second.entirely_replaced = true;
        break;
    case REFLECTOR:
        ref_opts_.back().second.replace_text = t;
        ref_opts_.back().second.entirely_replaced = true;
        break;
    case CONSTRUCTOR:
        ref_opts_.back().second.constructor_opts.back().second.replace_text = t;
        ref_opts_.back().second.constructor_opts.back().second.entirely_replaced = true;
        break;
    case METHOD:
        ref_opts_.back().second.method_opts.back().second.replace_text = t;
        ref_opts_.back().second.method_opts.back().second.entirely_replaced = true;
        break;
    case PROPERTY:
        ref_opts_.back().second.property_opts.back().second.replace_text = t;
        ref_opts_.back().second.property_opts.back().second.entirely_replaced = true;
        break;
    default:
        error("`replace with' not allowed at levels higher than `file'");
    }
}

void Configuration::emit_before(const std::string &s)
{
    std::string t(s.substr(1, s.length()-2));
    switch (context())
    {
    case FILE:
        file_opts_.back().second.header_text.append(t);
        break;
    case REFLECTOR:
        ref_opts_.back().second.header_text.append(t);
        break;
    case CONSTRUCTOR:
        ref_opts_.back().second.constructor_opts.back().second.header_text.append(t);
        break;
    case METHOD:
        ref_opts_.back().second.method_opts.back().second.header_text.append(t);
        break;
    case PROPERTY:
        ref_opts_.back().second.property_opts.back().second.header_text.append(t);
        break;
    default:
        error("`emit before' not allowed at levels higher than `file'");
    }
}

void Configuration::emit_after(const std::string &s)
{
    std::string t(s.substr(1, s.length()-2));
    switch (context())
    {
    case FILE:
        file_opts_.back().second.footer_text.append(t);
        break;
    case REFLECTOR:
        ref_opts_.back().second.footer_text.append(t);
        break;
    case CONSTRUCTOR:
        ref_opts_.back().second.constructor_opts.back().second.footer_text.append(t);
        break;
    case METHOD:
        ref_opts_.back().second.method_opts.back().second.footer_text.append(t);
        break;
    case PROPERTY:
        ref_opts_.back().second.property_opts.back().second.footer_text.append(t);
        break;
    default:
        error("`emit after' not allowed at levels higher than `file'");
    }
}

void Configuration::add_base_type(const std::string &s)
{
    if (context() != REFLECTOR)
        error("`add base' only valid at reflector level");
    ref_opts_.back().second.added_base_types.insert(s.substr(1, s.length()-2));
}

void Configuration::set_rw(const std::string &s)
{
    if (context() != REFLECTOR)
        error("`readerwriter' only valid at reflector level");
    ref_opts_.back().second.readerwriter = s.substr(1, s.length()-2);
    ref_opts_.back().second.readerwriter_specified = true;
}

void Configuration::set_cmp(const std::string &s)
{
    if (context() != REFLECTOR)
        error("`comparator' only valid at reflector level");
    ref_opts_.back().second.comparator = s.substr(1, s.length()-2);
    ref_opts_.back().second.comparator_specified = true;
}

void Configuration::add_attribute(const std::string &s)
{
    std::string t(s.substr(1, s.length()-2));
    switch (context())
    {
    case FILE:
        file_opts_.back().second.attributes.push_back(t);
        break;
    case REFLECTOR:
        ref_opts_.back().second.attributes.push_back(t);
        break;
    case CONSTRUCTOR:
        ref_opts_.back().second.constructor_opts.back().second.attributes.push_back(t);
        break;
    case METHOD:
        ref_opts_.back().second.method_opts.back().second.attributes.push_back(t);
        break;
    case PROPERTY:
        ref_opts_.back().second.property_opts.back().second.attributes.push_back(t);
        break;
    default:
        error("`attribute' not allowed at levels higher than `file'");
    }
}

void Configuration::path_delimiter(const std::string &s)
{
    setRelativePathDelimiter(s.substr(1, s.length()-2));
}

void Configuration::value_type(const std::string &)
{
    if (context() != REFLECTOR)
        error("`value-type' only valid at reflector level");
    ref_opts_.back().second.type_kind = ReflectorOptions::VALUE_TYPE;
}

void Configuration::object_type(const std::string &)
{
    if (context() != REFLECTOR)
        error("`object-type' only valid at reflector level");
    ref_opts_.back().second.type_kind = ReflectorOptions::OBJECT_TYPE;
}

void Configuration::abstract_object_type(const std::string &)
{
    if (context() != REFLECTOR)
        error("`abstract-object-type' only valid at reflector level");
    ref_opts_.back().second.type_kind = ReflectorOptions::ABSTRACT_OBJECT_TYPE;
}

void Configuration::dependency_unix(const std::string &s)
{
    if (context() != LIBRARY)
        error("`dependency' only valid at library level");
    lib_opts_.back().second.unix_dependencies.insert(s.substr(1, s.length()-2));
}

void Configuration::dependency_win32(const std::string &s)
{
    if (context() != LIBRARY)
        error("`dependency' only valid at library level");
    lib_opts_.back().second.win32_release_dependencies.insert(s.substr(1, s.length()-2));
}

void Configuration::dependency_win32d(const std::string &s)
{
    if (context() != LIBRARY)
        error("`dependency' only valid at library level");
    lib_opts_.back().second.win32_debug_dependencies.insert(s.substr(1, s.length()-2));
}

void Configuration::get_method(const std::string &s)
{
    if (context() != PROPERTY)
        error("`get_method' only valid at property level");
    ref_opts_.back().second.property_opts.back().second.get_method.assign(s.substr(1, s.length()-2));
}
void Configuration::set_method(const std::string &s)
{
    if (context() != PROPERTY)
        error("set_method' only valid at property level");
    ref_opts_.back().second.property_opts.back().second.set_method.assign(s.substr(1, s.length()-2));
}
void Configuration::count_method(const std::string &s)
{
    if (context() != PROPERTY)
        error("count_method' only valid at property level");
    ref_opts_.back().second.property_opts.back().second.count_method.assign(s.substr(1, s.length()-2));
}
void Configuration::add_method(const std::string &s)
{
    if (context() != PROPERTY)
        error("add_method' only valid at property level");
    ref_opts_.back().second.property_opts.back().second.add_method.assign(s.substr(1, s.length()-2));
}
void Configuration::insert_method(const std::string &s)
{
    if (context() != PROPERTY)
        error("insert_method' only valid at property level");
    ref_opts_.back().second.property_opts.back().second.insert_method.assign(s.substr(1, s.length()-2));
}
void Configuration::remove_method(const std::string &s)
{
    if (context() != PROPERTY)
        error("`remove_method' only valid at property level");
    ref_opts_.back().second.property_opts.back().second.remove_method.assign(s.substr(1, s.length()-2));
}
