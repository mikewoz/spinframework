#ifndef XML_NODE_
#define XML_NODE_

#include <libxml/parser.h>

#include <map>
#include <string>

namespace XML
{

    class Node
    {
    public:
        typedef std::map<std::string, std::string> Attribute_map;
        typedef std::multimap<std::string, Node *> Element_map;

        Node(xmlDoc *doc, xmlNode *root)
        {
            // retrieve name
            if (root->name)
                name_ = std::string(reinterpret_cast<const char *>(root->name));

            // retrieve attributes        
            xmlAttr *curattr = root->properties;
            while (curattr)
            {
                xmlChar *str = xmlNodeListGetString(doc, curattr->children, 1);
                std::string text = std::string(reinterpret_cast<char *>(str));
                xmlFree(str);
                attributes_.insert(std::make_pair(std::string(reinterpret_cast<const char *>(curattr->name)), text));
                curattr = curattr->next;
            }

            // retrieve content
            xmlChar *str = xmlNodeGetContent(root);
            content_ = std::string(reinterpret_cast<char *>(str));
            xmlFree(str);

            // retrieve elements
            xmlNode *curnode = root->xmlChildrenNode;
            while (curnode)
            {
                if (!xmlNodeIsText(curnode))
                {
                    elements_.insert(std::make_pair(std::string(reinterpret_cast<const char *>(curnode->name)), new Node(doc, curnode)));
                }
                curnode = curnode->next;
            }
        }

        ~Node()
        {
            for (Element_map::const_iterator i=elements_.begin(); i!=elements_.end(); ++i)
            {
                delete i->second;
            }
        }

        inline std::string get_content() const
        {
            return content_;
        }

        inline const Attribute_map &attributes() const { return attributes_; }
        inline const Element_map &elements() const { return elements_; }

        inline const std::string &get_name() const
        {
            return name_;
        }

        inline std::string get_attribute(const std::string &name) const
        {
            Attribute_map::const_iterator i = attributes_.find(name);
            if (i == attributes_.end())
            {
                return std::string();
            }
            return i->second;
        }

        inline bool has_attribute(const std::string &name) const
        {
            return attributes_.find(name) != attributes_.end();
        }

        inline const Node *get_first_element(const std::string &name) const
        {
            Element_map::const_iterator i = elements_.find(name);
            if (i == elements_.end())
            {
                return 0;
            }
            return i->second;
        }

        inline int get_num_elements(const std::string &name) const
        {
            return static_cast<int>(elements_.count(name));
        }

        inline bool has_elements(const std::string &name) const
        {
            return elements_.find(name) != elements_.end();
        }

        inline std::string get_first_element_content(const std::string &name) const
        {
            Element_map::const_iterator i = elements_.find(name);
            if (i == elements_.end())
            {
                return std::string();
            }
            return i->second->get_content();
        }

        inline const Node *extract_element(const std::string &name)
        {
            Element_map::iterator i = elements_.find(name);
            if (i == elements_.end())
            {
                return 0;
            }
            Node *node = i->second;
            elements_.erase(i);
            return node;
        }

    private:
        Attribute_map attributes_;
        Element_map elements_;
        std::string content_;
        std::string name_;
    };

}

#endif
