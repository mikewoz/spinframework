#include "NodeMap.h"
#include "Notify.h"
#include "PathNameUtils.h"
#include "XML_parser.h"

#include <fstream>

NodeMap::NodeMap(const std::string &src_dir)
:    src_dir_(PathNameUtils::consolidateDelimiters(src_dir, true))
{
}

NodeMap::~NodeMap()
{
    clear();
}

void NodeMap::clear()
{
    for (NodeList::iterator i=roots_.begin(); i!=roots_.end(); ++i)
        delete *i;

    roots_.clear();
    nodes_.clear();
}

bool NodeMap::load_file(const std::string &id)
{
    std::string filename = src_dir_ + id + ".xml";
    const XML::Node *root = XML::Parser::parse(filename);
    if (!root)
    {
        Notify::warn("could not load file `" + filename + "'");
        return false;
    }
    roots_.push_back(root);
    register_nodes(root);
    return true;
}

bool NodeMap::loadIndex()
{
    if (!roots_.empty()) 
        return false;

    const XML::Node *root = XML::Parser::parse(src_dir_ + "index.xml");
    if (root)
    {
        register_nodes(root);
        roots_.push_back(root);
        return true;
    }

    return false;
}

const XML::Node *NodeMap::findNode(const std::string &id)
{
    StringNodeMap::const_iterator i = nodes_.find(id);
    if (i != nodes_.end())
        return i->second;

    if (load_file(id))
    {
        i = nodes_.find(id);
        if (i != nodes_.end())
            return i->second;
    }

    return 0;
}

void NodeMap::register_nodes(const XML::Node *root)
{
    for (XML::Node::Element_map::const_iterator i=root->elements().begin(); i!=root->elements().end(); ++i)
    {
        std::string id = i->second->get_attribute("id");
        if (!id.empty())
            nodes_[id] = i->second;
        register_nodes(i->second);
    }
}
