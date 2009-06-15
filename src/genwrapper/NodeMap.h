#ifndef NODEMAP_H_
#define NODEMAP_H_

#include "XML_node.h"

#include <vector>
#include <map>
#include <string>

class NodeMap
{
public:
    NodeMap(const std::string &src_dir);
    ~NodeMap();

    bool loadIndex();
    inline const XML::Node *getIndexNode() const;
    const XML::Node *findNode(const std::string &id);
    void clear();

protected:
    bool load_file(const std::string &id);
    void register_nodes(const XML::Node *root);

private:
    typedef std::vector<const XML::Node *> NodeList;
    NodeList roots_;

    typedef std::map<std::string, const XML::Node *> StringNodeMap;
    StringNodeMap nodes_;

    std::string src_dir_;
};

// INLINE METHODS

inline const XML::Node *NodeMap::getIndexNode() const
{
    if (roots_.empty()) 
        return 0;
    return roots_.front();
}

#endif
