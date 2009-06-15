#ifndef XML_PARSER_
#define XML_PARSER_

#include "XML_node.h"

#include <libxml/parser.h>

#include <string>

namespace XML
{

    class Parser
    {
    public:
        static Node *parse(const std::string &filename)
        {
            xmlDoc *doc = xmlParseFile(filename.c_str());
            if (!doc)
                return false;

            xmlNode *root = xmlDocGetRootElement(doc);
            if (!root)
            {
                xmlFreeDoc(doc);
                return false;
            }

            Node *node = new Node(doc, root);
            xmlFreeDoc(doc);

            return node;
        }
    };

}

#endif
