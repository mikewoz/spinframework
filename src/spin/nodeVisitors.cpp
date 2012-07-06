#include "nodeVisitors.h"

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/Switch>
#include <osg/Sequence>
#include "ReferencedNode.h"

#include <iostream>

namespace spin
{

SearchVisitor::SearchVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
{
    // Flag all osg object as NULL
    GroupNode = NULL;
    GeodeNode = NULL;
    MTNode = NULL;
    PATNode = NULL;
    SwitchNode = NULL;
    SequenceNode = NULL;
}

void SearchVisitor::apply(osg::Node &searchNode)
{
    traverse(searchNode);
}


void SearchVisitor::apply(osg::Group &searchNode)
{
    osg::ref_ptr<osg::Group> n = dynamic_cast<osg::Group*> (&searchNode);
    if (n.valid())
    {
        if (searchForName == searchNode.getName()) GroupNode = n;
    }
    traverse(searchNode);
}

void SearchVisitor::apply(osg::Geode &searchNode)
{
    osg::ref_ptr<osg::Geode> n = dynamic_cast<osg::Geode*> (&searchNode);
    if (n.valid())
    {
        if (searchForName == searchNode.getName()) GeodeNode = n;
    }
    traverse(searchNode);
}


void SearchVisitor::apply(osg::MatrixTransform &searchNode)
{
    osg::ref_ptr<osg::MatrixTransform> n = dynamic_cast<osg::MatrixTransform*> (&searchNode);
    if (n.valid())
    {
        if (searchForName == searchNode.getName()) MTNode = n;
    }
    traverse(searchNode);
}


void SearchVisitor::apply(osg::PositionAttitudeTransform &searchNode)
{
    osg::ref_ptr<osg::PositionAttitudeTransform> n = dynamic_cast<osg::PositionAttitudeTransform*> (&searchNode);
    if (n.valid())
    {
        if (searchForName == searchNode.getName()) PATNode = n;
    }
    traverse(searchNode);
}


void SearchVisitor::apply(osg::Switch &searchNode)
{
    osg::ref_ptr<osg::Switch> n = dynamic_cast<osg::Switch*> (&searchNode);
    if (n.valid())
    {
        if (searchForName == searchNode.getName()) SwitchNode = n;
    }
    traverse(searchNode);
}


void SearchVisitor::apply(osg::Sequence &searchNode)
{
    osg::ref_ptr<osg::Sequence> n = dynamic_cast<osg::Sequence*> (&searchNode);
    if (n.valid())
    {
        if (searchForName == searchNode.getName()) SequenceNode = n;
    }
    traverse(searchNode);
}

// search for node with given name starting at the search node
void SearchVisitor::searchNode(osg::Node* searchFromMe, std::string searchName)
{
    GroupNode = NULL;
    GeodeNode = NULL;
    MTNode = NULL;
    PATNode = NULL;
    SwitchNode = NULL;
    SequenceNode = NULL;

    searchForName = searchName;
    searchFromMe->accept(*this);
}


osg::ref_ptr<osg::Group> SearchVisitor::getGroup() { return GroupNode; }
osg::ref_ptr<osg::MatrixTransform> SearchVisitor::getMT() { return MTNode; }
osg::ref_ptr<osg::Geode> SearchVisitor::getGeode() { return GeodeNode; }
osg::ref_ptr<osg::PositionAttitudeTransform> SearchVisitor::getPAT() { return PATNode; }
osg::ref_ptr<osg::Switch> SearchVisitor::getSwitchNode() { return SwitchNode; }
osg::ref_ptr<osg::Sequence> SearchVisitor::getSequenceNode() { return SequenceNode; }
        
NodeSearcher::NodeSearcher(NodeList& list) : _nodeList(list)
{
    setTraversalMode( NodeVisitor::TRAVERSE_ALL_CHILDREN );
}

void NodeSearcher::apply(osg::Node& node)
{
    //std::cout << "checking " << node.getName() << std::endl;
    //if (_searchName == node.getName())
    std::string n = node.getName();
    if (n.find(_searchName) != std::string::npos )
    {
        //std::cout << "found " << node.className() << std::endl;
        _nodeList.push_back(&node);
    }
    traverse(node);
}

void NodeSearcher::search(osg::Node* subgraph, std::string s)
{
    _nodeList.clear();
    _searchName = s;
    subgraph->accept(*this);
}

DebugVisitor::DebugVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
{
    setTraversalMode( NodeVisitor::TRAVERSE_ALL_CHILDREN );
}

void DebugVisitor::print(osg::Node* subgraph)
{
    subgraph->accept(*this);
}

void DebugVisitor::apply(osg::Node &node)
{
    std::cout << leadingSpaces(getNodePath().size()) << "NODE:  " << node.getName() << std::endl;
    traverse(node);
}

void DebugVisitor::apply(osg::PositionAttitudeTransform &node)
{
    std::cout << leadingSpaces(getNodePath().size()) << "PAT:   " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
    traverse(node);
}


void DebugVisitor::apply(osg::Geode &node)
{
    std::cout << leadingSpaces(getNodePath().size()) << "GEODE: " << node.getName() << std::endl;
    traverse(node);
}


void DebugVisitor::apply(osg::Group &node)
{
    ReferencedNode *n;

    if ((n=dynamic_cast<ReferencedNode*>(&node))) {
        std::cout << leadingSpaces(getNodePath().size()) << "SPIN NODE: type=" << n->getNodeType() << ", id=" << n->getID() << "  (" << node.getNumChildren () << " children)" << std::endl;
    } else {
        std::cout << leadingSpaces(getNodePath().size()) << "GROUP: " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
    }
    traverse(node);
}


UpdateSceneVisitor::UpdateSceneVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}

void UpdateSceneVisitor::apply(osg::Node &node) {
    traverse(node);
}


void UpdateSceneVisitor::apply(osg::Group &node)
{
    ReferencedNode *n;
    if ((n = dynamic_cast<ReferencedNode*>(&node)))
    {
        n->callbackUpdate(this);
    }
    traverse(node);
}


TextureStateSetFinder::TextureStateSetFinder(StateSetList& list) : _statesetList(list)
{
    setTraversalMode( NodeVisitor::TRAVERSE_ALL_CHILDREN );
}

void TextureStateSetFinder::apply(osg::Node& node)
{
    apply(node.getStateSet());
    traverse(node);
}

void TextureStateSetFinder::apply(osg::Geode& geode)
{
    apply(geode.getStateSet());
    for(unsigned int i=0;i<geode.getNumDrawables();++i)
    {
        apply(geode.getDrawable(i)->getStateSet());
    }
    traverse(geode);
}

void TextureStateSetFinder::apply(osg::StateSet* stateset)
{
    if (!stateset) return;

    osg::StateAttribute* attr = stateset->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
    if (attr)
    {

        // from doxygen:
        // attr->asTexture() is fast alternative to dynamic_cast<>
        // for determining if state attribute is a Texture.

        osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(attr);
        if (texture2D)
        {
            _statesetList.push_back(stateset);
            //apply(dynamic_cast<osg::Image*>(texture2D->getImage()));
        }

        osg::TextureRectangle* textureRec = dynamic_cast<osg::TextureRectangle*>(attr);
        if (textureRec)
        {
            _statesetList.push_back(stateset);
            //apply(dynamic_cast<osg::Image*>(textureRec->getImage()));
        }
    }
}
        
TextureStateSetFinder& TextureStateSetFinder::operator= (const TextureStateSetFinder&) { return *this; }

} // end of namespace spin

