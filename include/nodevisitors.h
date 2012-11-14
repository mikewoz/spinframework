// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
//
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#ifndef __NODE_VISITORS_H__
#define __NODE_VISITORS_H__

#include <osg/NodeVisitor>

#include <string>
#include <iostream>

namespace osg {
    class Node;
    class Texture2D;
    class TextureRectangle;
    class Sequence;
    class Switch;
    class PositionAttitudeTransform;
    class Geode;
    class MatrixTransform;
}
namespace osgAnimation
{
    class AnimationManagerBase;
    class BasicAnimationManager;
}

namespace spin
{

/**
 * \brief An OSG NodeVisitor class that allows us to search for a specific node
 *        name/type in the scene graph.
 *
 * For example, this is used to find the switch & sequence nodes when loading
 * an OSG model in the ModelNode class, so that we may discover animation
 * features.
 */

class SearchVisitor : public osg::NodeVisitor
{
public:
    SearchVisitor();

    virtual void apply(osg::Node &searchNode);

    virtual void apply(osg::Group &searchNode);
    virtual void apply(osg::Geode &searchNode);
    virtual void apply(osg::MatrixTransform &searchNode);
    virtual void apply(osg::PositionAttitudeTransform &searchNode);
    virtual void apply(osg::Switch &searchNode);
    virtual void apply(osg::Sequence &searchNode);

    // search for node with given name starting at the search node
    void searchNode(osg::Node* searchFromMe, std::string searchName);

    osg::ref_ptr<osg::Group> getGroup();
    osg::ref_ptr<osg::MatrixTransform> getMT();
    osg::ref_ptr<osg::Geode> getGeode();
    osg::ref_ptr<osg::PositionAttitudeTransform> getPAT();
    osg::ref_ptr<osg::Switch> getSwitchNode();
    osg::ref_ptr<osg::Sequence> getSequenceNode();

private:
    std::string searchForName;

    osg::ref_ptr<osg::Group> GroupNode;
    osg::ref_ptr<osg::Geode> GeodeNode;
    osg::ref_ptr<osg::MatrixTransform> MTNode;
    osg::ref_ptr<osg::PositionAttitudeTransform> PATNode;
    osg::ref_ptr<osg::Switch> SwitchNode;
    osg::ref_ptr<osg::Sequence> SequenceNode;
};

typedef std::vector< osg::ref_ptr<osg::Node> > NodeList;

// FIXME: is the definition below OK?
/**
 * \brief A NodeVisitor that stores a list of all nodes whose name matches
 * the one we are looking for.
 */

class NodeSearcher : public osg::NodeVisitor
{
    public:
        NodeSearcher(NodeList& list);

        virtual void apply(osg::Node& node);

        /**
         * Search for nodes with a given name in the provided subgraph.
         */

        void search(osg::Node* subgraph, std::string s);

        std::string _searchName;
        NodeList& _nodeList;

    protected:
        NodeSearcher& operator = (const NodeSearcher&) { return *this; }
};

/**
 * \brief An OSG NodeVisitor class that pretty prints the scene graph
 */
class DebugVisitor : public osg::NodeVisitor
{
    public:
        DebugVisitor();
        
        /**
         * prints the subgraph to std::cout
         */
        void print(osg::Node* subgraph);

        virtual void apply(osg::Node &node);
        virtual void apply(osg::PositionAttitudeTransform &node);
        virtual void apply(osg::Geode &node);
        virtual void apply(osg::Group &node);
};

/**
 * \brief A NodeVisitor class that invokes the update callback in every node
 */
class UpdateSceneVisitor : public osg::NodeVisitor
{
    public:
        UpdateSceneVisitor();
        virtual void apply(osg::Node &node);
        virtual void apply(osg::Group &node);
};

typedef std::vector< osg::ref_ptr<osg::StateSet> > StateSetList;
typedef std::vector< osg::ref_ptr<osg::Switch> > SwitchNodeList;
typedef std::vector< osg::ref_ptr<osg::Sequence> > SequenceNodeList;




/**
 * \brief A NodeVisitor class that finds all osg::Switch nodes
 */
class SwitchNodeFinder : public osg::NodeVisitor
{
    public:
        SwitchNodeFinder(SwitchNodeList& list) : _switchList(list)
        { setTraversalMode( NodeVisitor::TRAVERSE_ALL_CHILDREN ); }    
        virtual void apply(osg::Node& node);

        SwitchNodeList& _switchList;

    protected:
        SwitchNodeFinder& operator = (const SwitchNodeFinder&);
};

/**
 * \brief A NodeVisitor class that finds all osg::Sequence nodes
 */
class SequenceNodeFinder : public osg::NodeVisitor
{
    public:
        SequenceNodeFinder(SequenceNodeList& list) : _sequenceList(list)
        { setTraversalMode( NodeVisitor::TRAVERSE_ALL_CHILDREN ); }    
        virtual void apply(osg::Node& node);

        SequenceNodeList& _sequenceList;

    protected:
        SequenceNodeFinder& operator = (const SequenceNodeFinder&);
};

/**
 * \brief A NodeVisitor class that finds all textured statesets in a node:
 */
class TextureStateSetFinder : public osg::NodeVisitor
{
    public:
        TextureStateSetFinder(StateSetList& list);
        virtual void apply(osg::Node& node);
        virtual void apply(osg::Geode& geode);
        virtual void apply(osg::StateSet* stateset);

        /*
           inline void apply(osg::Image* img)
           {
           if (img)
           {
           std::cout << "      adding image: " << img->getFileName() << std::endl;
           _imageList.push_back(img);
           }
           }
         */

        StateSetList& _statesetList;

    protected:
        TextureStateSetFinder& operator = (const TextureStateSetFinder&);
};


class AnimationManagerFinder : public osg::NodeVisitor
{
public:
    osgAnimation::BasicAnimationManager* _am;
    AnimationManagerFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) { _am = 0; }
    virtual void apply(osg::Node& node);
};

} // end of namespace spin

#endif // __NODE_VISITORS_H__
