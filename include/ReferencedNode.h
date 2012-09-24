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


#ifndef __ReferencedNode_H
#define __ReferencedNode_H

#include <string>

#include "config.h"
#include "spinUtil.h"
#include "libloUtil.h"

#include <osg/Referenced>
#ifndef GL_GLEXT_LEGACY
#define GL_GLEXT_LEGACY // To avoid glext error
#endif
#include <osg/Group>
#include <osg/Node>
#include <osg/observer_ptr>
#ifndef DISABLE_PYTHON
#include <boost/python.hpp>
#endif
#include <cppintrospection/Value>

#ifdef DISABLE_PYTHON
namespace boost { namespace python { class object {void run(){}}; }}
#endif

namespace spin
{

class SceneManager;
class ReferencedNode;
class ReferencedStateSet;

typedef std::vector< osg::ref_ptr<ReferencedNode> > nodeListType;
typedef std::map< std::string, nodeListType > nodeMapType;
typedef std::pair< std::string, nodeListType > nodeMapPair;

typedef std::vector< osg::ref_ptr<ReferencedStateSet> > ReferencedStateSetList;
typedef std::map< std::string, ReferencedStateSetList > ReferencedStateSetMap;
typedef std::pair< std::string, ReferencedStateSetList > ReferencedStateSetPair;





/**
 * Python script that is ran periodically.
 */
typedef struct {
    #ifndef DISABLE_PYTHON_
    boost::python::object run;
    #endif
    double freq;
    double lastRun;
    bool enabled;
    bool serverSide;
    std::string path;
    std::string params;
    std::string pyScript;
    std::string pyModule;
} CronScript;

/**
 * Python script that is ran when some event happens.
 */
typedef struct {
    #ifndef DISABLE_PYTHON_
    boost::python::object run;
    #endif

    /**
     * A string containing the name of the Event.
     */

    std::string eventName;

    /**
     * Whether the event is enabled or disabled.
     */

    bool enabled;

    /**
     * Whether the event should be calculated serverside or only clientside.
     */

    bool serverSide;

    /**
     * The path of the python script to be attached.
     */

    std::string path;
    std::string params;
    std::string pyScript;
    std::string pyModule;
} EventScript;

typedef std::map< const std::string, CronScript* > CronScriptList;
typedef std::map< const std::string, EventScript* > EventScriptList;

typedef std::map< std::string, std::string > stringParamType;
typedef std::map< std::string, float > floatParamType;

/**
 * \brief The base class for all SPIN scene graph nodes.
 *
 * Any node that is to be attached to the scene graph needs to be extended from
 * the ReferencedNode class. By doing so, the OSC networking interface is created
 * automatically, and the wx gui is automatically populated with the nodes
 * editable properties.
 */
class ReferencedNode : virtual public osg::Group
{
public:
    ReferencedNode(SceneManager *sceneManager, const char *initID);
    ~ReferencedNode();

    virtual void registerNode(SceneManager *s);

    /**
     * For nodes that require regular programmatic control, there is a callback
     * that is evaluated with every refresh. This function can thus be used for
     * animations, or any other periodic updates.
     *
     * Note that changes to the scene graph structure (eg, moving/deleting nodes
     * should NOT be done within this callback because traversals stacks will
     * become corrupted. The technique is rather to enable a flag and then do
     * the actual change in the SceneManager::updateGraph() method.
     */
    virtual void callbackUpdate(osg::NodeVisitor* nv);

    /**
     * IMPORTANT:
     * subclasses of ReferencedNode are allowed to contain complicated subgraphs,
     * and can also change their attachmentNode so that children are attached
     * anywhere in that subgraph. If that is the case, the updateNodePath()
     * function MUST be overridden, and extra nodes must be manually pushed onto
     * currentNodePath_.
     */
    virtual void updateNodePath(bool updateChildren = true);

    int setAttachmentNode(osg::Group *n);

    /**
     * An internal method that keeps track of the nodepath (for efficient
     * computation of global position, etc.
     */
    void updateChildNodePaths();

    /**
     * This method schedules a change in parent for this node. The setParent()
     * does not immediately change the scenegraph, since it can be called at any
     * time, even while in a traversal. The graph is updated later using the
     * attach() method, which is called by SceneManager->updateGraph() when
     * there is a legal time to re-order the scenegraph.
     *
     * Internally, this method just sets the newParent property.
     */
    void setParent (const char *newvalue);

    /**
     * Attach this node to the node with parentID (if found).
     *
     */
    void attachTo (const char* parentID);

    /**
     * Detaches the node from the parentID (if found). Note: you can pass "*"
     * for the parentID and the node will be detached from ALL parents.
     */
    void detachFrom(const char* parentID);

    /**
     * The inGraph method checks if the node is actually attached to the scene
     * graph. There are cases (eg, SwitchNode or using detachFrom) that may
     * cause a node to be orphaned (not attached anywhere). In these cases,
     * reporters and pointers and anything that maintains a list of targets must
     * check if the node is inGraph().
     */
    bool inGraph();


    unsigned int getNumParents() const { return parentNodes_.size(); }

    /**
     * Returns the parent id (string)
     */
    std::string getParentID(int i) const;

    /**
     * Returns the current parent as an osg::Group
     */
    ReferencedNode* getParentNode(int i);


    std::vector<ReferencedNode*> getChildren();

    /**
     * A node can 'belong' to a certain host machine, allowing it to be rendered
     * or behave differently than on other machines.
     *
     * NOTE: the "NULL" string means that it belongs to no specific context.
     *
     * NOTE: a scene operating in SERVER_MODE will always create the node, so
     * this feature is only really relevant for clients applications.
     */
    virtual void setContext (const char *newvalue);

    void setAlpha (float alpha);
    float getAlpha() const { return subgraphAlpha_; }
    
    void setReceiveShadows(int b);
    int getReceiveShadows() const { return (int)receiveShadows_; }
    
    void setCastShadows(int b);
    int getCastShadows() const { return (int)castShadows_; }

    /**
     * Returns the current host
     */
    const char *getContext() const { return contextString_.c_str(); }
    std::string getContextString() const { return contextString_; }

    virtual void setParam (const char *paramName, const char *paramValue);
    virtual void setParam (const char *paramName, float paramValue);

    void setStateSetFromFile (const char* filename);
    void setStateSet         (const char* s);
    const char *getStateSet  () const { return stateset_->s_name; }

    /**
     * In derived classes, you can handle how a stateset gets applied to a node
     * (eg, which part of the subgraph it is attached by overriding the
     * updateStateSet method.
     */
    virtual void updateStateSet();

    /**
     * subclasses of ReferencedNode may contain complicated subgraphs, and any
     * children get attached not to the node pointer itself, but to an
     * attachmentNode. This attachmentNode essentially defines the origin of the
     * local coordinate system of this node (according to the subgraph). This
     * function returns a pointer to this node.
     */
    osg::Group *getAttachmentNode() const { return attachmentNode_; }

    /**
     * Debug print (to log/console)
     */
    virtual void debug();

    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node.
     */
    virtual std::vector<lo_message> getState() const;

    /**
     * Request to broadcast the node state via SceneManager.
     */
    virtual void stateDump();

    /**
     * Request to send the node state to one address
     */
    virtual void stateDump(lo_address addr);


    /**
     * Return the string id for this node
     */
    std::string getID() const { return std::string(id_->s_name); }
    std::string getNodeType() const { return nodeType_; }
    t_symbol* getNodeSymbol() { return id_; }

    std::string getOSCPath() const;

    bool addCronScript( bool serverSide, const std::string& label, const std::string& scriptPath,
                        double freq, const std::string& params );
    bool callCronScripts();
    bool enableCronScript( const char* label, int enable );
    bool removeCronScript( const char* label );

    bool addEventScript( bool serverSide, const std::string& label, const std::string& eventName,
                         const std::string& scriptPath,  const std::string& params );
    bool callEventScript( const std::string& eventName,
                          cppintrospection::ValueList& args );
    bool enableEventScript( const char* label, int enable );
    bool removeEventScript( const char* label );

    // a NodeMask is an unsigned int.
    void setNodeMask( osg::Node::NodeMask nm );

    // FIXME
    osg::NodePath currentNodePath_;
    bool scheduleForDeletion_;

    // ***********************************************************

 protected:

    /**
     * TO BE DEPRECATED?
     * The idea is that one type of node can only be attached to certain types
     * of other nodes, but that has not been implemented. Currently, the only
     * illegal parents include the node itself, or any children.
     */
    bool legalParent (t_symbol *newParent);

    void setNodeType(std::string t) { nodeType_ = t; }

    SceneManager *sceneManager_;

 private:
    t_symbol *id_;
    std::string nodeType_;
    std::string contextString_;
    nodeListType parentNodes_;
    float subgraphAlpha_;
    
    bool receiveShadows_;
    bool castShadows_;

    /**
     * The node that children get attached to:
     * We keep it private to force the use of setAttachmentNode(), which results
     * in an update of the currentNodePath_.
     */
    osg::Group *attachmentNode_;

    stringParamType stringParams_;
    floatParamType floatParams_;

    t_symbol* stateset_;

    std::string _scriptFile;
#ifndef DISABLE_PYTHON
    boost::python::object _scriptRun;
#endif
    EventScriptList _eventScriptList;
    CronScriptList _cronScriptList;
};

/**
 * FIXME: Please document this.
 */
class ReferencedNode_data : public osg::Referenced
{
    public:
        ReferencedNode_data(ReferencedNode *n) { node_ = n; }
        //~ReferencedNode_data();
        void update(osg::NodeVisitor* nv)
        {
            if (node_.valid())
                node_->callbackUpdate(nv);
        }
    private:
        osg::observer_ptr<ReferencedNode> node_;
};

/**
 * FIXME: Please document this.
 */
class ReferencedNode_callback : public osg::NodeCallback
{
    public:
        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            osg::ref_ptr<ReferencedNode_data> data = dynamic_cast<ReferencedNode_data*> (node->getUserData());
            if (data != NULL)
                data->update(nv);
            traverse(node, nv);
        }
};

} // end of namespace spin

#endif

