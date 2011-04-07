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

#include "spinUtil.h"
#include "libloUtil.h"
#include "MediaManager.h"

#include <osg/Referenced>
#include <osg/Group>
#include <osg/Node>
#include <osg/observer_ptr>
#include <boost/python.hpp>
#include <cppintrospection/Value>

namespace spin
{

class SceneManager;

/**
 * Python script that is ran periodically.
 */
typedef struct {
    boost::python::object run;
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
    boost::python::object run;
    std::string eventName;
    bool enabled;
    bool serverSide;
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
    ReferencedNode(SceneManager *sceneManager, char *initID);
    ~ReferencedNode();

    void registerNode(SceneManager *s);
    //void registerNode(std::string sceneID);

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
    virtual void callbackUpdate();

    /**
     * The attach() method is used to properly add a node to the scene graph.
     * Each node derived from ReferencedNode has an attachmentNode parameter that
     * can be overridden in derived classes, but to keep code simple, this is
     * the only this method that actually performs the attachment.
     *
     * In the default case, nodes should are attached directly to the parent
     * ReferencedNode instance.
     */
    void attach();
    /**
     * The detach() method is removes a node from the scene graph, depending on
     * it's attachment position (specified by attachmentNode).
     */
    void detach();

    /**
     * IMPORTANT:
     * subclasses of ReferencedNode are allowed to contain complicated subgraphs,
     * and can also change their attachmentNode so that children are attached
     * anywhere in that subgraph. If that is the case, the updateNodePath()
     * function MUST be overridden, and extra nodes must be manually pushed onto
     * currentNodePath.
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
     * Returns the current parent name (string)
     */
    char *getParent() const { return parent->s_name; }

    /**
     * Returns the current parent as an osg::Group
     */
    osg::Group *getParent(int i) { return osg::Group::getParent(i); }

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

    /**
     * Returns the current host
     */
    const char *getContext() const { return contextString.c_str(); }
    std::string getContextString() const { return contextString; }

    void setParam (const char *paramName, const char *paramValue);
    void setParam (const char *paramName, float paramValue);

    /**
     * subclasses of ReferencedNode may contain complicated subgraphs, and any
     * children get attached not to the node pointer itself, but to an
     * attachmentNode. This attachmentNode essentially defines the origin of the
     * local coordinate system of this node (according to the subgraph). This
     * function returns a pointer to this node.
     */
    osg::Group *getAttachmentNode() const { return attachmentNode; }

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

    // ***********************************************************
    // data:
    // FIXME: Please make data members private.

    t_symbol *id;
    std::string nodeType;

    std::string contextString;

    int pd_mail_id;
    lo_method oscHandler;

    t_symbol *parent, *newParent;

    bool scheduleForDeletion;

    // debug

    bool textFlag;

    stringParamType stringParams;
    floatParamType floatParams;

    float subgraphAlpha_;

    osg::NodePath currentNodePath;

    std::vector<ReferencedNode*> children;

    //osg::ref_ptr<osg::Geode> textGeode;

    SceneManager *sceneManager;
    MediaManager *mediaManager;

    //bool setScript( const std::string& s, const std::string& params );
    //bool setScript( const char *scriptPath, double freq );  // freq is nb of calls per second
    std::string getID() const;

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

 protected:

    std::string _scriptFile;
    boost::python::object _scriptRun;
    EventScriptList _eventScriptList;
    CronScriptList _cronScriptList;

 private:

    /**
     * TO BE DEPRECATED?
     * It used to be that we wanted to keep a list of all child ReferencedNode
     * nodes that are attached. Was this so that we could differentiate SPIN
     * nodes from other scene graph node? That seems silly, since one can always
     * try a dynamic_cast or use cppintrospection
     */
    ReferencedNode *as_getChild(ReferencedNode *child);
    /**
     * TO BE DEPRECATED?
     */
    void as_addChild(ReferencedNode *child);
    /**
     * TO BE DEPRECATED?
     */
    void as_removeChild(ReferencedNode *child);

    /**
     * TO BE DEPRECATED?
     * The idea is that one type of node can only be attached to certain types
     * of other nodes, but that has not been implemented. Currently, the only
     * illegal parents include the node itself, or any children.
     */
    bool legalParent (t_symbol *newParent);

    /**
     * The node that children get attached to:
     * We keep it private to force the use of setAttachmentNode(), which results
     * in an update of the currentNodePath.
     */
    osg::Group *attachmentNode;

};

/**
 * FIXME: Please document this.
 */
class ReferencedNode_data : public osg::Referenced
{
    public:
        ReferencedNode_data(ReferencedNode *n) { node_ = n; }
        //~ReferencedNode_data();
        void update()
        {
            if (node_.valid())
                node_->callbackUpdate();
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
                data->update();
            traverse(node, nv);
        }
};

} // end of namespace spin

#endif

