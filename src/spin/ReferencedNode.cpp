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

#include <string>
#include <vector>
#include <iostream>
#include <exception>
#include <osgDB/FileUtils>
#include <osg/BlendFunc>
#include <osg/BlendColor>
#include <osg/BlendEquation>

#include "spinApp.h"
#include "spinBaseContext.h"
#include "spinServerContext.h"
#include "SceneManager.h"
#include "ReferencedNode.h"
#include "SwitchNode.h"

#include <cppintrospection/Value>
#include <cppintrospection/Type>

extern pthread_mutex_t sceneMutex;

namespace spin
{

// ***********************************************************
// constructor (one arg required: the node ID)
ReferencedNode::ReferencedNode (SceneManager *sceneManager, const char *initID) :
    contextString_("NULL")
{
    id_ = gensym(initID);
    id_->s_thing = this;
    id_->s_type = REFERENCED_NODE;

    nodeType_ = "ReferencedNode";

    this->setName(std::string(id_->s_name));

    // set some initial symbols:
    //parent_ = WORLD_SYMBOL;
    //newParent_ = WORLD_SYMBOL;

    scheduleForDeletion_ = false;
    subgraphAlpha_ = 1.0;

    // When children are attached to this, they get added to the attachmentNode:
    attachmentNode_ = this;

    // We need to set up a callback. This should be on the topmost node, so that during node
    // traversal, we update our parameters before anything is drawn.
    //this->setUserData( dynamic_cast<osg::Referenced*>(this) );
    this->setUserData(new ReferencedNode_data(this));
    this->setUpdateCallback(new ReferencedNode_callback);

    // tell OSG that this object will be updated dynamically
    this->setDataVariance(osg::Object::DYNAMIC);

    // set initial nodepath:
    currentNodePath_.clear();

    if (this->getID()=="world")
    {
        this->setNodeMask(INTERACTIVE_NODE_MASK);
    }
    else
    {
        registerNode(sceneManager);
        this->setNodeMask(GEOMETRIC_NODE_MASK); // nodemask info in spinUtil.h
        attachTo("world");
    }


}

// ***********************************************************
// destructor
ReferencedNode::~ReferencedNode()
{
    std::string oscPattern = "/SPIN/" + spinApp::Instance().getSceneID() + "/" + this->getID();

    if (! spinBaseContext::signalStop)
    {
        // unregister with OSC parser:
        std::vector<lo_server>::iterator it;
        for (it = spinApp::Instance().getContext()->lo_rxServs_.begin(); it != spinApp::Instance().getContext()->lo_rxServs_.end(); ++it)
        {
            lo_server_del_method((*it), oscPattern.c_str(), NULL);
        }
        lo_server_del_method(spinApp::Instance().getContext()->lo_tcpRxServer_, oscPattern.c_str(), NULL);
    }
    id_->s_thing = 0;
}

std::string ReferencedNode::getOSCPath() const
{
    return std::string("/SPIN/" + spinApp::Instance().getSceneID()+"/"+ getID());
}

void ReferencedNode::registerNode(SceneManager *s)
{
    sceneManager_ = s;

    // register with OSC parser:
    std::string oscPattern = "/SPIN/" + sceneManager_->sceneID + "/" + this->getID();
    std::vector<lo_server>::iterator it;
    for (it = spinApp::Instance().getContext()->lo_rxServs_.begin(); it != spinApp::Instance().getContext()->lo_rxServs_.end(); ++it)
    {
        lo_server_add_method((*it),
                             oscPattern.c_str(),
                             NULL,
                             spinBaseContext::nodeCallback,
                             (void*)id_);
    }

    // and with the TCP receiver in the server case:
    lo_server_add_method(spinApp::Instance().getContext()->lo_tcpRxServer_,
                         oscPattern.c_str(),
                         NULL,
                         spinBaseContext::nodeCallback,
                         (void*)id_);

}

void ReferencedNode::callbackUpdate(osg::NodeVisitor* nv)
{
    callCronScripts();
}

void ReferencedNode::attachTo (const char* parentID)
{
    if ((getID()=="world") || (std::string(parentID) == "NULL"))
        return;

    osg::ref_ptr<ReferencedNode> newParentNode = sceneManager_->getNode(parentID);

    if (newParentNode.valid())
    {
        if (! (newParentNode->getAttachmentNode()->containsNode(this)))
        {
            pthread_mutex_lock(&sceneMutex);
            newParentNode->getAttachmentNode()->addChild(this);
            pthread_mutex_unlock(&sceneMutex);

            // add to parent list:
            this->parentNodes_.push_back(newParentNode);

            // remove node from current parent (make sure to release the mutex first!)
            /*
            if (this->parent_ != this->newParent_)
            {
                this->detach();
            }
            */

            // update currentNodePath:
            this->updateNodePath();

            // broadcast this change to any remote clients:
            BROADCAST(this, "ss", "attachTo", parentID);

            // send a parentChange message to clients who are only listening to scene
            // messages (ie, they are not filtering every single node message).
            // TODO: do this via TCP?
            spinApp::Instance().BroadcastSceneMessage("ssss", "graphChange", "attach", this->getID().c_str(), parentID, SPIN_ARGS_END);

        }
    }
}

void ReferencedNode::detachFrom(const char* parentID)
{
    if (getID()=="world") return;

    // detach from all parents if the "*" wildcard is specified:
    if (std::string(parentID)=="*")
    {
        while (parentNodes_.size())
        {
            std::string pID = parentNodes_[0]->getID();

            if (parentNodes_[0]->attachmentNode_->containsNode(this))
            {
                pthread_mutex_lock(&sceneMutex);
                parentNodes_[0]->attachmentNode_->removeChild(this);
                pthread_mutex_unlock(&sceneMutex);
            }
            parentNodes_.erase(parentNodes_.begin());

            BROADCAST(this, "ss", "detachFrom", pID.c_str());
            spinApp::Instance().BroadcastSceneMessage("ssss", "graphChange", "detach", this->getID().c_str(), pID.c_str(), SPIN_ARGS_END);

        }

        this->updateNodePath();
    }

    // otherwise find the parent node and detach this from that parent:
    else
    {
        osg::ref_ptr<ReferencedNode> pNode = sceneManager_->getNode(parentID);
        if (pNode.valid())
        {
            if (pNode->attachmentNode_->containsNode(this))
            {
                pthread_mutex_lock(&sceneMutex);
                pNode->attachmentNode_->removeChild(this);
                pthread_mutex_unlock(&sceneMutex);
            }

            // remove from parent list:
            nodeListType::iterator iter;
            for (iter=parentNodes_.begin(); iter!=parentNodes_.end(); ++iter)
            {
                if ((*iter).get() == pNode.get())
                {
                    parentNodes_.erase(iter);
                    break;
                }
            }

            this->updateNodePath();

            BROADCAST(this, "ss", "detachFrom", parentID);
            spinApp::Instance().BroadcastSceneMessage("ssss", "graphChange", "detach", this->getID().c_str(), parentID, SPIN_ARGS_END);

        }
    }
}

bool ReferencedNode::inGraph()
{
    // for each parent, do a recursive check:
    nodeListType::iterator iter;
    for (iter=parentNodes_.begin(); iter!=parentNodes_.end(); ++iter)
    {
        // if the parent is world, then this node is surely in the scene:
        if ((*iter)->getID() == "world")
            return true;

        // if the parent is a switch node, check if it is in the scene, but only
        // if this node is enabled:
        SwitchNode *sw = dynamic_cast<SwitchNode*>((*iter).get());
        if (sw)
        {
            if ( (sw->isEnabled(this)) && (sw->inGraph()) )
                return sw->inGraph();
        }

        // for all other nodes, if the parent is in the scene, so are we:
        else if ((*iter)->inGraph())
            return true;
    }

    // no parents are in the scene, so we are not:
    return false;
}

std::string ReferencedNode::getParentID(int i) const
{
    if (i>=0 && i<parentNodes_.size())
        return parentNodes_[i]->getID();
    else
        return "NULL";
}
/*
osg::Group* ReferencedNode::getParentOSGGroup(int i)
{
    return osg::Group::getParent(i);
}
*/
ReferencedNode* ReferencedNode::getParentNode(int i)
{
    if (i>=0 && i<parentNodes_.size())
        return parentNodes_[i].get();
    else
        return NULL;
}

std::vector<ReferencedNode*> ReferencedNode::getChildren()
{
    std::vector<ReferencedNode*> children;

    for (int i=0; i < this->attachmentNode_->getNumChildren(); i++)
    {
        ReferencedNode *n = dynamic_cast<ReferencedNode*>(this->attachmentNode_->getChild(i));
        if (n)
        {
            children.push_back(n);
        }
    }

    return children;
}


// IMPORTANT:
// subclasses of ReferencedNode are allowed to contain complicated subgraphs, and
// can also change their attachmentNode so that children are attached anywhere
// in this subgraph. If that is the case, the updateNodePath() function MUST be
// overridden, and extra nodes must be manually pushed onto the currentNodePath_.

void ReferencedNode::updateNodePath(bool updateChildren)
{
    currentNodePath_.clear();
    osg::ref_ptr<ReferencedNode> p = parentNodes_[0];
    if (p && (p->getID() != "world") && (p->getID() != "NULL"))
    {
        currentNodePath_ = p->currentNodePath_; // this does a copy
    }

    // this nodePath only stores the path until this node (osg::Group).
    currentNodePath_.push_back(this);

    // Now update NodePaths for all children if the updateChildren flag is set.
    // For some derived nodes, they may want to control how they control the
    // update of children (eg, only after their nodepath is added).
    if (updateChildren)
        updateChildNodePaths();
}


void ReferencedNode::updateChildNodePaths()
{
    std::vector<ReferencedNode*> children = this->getChildren();
    std::vector<ReferencedNode*>::iterator childIter;
    for (childIter = children.begin(); childIter != children.end() ; childIter++)
    {
        (*childIter)->updateNodePath();
    }
}

int ReferencedNode::setAttachmentNode(osg::Group *n)
{
    if (n)
    {
        attachmentNode_ = n;

        // update the nodepath now that we've defined a new attachmentNode
        this->updateNodePath();
        return 1;
    }
    return 0;
}

bool ReferencedNode::legalParent (t_symbol *newParent)
{
    if (newParent == this->id_)
    {
        return false;
    }
    else
    {
        std::vector<ReferencedNode*> children = this->getChildren();
        std::vector<ReferencedNode*>::iterator childIter;
        for (childIter = children.begin(); childIter != children.end() ; childIter++)
        {
            if ((*childIter)->id_ == newParent)
                return false;
        }
    }
    return true;
}

void ReferencedNode::setParent (const char* newvalue)
{
    // setParent is a legacy method but is still a useful command. We'll assume
    // that the user wants to remove any previous attachments and ONLY attach
    // the node to one parent. Thus, we first call detachFrom("*") to remove it
    // from any existing parents, then we call attachTo.
    //
    // NOTE: if the node is attached to many places, some of the detachFrom
    // broadcasts might get lost (due to UDP), so for extra redundancy, we
    // will also broadcast the setParent message and hope that the client gets
    // it and calls an extra detachFrom("*") locally.

    this->detachFrom("*");
    BROADCAST(this, "ss", "setParent", newvalue); // redundant
    this->attachTo(newvalue);
}

void ReferencedNode::setContext (const char *newvalue)
{
    contextString_ = std::string(newvalue);
    BROADCAST(this, "ss", "setContext", getContext());
}

void ReferencedNode::setAlpha (float alpha)
{
    if (subgraphAlpha_ == alpha)
        return;

    subgraphAlpha_ = alpha;
    if (subgraphAlpha_ < 0.0)
        subgraphAlpha_ = 0.0;
    else if (subgraphAlpha_ > 1.0)
        subgraphAlpha_ = 1.0;

    osg::StateSet *ss = this->getOrCreateStateSet();
    ss->setDataVariance(osg::Object::DYNAMIC);

    // turn on blending and tell OSG to sort meshes before displaying them
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);

    osg::BlendFunc *blendFunc = new osg::BlendFunc();
    osg::BlendColor *blendColor= new osg::BlendColor(osg::Vec4(1, 1, 1, subgraphAlpha_));

    blendFunc->setDataVariance(osg::Object::DYNAMIC);
    blendColor->setDataVariance(osg::Object::DYNAMIC);

    blendFunc->setSource(osg::BlendFunc::CONSTANT_ALPHA);
    blendFunc->setDestination(osg::BlendFunc::ONE_MINUS_CONSTANT_ALPHA);
    ss->setAttributeAndModes(blendFunc, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    ss->setAttributeAndModes(blendColor, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

    this->osg::Group::setStateSet(ss);

    /*
    osg::BlendEquation* blendEquation = new osg::BlendEquation(osg::BlendEquation::FUNC_ADD);
    blendEquation->setDataVariance(osg::Object::DYNAMIC);

    //blendEquation->setEquation(osg::BlendEquation::FUNC_ADD);
    //blendEquation->setEquation(osg::BlendEquation::FUNC_SUBTRACT);
    //blendEquation->setEquation(osg::BlendEquation::FUNC_REVERSE_SUBTRACT);
    //blendEquation->setEquation(osg::BlendEquation::RGBA_MIN);
    //blendEquation->setEquation(osg::BlendEquation::RGBA_MAX);
    blendEquation->setEquation(osg::BlendEquation::ALPHA_MIN);
    //blendEquation->setEquation(osg::BlendEquation::ALPHA_MAX);
    //blendEquation->setEquation(osg::BlendEquation::LOGIC_OP);

    ss->setAttributeAndModes(blendEquation,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    */

    std::cout << "set alpha for " << this->id_->s_name << " to " << subgraphAlpha_ << std::endl;
}

void ReferencedNode::setParam (const char *paramName, const char *paramValue)
{
    //std::cout << id_->s_name << " got setParam: " << paramValue << std::endl;
    stringParams_[std::string(paramName)] = std::string(paramValue);
    BROADCAST(this, "sss", "setParam", paramName, paramValue);
}

void ReferencedNode::setParam (const char *paramName, float paramValue)
{
    floatParams_[std::string(paramName)] = paramValue;
    BROADCAST(this, "ssf", "setParam", paramName, paramValue);
}

// -----------------------------------------------------------------------------

void ReferencedNode::setStateSetFromFile(const char* filename)
{
    osg::ref_ptr<ReferencedStateSet> ss = sceneManager_->createStateSet(filename);
    if (ss.valid())
    {
        if (ss->getIDSymbol() == stateset_) return; // we're already using that stateset
        stateset_ = ss->getIDSymbol();
        updateStateSet();
        BROADCAST(this, "ss", "setStateSet", getStateSet());
    }
}

void ReferencedNode::setStateSet (const char* s)
{
    if (gensym(s)==stateset_) return;

    osg::ref_ptr<ReferencedStateSet> ss = sceneManager_->getStateSet(s);
    if (ss.valid())
    {
        stateset_ = ss->getIDSymbol();

        BROADCAST(this, "ss", "setStateSet", getStateSet());
    }

    updateStateSet();
}

void ReferencedNode::updateStateSet()
{
    osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
    if (ss.valid()) osg::Group::setStateSet( ss.get() );

    // if not valid, create a new (empty) stateset (ie, clear the previous state)
    else osg::Group::setStateSet(new osg::StateSet());
}

// -----------------------------------------------------------------------------


void ReferencedNode::debug()
{
    lo_arg **args;
    int i, argc;
    char *argTypes;

    std::cout << "****************************************" << std::endl;
    std::cout << "************* NODE  DEBUG: *************" << std::endl;
    std::cout << "\nnode: " << getID() << ", type: " << getNodeType() << std::endl;

    std::cout << "   Node path:" << std::endl;
    for (osg::NodePath::iterator itr = currentNodePath_.begin(); itr != currentNodePath_.end(); ++itr)
    {
        std::cout << "   -> " << (*itr)->getName() << std::endl;
    }

    const osg::BoundingSphere& bs = this->getBound();
    std::cout << "   Subgraph centroid: " << stringify(bs.center()) << std::endl;
    std::cout << "   Subgraph radius: " << bs.radius() << std::endl;

    std::vector<lo_message> nodeState = this->getState();
    std::vector<lo_message>::iterator nodeStateIterator;
    for (nodeStateIterator = nodeState.begin(); nodeStateIterator != nodeState.end(); ++nodeStateIterator)
    {
        argTypes = lo_message_get_types(*nodeStateIterator);
        argc = lo_message_get_argc(*nodeStateIterator);
        args = lo_message_get_argv(*nodeStateIterator);

        std::cout << "  ";
        for (i = 0; i < argc; i++)
        {
            std::cout << " ";
            if (lo_is_numerical_type((lo_type)argTypes[i]))
            {
                std::cout << (float) lo_hires_val( (lo_type)argTypes[i], args[i] );
            } else if (strlen((char*) args[i]))
            {
                std::cout << (char*) args[i];
            }
            else
            {
                std::cout << "NULL";
            }
        }
        std::cout << std::endl;
    }

    std::vector<ReferencedNode*> children = this->getChildren();
    if (! children.empty())
    {
        std::cout << "   children:" << std::endl;
        std::vector<ReferencedNode*>::iterator childIter;
        for (childIter = children.begin(); childIter != children.end(); ++childIter)
        {
            std::cout << "      " << (*childIter)->getID() << std::endl;
        }
    }
    BROADCAST(this, "s", "debug");
}

std::vector<lo_message> ReferencedNode::getState() const
{
    std::vector<lo_message> ret;

    lo_message msg;

    for (int i=0; i<this->getNumParents(); i++)
    {
        msg = lo_message_new();
        lo_message_add(msg, "ss", "attachTo", this->getParentID(i).c_str());
        ret.push_back(msg);
    }

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setContext", this->getContext());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setAlpha", this->getAlpha());
    ret.push_back(msg);

    stringParamType::const_iterator stringIter;
    for (stringIter = stringParams_.begin(); stringIter != stringParams_.end(); stringIter++ )
    {
        msg = lo_message_new();
        lo_message_add(msg, "sss", "setParam", (*stringIter).first.c_str(), (const char*)(*stringIter).second.c_str());
        ret.push_back(msg);
    }

    floatParamType::const_iterator floatIter;
    for (floatIter = floatParams_.begin(); floatIter != floatParams_.end(); floatIter++ )
    {
        msg = lo_message_new();
        lo_message_add(msg, "ssf", "setParam", (*floatIter).first.c_str(), (*floatIter).second);
        ret.push_back(msg);
    }

    for ( CronScriptList::const_iterator it = _cronScriptList.begin(); it != _cronScriptList.end(); it++ )
    {
        if (! it->second )
            continue;
        msg = lo_message_new();
        lo_message_add(msg, "ssssf", "addCronScript",
                       it->second->serverSide ? "S" : "C",
                       it->first.c_str(), // the label is the key of the pair!
                       it->second->path.c_str(),
                       it->second->freq);
        if (it->second->params != "")
            lo_message_add_string(msg, it->second->params.c_str() );
        ret.push_back(msg);
    }

    for( EventScriptList::const_iterator it = _eventScriptList.begin(); it != _eventScriptList.end(); it++ )
    {
        if (! it->second )
            continue;
        msg = lo_message_new();
        lo_message_add(msg, "sssss", "addEventScript",
                       it->second->serverSide ? "S" : "C",
                       it->first.c_str(), // the label!
                       it->second->eventName.c_str(),
                       it->second->path.c_str() );
        if (it->second->params != "")
            lo_message_add_string(msg, it->second->params.c_str());
        ret.push_back(msg);
    }

    return ret;
}

void ReferencedNode::stateDump()
{
    std::vector<lo_message> stateBundle = this->getState();

    lo_message msg = lo_message_new();
    lo_message_add_string(msg, "parentList");
    for (int i=0; i<this->getNumParents(); i++)
    {
        lo_message_add_string(msg, this->getParentID(i).c_str());
    }
    stateBundle.push_back(msg);

    spinApp::Instance().NodeBundle(this->getID(), stateBundle);
}


void ReferencedNode::stateDump(lo_address txAddr)
{
    std::vector<lo_message> stateBundle = this->getState();

    lo_message msg = lo_message_new();
    lo_message_add_string(msg, "parentList");
    for (int i=0; i<this->getNumParents(); i++)
    {
        lo_message_add_string(msg, this->getParentID(i).c_str());
    }
    stateBundle.push_back(msg);

    spinApp::Instance().NodeBundle(this->getID(), stateBundle, txAddr);
}

bool ReferencedNode::addCronScript( bool serverSide, const std::string& label, const std::string& scriptPath,
                                    double freq, const std::string& params )
{
#ifndef DISABLE_PYTHON

   // do we already have a script with the same label?
    CronScriptList::iterator it;
    it = _cronScriptList.find(std::string(label));
    if (it != _cronScriptList.end())
        return false; // yes! don't reload it.

    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();

    std::string sf = osgDB::findDataFile( getAbsolutePath(scriptPath) );
    if ( sf.empty() ) {
        std::cout << "script file '" << getAbsolutePath(scriptPath) << "' not found." << std::endl;
        return false;
    }

    std::cout << "Loading script: " << sf << std::endl;

    boost::python::object s, p;

    char cmd[512];
    //osg::Timer_t utick = timer->tick();
    unsigned long long utick =  (unsigned long long) timer->tick();
    std::string pyModule, pyScript, pyClassName;

    CronScript* cs = new CronScript;
    cs->path = sf;
    cs->serverSide = serverSide;
    cs->params = params;
    cs->freq = freq;
    cs->enabled = false;

    if ( spin.getContext()->isServer() != serverSide )
    {
        cs->lastRun = 0;
        cs->enabled = false;
        cs->pyModule = "";
        cs->pyScript = "";
        _cronScriptList.insert( std::pair<const std::string, CronScript*>( std::string(label), cs ) );
        return true;
    }

    try
    {
        sprintf( cmd, "mod%llx", utick );
        pyModule = cmd;
        sprintf( cmd, "script%llx", utick );
        pyScript = cmd;

        sprintf( cmd, "%s = spin.load_module('%s')", pyModule.c_str(), sf.c_str() );
        exec( cmd, spin._pyNamespace, spin._pyNamespace );

        s = spin._pyNamespace[pyModule.c_str()];
        char* cls = boost::python::extract<char*>(s.attr("__spin_behavior_class__") );

        sprintf( cmd, "%s = %s.%s('%s' %s)", pyScript.c_str(), pyModule.c_str(), cls, id_->s_name, params.c_str() );

        //std::cout << "Python cmd: " << cmd << std::endl;
        exec( cmd, spin._pyNamespace, spin._pyNamespace );

        s = spin._pyNamespace[pyScript.c_str()];
        p = s.attr( "run" );

        cs->freq = freq;
        cs->lastRun = timer->time_s() - 1.0/freq;
        cs->run = p;
        cs->enabled = true;
        cs->pyModule = pyModule;
        cs->pyScript = pyScript;

        _cronScriptList.insert( std::pair<const std::string, CronScript*>( std::string(label), cs ) );

    }
    catch ( boost::python::error_already_set const & )
    {
        std::cout << "Python error: " << std::endl;
        PyErr_Print();
        PyErr_Clear();
        return false;
    }
    catch ( std::exception& e )
    {
        std::cout << "Python error: " << e.what() << std::endl;
        return false;
    }
    catch(...)
    {                        // catch all other exceptions
        std::cout << "Python error... Caught... something??\n";
        return false;
    }
    return true;

#else
    std::cout << "Python interpreter is disabled. Could not addCronScript to " << getNodeType() << ": " << getID() << std::endl;
    return false;
#endif
}

bool ReferencedNode::callCronScripts()
{
#ifndef DISABLE_PYTHON
    if (_cronScriptList.empty())
        return false;
    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();
    double d;

    try
    {
        for (CronScriptList::iterator it = _cronScriptList.begin(); it != _cronScriptList.end(); it++)
        {
            if (! it->second )
                continue;
            if (it->second->serverSide != spin.getContext()->isServer())
                continue;
            d = 1.0 / it->second->freq;
            if (timer->time_s() >= d + it->second->lastRun)
            {
                it->second->lastRun += d;
                if (it->second->enabled )
                    it->second->run();
            }
        }
    }
    catch ( boost::python::error_already_set & )
    {
        std::cout << spin.getCurrentPyException() << std::endl;
        std::cout << "Python error: [";
        PyErr_Print();
        PyErr_Clear();
        std::cout << "]" << std::endl;
        return false;
    }
    catch ( std::exception& e )
    {
        std::cout << "Python error: "<< e.what() << std::endl;
        return false;
    }
    catch(...)
    {                        // catch all other exceptions
        std::cout << "Python error: Caught... something??\n";
        return false;
    }
    return true;
#else
    return false;
#endif
}

bool ReferencedNode::enableCronScript( const char* label, int enable )
{
    CronScriptList::iterator it;
    it = _cronScriptList.find(std::string(label));

    if (it != _cronScriptList.end())
    {
        it->second->enabled = (enable != 0);
        return true;
    }
    return false;
}

bool ReferencedNode::removeCronScript( const char* label )
{
    spinApp &spin = spinApp::Instance();
    char cmd[100];
    CronScriptList::iterator it;
    it = _cronScriptList.find( std::string(label) );
    if (it != _cronScriptList.end())
    {
        if (spin.getContext()->isServer() == it->second->serverSide)
        {
            sprintf( cmd, "del %s", it->second->pyScript.c_str() );
            if (! spin.execPython(cmd))
                return false;
        }
        delete(it->second);
        it->second = NULL;
        _cronScriptList.erase(it);
        return true;
    }
    return false;
}

bool ReferencedNode::addEventScript( bool serverSide, const std::string& label, const std::string& eventName,
                                     const std::string& scriptPath, const std::string& params )
{
#ifndef DISABLE_PYTHON

    // do we already have a script with the same label?
    EventScriptList::iterator it;
    it = _eventScriptList.find( std::string(label) );
    if (it != _eventScriptList.end())
        return false; // yes! don't reload it.

    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();

    std::string sf = osgDB::findDataFile( getAbsolutePath(scriptPath) );
    if ( sf.empty() ) {
        std::cout << "script file '" << getAbsolutePath(scriptPath) << "' not found." << std::endl;
        return false;
    }

    boost::python::object s, p;
    char cmd[512];
    unsigned long long utick =  (unsigned long long) timer->tick();
    std::string pyModule, pyScript;

    EventScript* es = new EventScript;
    es->path = sf;
    es->serverSide = serverSide;
    es->params = params;
    es->eventName = eventName;
    es->enabled = false;

    if ( spin.getContext()->isServer() != serverSide )
    {
        es->enabled = false;
        es->pyModule = "";
        es->pyScript = "";
        _eventScriptList.insert( std::pair<const std::string, EventScript*>( std::string(label), es ) );
        return true;
    }

    try
    {
        sprintf( cmd, "mod%llx", utick );
        pyModule = cmd;
        sprintf( cmd, "script%llx", utick );
        pyScript = cmd;

        //sprintf(cmd, "mod%llx = spin.load_module('%s')", utick, sf.c_str());
        sprintf( cmd, "%s = spin.load_module('%s')", pyModule.c_str(), sf.c_str() );
        //std::cout << "Python cmd: " << cmd << std::endl;
        exec(cmd, spin._pyNamespace, spin._pyNamespace);

        s = spin._pyNamespace[pyModule.c_str()];
        char* cls = boost::python::extract<char*>(s.attr("__spin_behavior_class__") );

        //sprintf( cmd, "script%llx = mod%llx.Script('%s' %s)", utick, utick, id_->s_name, params.c_str() );
        sprintf( cmd, "%s = %s.%s('%s' %s)", pyScript.c_str(), pyModule.c_str(), cls, id_->s_name, params.c_str() );
        //std::cout << "Python cmd: " << cmd << std::endl;
        exec(cmd, spin._pyNamespace, spin._pyNamespace);

        //sprintf(cmd, "script%llx", utick);
        //s = spin._pyNamespace[cmd];
        s = spin._pyNamespace[pyScript.c_str()];
        p = s.attr("run");

        es->run = p;
        es->enabled = true;
        es->pyModule = pyModule;
        es->pyScript = pyScript;

        _eventScriptList.insert( std::pair<const std::string, EventScript*>( std::string(label), es ) );

    } catch (boost::python::error_already_set const & ) {
        std::cout << "Python error: " << std::endl;
        PyErr_Print();
        PyErr_Clear();
        return false;
    } catch ( std::exception& e ) {
        std::cout << "Python error: " << e.what() << std::endl;

        return false;
    } catch(...) {                        // catch all other exceptions
        std::cout << "Python error: Caught... something??\n";
        return false;
    }

    return true;

#else
    std::cout << "Python interpreter is disabled. Could not addEventScript to " << getNodeType() << ": " << id_->s_name << std::endl;
    return false;
#endif
}

bool ReferencedNode::callEventScript( const std::string& eventName,
                                      cppintrospection::ValueList& args )
{
#ifndef DISABLE_PYTHON
    if (_eventScriptList.empty())
        return false;
    spinApp &spin = spinApp::Instance();
    boost::python::list argList;
    bool argListBuilt = false;
    bool eventScriptCalled = false;

    for( EventScriptList::iterator it = _eventScriptList.begin();
         it != _eventScriptList.end(); it++ )
    {

        if (! it->second )
            continue;
        if (it->second->eventName != eventName )
            continue;
        if (it->second->serverSide != spin.getContext()->isServer() )
            continue;
        if (! it->second->enabled )
            continue;

        if (! argListBuilt )
        {
            try
            {
                for ( size_t i = 0; i < args.size(); i++ )
                {
                    const std::type_info* argt = &args[i].getType().getStdTypeInfo();

                    if ( *argt == typeid(int) )
                    {
                        argList.append( cppintrospection::variant_cast<int>(args[i]) );
                    }
                    else if ( *argt == typeid(float) )
                    {
                        argList.append( cppintrospection::variant_cast<float>(args[i]) );
                    }
                    else if ( *argt == typeid(double) )
                    {
                        argList.append( cppintrospection::variant_cast<double>(args[i]) );
                    }
                    else if ( *argt == typeid(std::string) )
                    {
                        argList.append( cppintrospection::variant_cast<std::string>(args[i]).c_str() );
                    }
                    else if ( *argt == typeid(const char*) )
                    {
                        argList.append( cppintrospection::variant_cast<const char*>(args[i]) );
                    }
                    else if (*argt == typeid(char*))
                    {
                        argList.append( cppintrospection::variant_cast<char*>(args[i]) );
                    }
                    else
                    {
                        std::cout << "callEventScript: unsupported argument type: " << argt->name() << std::endl;
                        return false;
                    }
                }
            }
            catch (...)
            {
                std::cout << "callEventScript: something went wrong" << std::endl;
                return false;
            }
        }

        try
        {
            it->second->run( eventName.c_str(), argList );
            eventScriptCalled = true;
        } catch (boost::python::error_already_set const & ) {
            std::cout << "0: Python error: " << std::endl;
            PyErr_Print();
            PyErr_Clear();
            return false;
        } catch ( std::exception& e ) {
            std::cout << "Python error: " << e.what() << std::endl;
            return false;
        } catch(...) {
            std::cout << "Python error: Caught... something??\n";
            return false;
        }
    }
    return eventScriptCalled;

#else
    return false;
#endif
}

bool ReferencedNode::enableEventScript(const char* label, int enable)
{
    EventScriptList::iterator it;
    it = _eventScriptList.find( std::string(label) );

    if (it != _eventScriptList.end())
    {
        it->second->enabled = (enable != 0);
        return true;
    }
    return false;
}

bool ReferencedNode::removeEventScript(const char* label)
{
    spinApp &spin = spinApp::Instance();
    char cmd[100];
    EventScriptList::iterator it;
    it = _eventScriptList.find( std::string(label) );

    if (it != _eventScriptList.end())
    {
        if (spin.getContext()->isServer() == it->second->serverSide)
        {
            sprintf( cmd, "del %s", it->second->pyScript.c_str() );
            if (! spin.execPython(cmd))
                return false;
        }
        delete(it->second);
        it->second = NULL;
        _eventScriptList.erase( it );
        return true;
    }
    return false;
}

} // end of namespace spin

