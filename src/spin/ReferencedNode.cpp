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

#include "spinApp.h"
#include "spinBaseContext.h"
#include "ReferencedNode.h"
#include "SceneManager.h"


using namespace std;

extern pthread_mutex_t sceneMutex;

// ***********************************************************
// constructor (one arg required: the node ID)
ReferencedNode::ReferencedNode (SceneManager *sceneManager, char *initID) :
    contextString("NULL")
{

    id = gensym(initID);
    id->s_thing = this;
    id->s_type = REFERENCED_NODE;

    nodeType = "ReferencedNode";

    this->setName(string(id->s_name) + ".ReferencedNode");

    // set some initial symbols:
    parent = WORLD_SYMBOL;
    newParent = WORLD_SYMBOL;

    textFlag = false;
    scheduleForDeletion = false;


    // When children are attached to this, they get added to the attachmentNode:
    attachmentNode = this;

    // We need to set up a callback. This should be on the topmost node, so that during node
    // traversal, we update our parameters before anything is drawn.
    this->setUserData( dynamic_cast<osg::Referenced*>(this) );
    this->setUpdateCallback(new ReferencedNode_callback);

    // set initial nodepath:
    currentNodePath.clear();

    registerNode(sceneManager);

    this->setNodeMask(GEOMETRIC_NODE_MASK); // nodemask info in spinUtil.h

    attach();

}

// ***********************************************************
// destructor
ReferencedNode::~ReferencedNode()
{

    if (!spinBaseContext::signalStop)
    {
        // unregister with OSC parser:
        string oscPattern = "/SPIN/" + spinApp::Instance().getSceneID() + "/" + string(id->s_name);
        lo_server_del_method(spinApp::Instance().getContext()->lo_rxServ_, oscPattern.c_str(), NULL);
    }

    id->s_thing = 0;
}


void ReferencedNode::registerNode(SceneManager *s)
{
    sceneManager = s;
    mediaManager = sceneManager->mediaManager;

    // register with OSC parser:
    string oscPattern = "/SPIN/" + sceneManager->sceneID + "/" + string(id->s_name);
    lo_server_add_method(spinApp::Instance().getContext()->lo_rxServ_, oscPattern.c_str(),
            NULL, spinBaseContext::nodeCallback, (void*)id);
#ifdef OSCDEBUG
    std::cout << "oscParser registered: " << oscPattern << std::endl;
#endif
}

// *****************************************************************************

void ReferencedNode::callbackUpdate()
{
    callCronScripts();


}


// *****************************************************************************

void ReferencedNode::attach()
{
    if (this->newParent==NULL_SYMBOL) return;

    pthread_mutex_lock(&sceneMutex);

    osg::ref_ptr<ReferencedNode> newParentNode = dynamic_cast<ReferencedNode*>(newParent->s_thing);

    // if the parent is invalid (which will be the case, for example, if the user
    // specified 'world' as the parent), we attach to the worldNode:
    if (!newParentNode.valid())
    {
        if (!(sceneManager->worldNode->containsNode( this ))) sceneManager->worldNode->addChild(this);
        this->newParent = WORLD_SYMBOL;
    }

    // Otherwise attach to the parent:
    else
    {
        if (!(newParentNode->attachmentNode->containsNode(this)))
        {
            newParentNode->attachmentNode->addChild(this);
            newParentNode->as_addChild(this);
        }
    }

    pthread_mutex_unlock(&sceneMutex);

    // remove node from current parent (make sure to release the mutex first!)
    if (this->parent != this->newParent)
    {
        this->detach();
    }

    // update the new parent symbols:
    this->parent = this->newParent;
    this->newParent = NULL_SYMBOL;

    // update currentNodePath:
    this->updateNodePath();

    // broadcast this change to any remote clients:
    BROADCAST(this, "ss", "setParent", this->parent->s_name);

}

// ***********************************************************
// remove this node from the scenegraph:
void ReferencedNode::detach()
{
    pthread_mutex_lock(&sceneMutex);

    if (parent == WORLD_SYMBOL)
    {
        if (sceneManager->worldNode->containsNode(this)) sceneManager->worldNode->removeChild(this);
    }

    else {
        osg::ref_ptr<ReferencedNode> pNode = dynamic_cast<ReferencedNode*>(parent->s_thing);
        if (pNode.valid())
        {
            if (pNode->attachmentNode->containsNode(this))
            {
                pNode->attachmentNode->removeChild(this);
                pNode->as_removeChild(this);
            }
        }
    }

    pthread_mutex_unlock(&sceneMutex);
}


// *****************************************************************************
// IMPORTANT:
// subclasses of ReferencedNode are allowed to contain complicated subgraphs, and
// can also change their attachmentNode so that children are attached anywhere
// in this subgraph. If that is the case, the updateNodePath() function MUST be
// overridden, and extra nodes must be manually pushed onto the currentNodePath.

void ReferencedNode::updateNodePath()
{
    currentNodePath.clear();
    if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
    {
        osg::ref_ptr<ReferencedNode> parentNode = dynamic_cast<ReferencedNode*>(parent->s_thing);
        if (parentNode.valid())
        {
            currentNodePath = parentNode->currentNodePath;
        }
    }

    // this nodePath only stores the path until this node (osg::Group).
    currentNodePath.push_back(this);

    // now update NodePaths for all children:
    updateChildNodePaths();

}


void ReferencedNode::updateChildNodePaths()
{
    vector<ReferencedNode*>::iterator childIter;
    for (childIter = children.begin(); childIter != children.end() ; childIter++)
    {
        (*childIter)->updateNodePath();
    }
}

int ReferencedNode::setAttachmentNode(osg::Group *n)
{
    if (n)
    {
        attachmentNode = n;

        // update the nodepath now that we've defined a new attachmentNode
        this->updateNodePath();

        return 1;

    }

    return 0;
}


// *****************************************************************************
ReferencedNode *ReferencedNode::as_getChild(ReferencedNode *child)
{
    vector<ReferencedNode*>::iterator iter;
    for (iter = children.begin(); iter != children.end() ; iter++)
    {
        if ((*iter) == child) return (*iter);
    }
    return NULL;
}

void ReferencedNode::as_addChild(ReferencedNode *child)
{
    children.push_back(child);
}

void ReferencedNode::as_removeChild(ReferencedNode *child)
{
    vector<ReferencedNode*>::iterator iter;
    for (iter = children.begin(); iter != children.end() ; iter++)
    {
        if ((*iter) == child)
        {
            children.erase(iter);
            break;
        }
    }
}



// *****************************************************************************
bool ReferencedNode::legalParent (t_symbol *newParent)
{
    vector<ReferencedNode*>::iterator childIter;

    if (newParent == this->id)
    {
        return false;
    }

    else
    {
        for (childIter = children.begin(); childIter != children.end() ; childIter++)
        {
            if ((*childIter)->id == newParent) return false;
        }
    }

    return true;
}

// *****************************************************************************

void ReferencedNode::setParent (const char *newvalue)
{
    t_symbol *s = gensym(newvalue);
    if (parent != s)
    {
        newParent = s;
        attach();
    }
}

void ReferencedNode::setContext (const char *newvalue)
{
    contextString = string(newvalue);
    BROADCAST(this, "ss", "setContext", getContext());
}

void ReferencedNode::setParam (const char *paramName, const char *paramValue)
{
    //std::cout << id->s_name << " got setParam: " << paramValue << std::endl;
    stringParams[string(paramName)] = string(paramValue);
    BROADCAST(this, "sss", "setParam", paramName, paramValue);
}

void ReferencedNode::setParam (const char *paramName, float paramValue)
{
    floatParams[string(paramName)] = paramValue;
    BROADCAST(this, "ssf", "setParam", paramName, paramValue);
}

// *****************************************************************************

void ReferencedNode::debug()
{
    lo_arg **args;
    int i, argc;
    char *argTypes;

    std::cout << "****************************************" << std::endl;
    std::cout << "************* NODE  DEBUG: *************" << std::endl;

    std::cout << "\nnode: " << id->s_name << ", type: " << nodeType << std::endl;

    vector<lo_message> nodeState = this->getState();
    vector<lo_message>::iterator nodeStateIterator;
    for (nodeStateIterator = nodeState.begin(); nodeStateIterator != nodeState.end(); ++nodeStateIterator)
    {
        argTypes = lo_message_get_types(*nodeStateIterator);
        argc = lo_message_get_argc(*nodeStateIterator);
        args = lo_message_get_argv(*nodeStateIterator);

        std::cout << "  ";
        for (i = 0; i<argc; i++) {
            std::cout << " ";
            if (lo_is_numerical_type((lo_type)argTypes[i]))
            {
                std::cout << (float) lo_hires_val( (lo_type)argTypes[i], args[i] );
            } else if (strlen((char*) args[i])) {
                std::cout << (char*) args[i];
            } else {
                std::cout << "NULL";
            }
        }
        std::cout << std::endl;
    }


    if (!this->children.empty())
    {
        std::cout << "   children:" << std::endl;
        vector<ReferencedNode*>::iterator childIter;
        for (childIter = this->children.begin(); childIter != this->children.end(); ++childIter)
        {
            std::cout << "      " << (*childIter)->id->s_name << std::endl;
        }
    }

    BROADCAST(this, "s", "debug");
}

std::vector<lo_message> ReferencedNode::getState ()
{
    std::vector<lo_message> ret;

    lo_message msg;

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setParent", this->getParent());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "ss", "setContext", this->getContext());
    ret.push_back(msg);

    stringParamType::iterator stringIter;
    for (stringIter=stringParams.begin(); stringIter!=stringParams.end(); stringIter++ )
    {
        msg = lo_message_new();
        lo_message_add(msg, "sss", "setParam", (*stringIter).first.c_str(), (const char*)(*stringIter).second.c_str());
        ret.push_back(msg);
    }

    floatParamType::iterator floatIter;
    for (floatIter=floatParams.begin(); floatIter!=floatParams.end(); floatIter++ )
    {
        msg = lo_message_new();
        lo_message_add(msg, "ssf", "setParam", (*floatIter).first.c_str(), (*floatIter).second);
        ret.push_back(msg);
    }


    for ( CronScriptList::iterator it = _cronScriptList.begin();
          it != _cronScriptList.end(); it++ ) {

        if ( !it->second ) continue;
        msg = lo_message_new();
        lo_message_add(msg, "ssssf", "addCronScript",
                       it->second->serverSide ? "S" : "C",
                       it->first.c_str(), // the label is the key of the pair!
                       it->second->path.c_str(),
                       it->second->freq);
        if (it->second->params != "") lo_message_add_string(msg, it->second->params.c_str() );
        ret.push_back(msg);
    }

    for( EventScriptList::iterator it = _eventScriptList.begin();
         it != _eventScriptList.end(); it++ ) {

        if ( !it->second ) continue;
        msg = lo_message_new();
        lo_message_add(msg, "sssss", "addEventScript",
                       it->second->serverSide ? "S" : "C",
                       it->first.c_str(), // the label!
                       it->second->eventName.c_str(),
                       it->second->path.c_str() );
        if (it->second->params != "") lo_message_add_string(msg, it->second->params.c_str() );
        ret.push_back(msg);

    }

    return ret;
}

// *****************************************************************************
void ReferencedNode::stateDump ()
{
    spinApp::Instance().NodeBundle(this->id, this->getState());
}

// *****************************************************************************
void ReferencedNode::stateDump(lo_address txAddr)
{
    spinApp::Instance().NodeBundle(this->id, this->getState(), txAddr);
}

std::string ReferencedNode::getID() const
{
    return std::string(id->s_name);
}

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

bool ReferencedNode::addCronScript( bool serverSide, const std::string& label, const std::string& scriptPath,
                                    double freq, const std::string& params )
{
   // do we already have a script with the same label?
    CronScriptList::iterator it;
    it = _cronScriptList.find( std::string(label) );
    if ( it != _cronScriptList.end() ) return false; // yes! don't reload it.

    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();

    std::string sf = osgDB::findDataFile( scriptPath );
    cout << "script file path is: " << sf << endl;

    boost::python::object s, p;

    char cmd[100];
    //osg::Timer_t utick = timer->tick();
    unsigned long long utick =  (unsigned long long) timer->tick();
    std::string pyModule, pyScript, pyClassName;

    CronScript* cs = new CronScript;
    cs->path = scriptPath;
    cs->serverSide = serverSide;
    cs->params = params;
    cs->freq = freq;
    cs->enabled = false;

    if ( spin.getContext()->isServer() != serverSide ) {
        cs->lastRun = 0;
        cs->enabled = false;
        cs->pyModule = "";
        cs->pyScript = "";
        _cronScriptList.insert( pair<const std::string, CronScript*>( std::string(label), cs ) );
        return true;
    }

    try {
        sprintf( cmd, "mod%llx", utick );
        pyModule = cmd;
        sprintf( cmd, "script%llx", utick );
        pyScript = cmd;

        sprintf( cmd, "%s = spin.load_module('%s')", pyModule.c_str(), sf.c_str() );
        exec( cmd, spin._pyNamespace, spin._pyNamespace );

        s = spin._pyNamespace[pyModule.c_str()];
        char* cls = boost::python::extract<char*>(s.attr("__spin_behavior_class__") );

        sprintf( cmd, "%s = %s.%s('%s' %s)", pyScript.c_str(), pyModule.c_str(), cls, id->s_name, params.c_str() );

        std::cout << "Python cmd: " << cmd << std::endl;
        exec( cmd, spin._pyNamespace, spin._pyNamespace );

        s = spin._pyNamespace[pyScript.c_str()];
        p = s.attr( "run" );

        cs->freq = freq;
        cs->lastRun = timer->time_s() - 1.0/freq;
        cs->run = p;
        cs->enabled = true;
        cs->pyModule = pyModule;
        cs->pyScript = pyScript;

        _cronScriptList.insert( pair<const std::string, CronScript*>( std::string(label), cs ) );

    } catch ( boost::python::error_already_set const & ) {
        std::cout << "Python error: " << std::endl;
        PyErr_Print();
        PyErr_Clear();
        return false;
    } catch ( std::exception& e ) {
        std::cout << "Python error: " << e.what() << std::endl;
        return false;
    } catch(...) {                        // catch all other exceptions
        std::cout << "Python error... Caught... something??\n";
        return false;
    }

    return true;

}

// *****************************************************************************

bool ReferencedNode::callCronScripts() {

    if (_cronScriptList.empty()) return false;
    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();
    double d;

    try {

        for ( CronScriptList::iterator it = _cronScriptList.begin();
              it != _cronScriptList.end(); it++ ) {

            if ( !it->second ) continue;
            if ( it->second->serverSide != spin.getContext()->isServer() ) continue;

            d = 1.0 / it->second->freq;
            if ( timer->time_s() >= d + it->second->lastRun ) {
                it->second->lastRun += d;
                if ( it->second->enabled ) it->second->run();
            }
        }


    } catch ( boost::python::error_already_set & ) {
        std::cout << spin.getCurrentPyException() << std::endl;
        std::cout << "Python error: [";
        PyErr_Print();
        PyErr_Clear();
        std::cout << "]" << std::endl;
        return false;
    } catch ( std::exception& e ) {
        std::cout << "Python error: "<< e.what() << std::endl;
        return false;
    } catch(...) {                        // catch all other exceptions
        std::cout << "Python error: Caught... something??\n";
        return false;
    }

    return true;
}

// *****************************************************************************

bool ReferencedNode::enableCronScript( const char* label, int enable ) {

    CronScriptList::iterator it;
    it = _cronScriptList.find( std::string(label) );

    if ( it != _cronScriptList.end() ) {
        it->second->enabled = (enable != 0);
        return true;
    }

    return false;
}

// *****************************************************************************

bool ReferencedNode::removeCronScript( const char* label ) {

    spinApp &spin = spinApp::Instance();
    char cmd[100];
    CronScriptList::iterator it;
    it = _cronScriptList.find( std::string(label) );

    if ( it != _cronScriptList.end() ) {

        if ( spin.getContext()->isServer() == it->second->serverSide ) {
            sprintf( cmd, "del %s", it->second->pyScript.c_str() );
            if ( !spin.execPython( cmd ) ) return false;
        }
        delete( it->second );
        it->second = NULL;
        _cronScriptList.erase( it );
        return true;
    }

    return false;
}



// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

bool ReferencedNode::addEventScript( bool serverSide, const std::string& label, const std::string& eventName,
                                     const std::string& scriptPath, const std::string& params ) {

    // do we already have a script with the same label?
    EventScriptList::iterator it;
    it = _eventScriptList.find( std::string(label) );
    if ( it != _eventScriptList.end() ) return false; // yes! don't reload it.

    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();

    std::string sf = osgDB::findDataFile( scriptPath );
    cout << "script file path is: " << sf << endl;

    boost::python::object s, p;
    char cmd[100];
    unsigned long long utick =  (unsigned long long) timer->tick();
    std::string pyModule, pyScript;

    EventScript* es = new EventScript;
    es->path = scriptPath;
    es->serverSide = serverSide;
    es->params = params;
    es->eventName = eventName;
    es->enabled = false;

    if ( spin.getContext()->isServer() != serverSide ) {
        es->enabled = false;
        es->pyModule = "";
        es->pyScript = "";
        _eventScriptList.insert( pair<const std::string, EventScript*>( std::string(label), es ) );
        return true;
    }


    try {
        sprintf( cmd, "mod%llx", utick );
        pyModule = cmd;
        sprintf( cmd, "script%llx", utick );
        pyScript = cmd;

        //sprintf(cmd, "mod%llx = spin.load_module('%s')", utick, sf.c_str());
        sprintf( cmd, "%s = spin.load_module('%s')", pyModule.c_str(), sf.c_str() );
        std::cout << "Python cmd: " << cmd << std::endl;
        exec(cmd, spin._pyNamespace, spin._pyNamespace);

        s = spin._pyNamespace[pyModule.c_str()];
        char* cls = boost::python::extract<char*>(s.attr("__spin_behavior_class__") );

        //sprintf( cmd, "script%llx = mod%llx.Script('%s' %s)", utick, utick, id->s_name, params.c_str() );
        sprintf( cmd, "%s = %s.%s('%s' %s)", pyScript.c_str(), pyModule.c_str(), cls, id->s_name, params.c_str() );
        std::cout << "Python cmd: " << cmd << std::endl;
        exec(cmd, spin._pyNamespace, spin._pyNamespace);

        //sprintf(cmd, "script%llx", utick);
        //s = spin._pyNamespace[cmd];
        s = spin._pyNamespace[pyScript.c_str()];
        p = s.attr("run");

        es->run = p;
        es->enabled = true;
        es->pyModule = pyModule;
        es->pyScript = pyScript;

        _eventScriptList.insert( pair<const std::string, EventScript*>( std::string(label), es ) );

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

}


// *****************************************************************************

bool ReferencedNode::callEventScript( const std::string& eventName,
                                      osgIntrospection::ValueList& args ) {

    if ( _eventScriptList.empty() ) return false;

    spinApp &spin = spinApp::Instance();
    boost::python::list argList;
    bool argListBuilt = false;
    bool eventScriptCalled = false;

    for( EventScriptList::iterator it = _eventScriptList.begin();
         it != _eventScriptList.end(); it++ ) {

        if ( !it->second ) continue;
        if ( it->second->eventName != eventName ) continue;
        if ( it->second->serverSide != spin.getContext()->isServer() ) continue;
        if ( !it->second->enabled ) continue;

        if ( !argListBuilt ) {
            try {
                for ( size_t i = 0; i < args.size(); i++ ) {
                    const std::type_info* argt = &args[i].getType().getStdTypeInfo();

                    if ( *argt == typeid(int) ) {
                        argList.append( osgIntrospection::variant_cast<int>(args[i]) );
                    } else if ( *argt == typeid(float) ) {
                        argList.append( osgIntrospection::variant_cast<float>(args[i]) );
                    } else if ( *argt == typeid(double) ) {
                        argList.append( osgIntrospection::variant_cast<double>(args[i]) );
                    } else if ( *argt == typeid(std::string) ) {
                        argList.append( osgIntrospection::variant_cast<std::string>(args[i]).c_str() );
                    } else if ( *argt == typeid(const char*) ) {
                        argList.append( osgIntrospection::variant_cast<const char*>(args[i]) );
                    } else if ( *argt == typeid(char*) ) {
                        argList.append( osgIntrospection::variant_cast<char*>(args[i]) );
                    } else {
                        std::cout << "callEventScript: unsupported argument type: " << argt->name() << std::endl;
                        return false;
                    }
                }
            } catch (...) {
                std::cout << "callEventScript: something went wrong" << std::endl;
                return false;
            }
        }

        try {
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

}

// *****************************************************************************

bool ReferencedNode::enableEventScript( const char* label, int enable ) {

    EventScriptList::iterator it;
    it = _eventScriptList.find( std::string(label) );

    if ( it != _eventScriptList.end() ) {
        it->second->enabled = (enable != 0);
        return true;
    }

    return false;
}

// *****************************************************************************

bool ReferencedNode::removeEventScript( const char* label ) {

    spinApp &spin = spinApp::Instance();
    char cmd[100];
    EventScriptList::iterator it;
    it = _eventScriptList.find( std::string(label) );

    if ( it != _eventScriptList.end() ) {
        if ( spin.getContext()->isServer() == it->second->serverSide ) {
            sprintf( cmd, "del %s", it->second->pyScript.c_str() );
            if ( !spin.execPython( cmd ) ) return false;
        }
        delete( it->second );
        it->second = NULL;
        _eventScriptList.erase( it );
        return true;
    }

    return false;
}



// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
