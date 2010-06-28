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


#include "SceneManager.h"
#include "spinBaseContext.h"
#include "MediaManager.h"
#include "ReferencedNode.h"
#include "SoundConnection.h"
#include "spinUtil.h"
#include "osgUtil.h"
#include "spinApp.h"

#include "ImageTexture.h"
#include "VideoTexture.h"
#include "SharedVideoTexture.h"

#include "nodeVisitors.h"
#include "spinLog.h"

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include <sstream>

#include <osgDB/WriteFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgUtil/Optimizer>

#include <osgIntrospection/Reflection>
#include <osgIntrospection/Type>
#include <osgIntrospection/Value>
#include <osgIntrospection/variant_cast>
#include <osgIntrospection/Exceptions>
#include <osgIntrospection/MethodInfo>
#include <osgIntrospection/PropertyInfo>


#include <osgIntrospection/ReflectionMacros>
#include <osgIntrospection/TypedMethodInfo>
#include <osgIntrospection/StaticMethodInfo>
#include <osgIntrospection/Attributes>

#include <osgIntrospection/ExtendedTypeInfo>

using namespace osgIntrospection;

extern pthread_mutex_t pthreadLock;

// *****************************************************************************
// constructors:

SceneManager::SceneManager(const std::string &id)
{
    this->sceneID = id;

    graphicalMode = false;

    // initialize storage vectors:
    //nodeTypes.clear();
    nodeMap.clear();
    stateMap.clear();
    //nodeList.clear();

    // Set resourcesPath:
    std::string currentDir = getenv("PWD");
    if ((currentDir.length()>8) && (currentDir.substr(currentDir.length()-9))==std::string("/src/spin"))
    {
        resourcesPath = "../Resources";
    } else {
        resourcesPath = "/usr/local/share/spinFramework";
    }
    std::cout << "  Resources path:\t\t" << resourcesPath << std::endl;

    // get user defined env variable OSG_FILE_PATH
    osgDB::Registry::instance()->initDataFilePathList();

    // add all default resources paths to osg's file path list
    osgDB::FilePathList fpl = osgDB::getDataFilePathList();
    fpl.push_back( resourcesPath );
    fpl.push_back( resourcesPath + "/scripts/");
    fpl.push_back( resourcesPath + "/fonts/");
    fpl.push_back( resourcesPath + "/images/");
    osgDB::setDataFilePathList( fpl );

    mediaManager = new MediaManager(resourcesPath);
    
    //std::cout << "  SceneManager ID:\t\t" << id << std::endl;
    //std::cout << "  SceneManager receiving on:\t" << addr << ", port: " << port << std::endl;

    // discover all relevant nodeTypes by introspection, and fill the nodeMap
    // with empty vectors:

    try
    {

        {
        const osgIntrospection::Type &ReferencedNodeType = osgIntrospection::Reflection::getType("ReferencedNode");
        //nodeTypes.clear();
        const osgIntrospection::TypeMap &allTypes = osgIntrospection::Reflection::getTypes();
        osgIntrospection::TypeMap::const_iterator it;
        for ( it=allTypes.begin(); it!=allTypes.end(); it++)
        {
            if (((*it).second)->isDefined())
            {
                //std::cout << ((*it).second)->getName() << " isSubclassOf(ReferencedNode)? " << ((*it).second)->isSubclassOf(ReferencedNodeType) << std::endl;
                if ( ((*it).second)->isSubclassOf(ReferencedNodeType) )
                {
                    std::string theType = ((*it).second)->getName();
                    //nodeTypes.push_back(theType);
                    nodeListType emptyVector;
                    nodeMap.insert(nodeMapPair(theType, emptyVector));
                }
            }
        }
        }

        // Same thing for ReferencedStateSets:
        {
        const osgIntrospection::Type &ReferencedStateSetType = osgIntrospection::Reflection::getType("ReferencedStateSet");
        const osgIntrospection::TypeMap &allTypes = osgIntrospection::Reflection::getTypes();
        osgIntrospection::TypeMap::const_iterator it;
        for ( it=allTypes.begin(); it!=allTypes.end(); it++)
        {
            if (((*it).second)->isDefined())
            {
                if ( ((*it).second)->isSubclassOf(ReferencedStateSetType) )
                {
                    std::string theType = ((*it).second)->getName();
                    ReferencedStateSetList emptyVector;
                    stateMap.insert(ReferencedStateSetPair(theType, emptyVector));
                }
            }
        }
        }
    }
    catch (const osgIntrospection::Exception & ex)
    {
        std::cerr << "SceneManager could not set up:\n" << ex.what() << std::endl;
        std::cout << "These nodes were defined:";
        for (nodeMapType::iterator it = nodeMap.begin(); it != nodeMap.end(); it++)
        {
            std::cout << " " << (*it).first;
        }
        std::cout << std::endl;
        std::cout << "These states were defined:";
        for (ReferencedStateSetMap::iterator sIt=stateMap.begin(); sIt!=stateMap.end(); ++sIt )
        {
            std::cout << " " << (*sIt).first;
        }
        std::cout << std::endl;

        exit (EXIT_FAILURE);
    }

    // need to remove DSPNode???

    /*
    std::cout << "Found the following nodeTypes:" << std::endl;
    vector<string>::const_iterator si;
    for( si=nodeTypes.begin(); si!=nodeTypes.end(); si++)
    {
        cout << *si << endl;
    }
    */


    // create some initial nodeS:
    rootNode = new osg::Group();
    rootNode->setName("root");
    worldNode = new osg::Group();
    worldNode->setName("world");
    rootNode->addChild(worldNode.get());

    for (int i=0; i<OSG_NUM_LIGHTS; i++)
    {
        activeLights[i] = false;
    }

    // why do we do this?:
    osg::StateSet* rootStateSet = new osg::StateSet;
    for (int i=0; i<OSG_NUM_LIGHTS; i++)
    {
        rootStateSet->setMode(GL_LIGHT0 + i, osg::StateAttribute::ON);
        //rootStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
    }
    rootNode->setStateSet(rootStateSet);

    // To prevent same external texture from being loaded multiple times,
    // we use the osgDB::SharedStateManager:

    //sharedStateManager = new osgDB::SharedStateManager;
    sharedStateManager = osgDB::Registry::instance()->getOrCreateSharedStateManager();
    if (sharedStateManager.valid())
    {
        //sharedStateManager->setShareMode(osgDB::SharedStateManager::SHARE_ALL);
        sharedStateManager->setShareMode(osgDB::SharedStateManager::SHARE_TEXTURES);
    } else {
        std::cout << "ERROR: Could not create sharedStateManager" << std::endl;
    }

    // then, when a node is created:
    //      osgDB::Registry::instance()->getSharedStateManager()->share(node);
    // and when a node is deleted:
    //      osgDB::Registry::instance()->getSharedStateManager()->prune();


    //Another way?:
    /*
    osgDB::Options* opt = osgDB::Registry::instance()->getOptions();
    if (opt == NULL) {
        opt = new osgDB::Options();
    }
    opt->setObjectCacheHint(osgDB::Options::CACHE_ALL);
    osgDB::Registry::instance()->setOptions(opt);
    */
}

// *****************************************************************************
// destructor
SceneManager::~SceneManager()
{
    std::cout << "Cleaning up SceneManager..." << std::endl;

    // FIXME: DOING THIS HERE IS REALLY BAD, should happen earlier
#if 0
    // Force a delete (and destructor call) for all nodes still in the scene:
    unsigned i = 0;
    ReferencedNode *n;
    while (i < worldNode->getNumChildren())
    {
        if ((n = dynamic_cast<ReferencedNode*>(worldNode->getChild(i))))
        {
            // delete the graph of any ReferencedNode:
            std::cout << "Delete graph!\n";
            deleteGraph(n->id->s_name);
        }
        else
        {
            // it's possible that there are other nodes attached to worldNode,
            // so just skip them:
            i++;
        }
    }

    // clear any states that are left over:
    std::cout << "clear states!\n";
    clearStates();
#endif

    // stop sceneManager OSC threads:
    usleep(100);
}

void SceneManager::registerStateSet(ReferencedStateSet *s)
{
    //stateList.push_back(s->id);
    stateMap[s->classType].push_back(s->id);

    std::string oscPattern = "/SPIN/" + sceneID + "/" + std::string(s->id->s_name);
    lo_server_add_method(spinApp::Instance().getContext()->lo_rxServ_, oscPattern.c_str(), NULL, 
            spinBaseContext::nodeCallback, (void*)s->id);

    SCENE_MSG("sss", "registerState", s->id->s_name, s->classType.c_str());

    sendNodeList("*");
}

void SceneManager::unregisterStateSet(ReferencedStateSet *s)
{
    std::string oscPattern = "/SPIN/" + sceneID + "/" + std::string(s->id->s_name);
    lo_server_del_method(spinApp::Instance().getContext()->lo_rxServ_, oscPattern.c_str(), NULL);

    ReferencedStateSetList::iterator itr;
    itr = std::find( stateMap[s->classType].begin(), stateMap[s->classType].end(), s->id );
    if ( itr != stateMap[s->classType].end() ) stateMap[s->classType].erase(itr);

    SCENE_MSG("ss", "unregisterState", s->id->s_name);

    sendNodeList("*");
}

// *****************************************************************************


void SceneManager::sendNodeList(std::string typeFilter)
{
    // TODO: typeFilter not used yet
    std::string OSCpath = "/SPIN/" + sceneID;
    lo_message msg;

    std::vector<lo_message> msgs;


    if ((typeFilter.empty()) || (typeFilter=="*"))
    {
        {
            // for each type, send an OSC message:
            nodeMapType::iterator it;
            nodeListType::iterator iter;
            for ( it=nodeMap.begin(); it!=nodeMap.end(); it++ )
            {
                //std::cout << "Nodes of type '" << (*it).first << "':";
                //for (i=0; i<(*it).second.size(); i++) std::cout << " " << (*it).second[i];
                //std::cout << std::endl;

                msg = lo_message_new();

                lo_message_add_string(msg, "nodeList" );
                lo_message_add_string(msg, (*it).first.c_str() );

                if ( (*it).second.size() )
                {
                    for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                    {
                        lo_message_add_string(msg, (char*) (*iter)->id->s_name );
                    }
                } else {
                    lo_message_add_string(msg, "NULL");
                }

                msgs.push_back(msg);
            }
        }

        {
            // for each type, send an OSC message:
            ReferencedStateSetMap::iterator it;
            ReferencedStateSetList::iterator iter;

            for ( it=stateMap.begin(); it!=stateMap.end(); it++ )
            {
                //std::cout << "Nodes of type '" << (*it).first << "':";
                //for (i=0; i<(*it).second.size(); i++) std::cout << " " << (*it).second[i]->s_name;
                //std::cout << std::endl;

                msg = lo_message_new();

                lo_message_add_string(msg, "stateList" );
                lo_message_add_string(msg, (*it).first.c_str() );

                if ( (*it).second.size() )
                {
                    for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                    {
                        lo_message_add_string(msg, (char*) (*iter)->s_name );
                    }
                } else {
                    lo_message_add_string(msg, "NULL");
                }

                msgs.push_back(msg);
            }
        }

        // remember to send connections as well
        sendConnectionList();
    }

    // just send a list for the desired type:
    else {

        if (typeFilter=="SoundConnection")
        {
            sendConnectionList();
        }
        else
        {
            {
                nodeMapType::iterator it;
                nodeListType::iterator iter;
                it = nodeMap.find(typeFilter);
                if ( it != nodeMap.end() )
                {
                    msg = lo_message_new();

                    lo_message_add_string(msg, "nodeList" );
                    lo_message_add_string(msg, (*it).first.c_str() );
                    if ( (*it).second.size() )
                    {
                        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                        {
                            lo_message_add_string(msg, (char*) (*iter)->id->s_name );
                        }
                    } else {
                        lo_message_add_string(msg, "NULL");
                    }

                    msgs.push_back(msg);
                }
            }
            {
                ReferencedStateSetMap::iterator it;
                ReferencedStateSetList::iterator iter;
                it = stateMap.find(typeFilter);
                if ( it != stateMap.end() )
                {
                    msg = lo_message_new();

                    lo_message_add_string(msg, "stateList" );
                    lo_message_add_string(msg, (*it).first.c_str() );
                    if ( (*it).second.size() )
                    {
                        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                        {
                            lo_message_add_string(msg, (char*) (*iter)->s_name );
                        }
                    } else {
                        lo_message_add_string(msg, "NULL");
                    }

                    msgs.push_back(msg);
                }
            }

        }
    }

    spinApp::Instance().SceneBundle(msgs);
}

void SceneManager::sendConnectionList()
{
    // need to manually send soundConnections, since they are not in the nodeMap

    lo_message msg = lo_message_new();
    lo_message_add_string(msg, "nodeList" );
    lo_message_add_string(msg, "SoundConnection" );

    std::vector<SoundConnection*> connections = getConnections();

    if (connections.size())
    {
        for (std::vector<SoundConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
        {
            lo_message_add_string(msg, (*iter)->id->s_name );
        }

    } else {
        lo_message_add_string(msg, "NULL" );
    }

    SCENE_LO_MSG(msg);
}

// *****************************************************************************
void SceneManager::debug()
{

    std::cout << "****************************************" << std::endl;
    std::cout << "************* SCENE DEBUG: *************" << std::endl;

    std::cout << "\nNODE LIST for scene with id '" << sceneID << "':" << std::endl;
    nodeMapType::iterator it;
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        std::cout << "-> " << (*it).first << "s:" << std::endl;

        nodeListType::iterator iter;
        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
        {
            std::cout << "    " << (*iter)->id->s_name << " (parent=" << (*iter)->parent->s_name << ")" << std::endl;
        }
    }

    std::cout << "\n STATE LIST: " << std::endl;
    ReferencedStateSetMap::iterator sIt;
    for ( sIt=stateMap.begin(); sIt!=stateMap.end(); ++sIt )
    {
        std::cout << "-> " << (*sIt).first << "s:" << std::endl;

        ReferencedStateSetList::iterator sIter;
        for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
        {
            if ((*sIter)->s_thing)
            {
                ReferencedStateSet *s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
                std::cout << "    " << (*sIter)->s_name << " (parents=";
                for (unsigned i = 0; i < s->getNumParents(); i++)
                {
                    std::cout << " " << s->getParent(i)->getName();
                }
                std::cout << ")" << std::endl;
            }
        }
    }

    /*
    std::cout << "\n Nodes with textures: " << std::endl;
    StateSetList statesets;
    TextureStateSetFinder f(statesets);
    rootNode->accept(f);
    for (StateSetList::iterator itr=statesets.begin(); itr!=statesets.end(); ++itr)
    {
        osg::StateAttribute *attr = (*itr)->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
        if (attr)
        {
            if (attr->asTexture()->getImage(0))
            {
                // ARGH... can't do this, because image doesn't store filename after load!
                std::string imageFile = attr->asTexture()->getImage(0)->getFileName();
                std::cout << (*itr)->getParent(0)->getName() << ": " << imageFile << std::endl;
            }
        }
    }
    */


    std::cout << "\nSCENE GRAPH:" << std::endl;
    DebugVisitor ev;
    ev.apply(*(this->rootNode.get()));

    // send debug message to all clients:
    SCENE_MSG("s", "debug");

}


// *****************************************************************************

ReferencedNode* SceneManager::createNode(std::string id, std::string type)
{
    const char *charID = id.c_str();
    const char *charType = type.c_str();
    return createNode(charID, charType);
}

ReferencedNode* SceneManager::createNode(const char *id, const char *type)
{
    t_symbol *nodeID = gensym(id);
    std::string nodeType = std::string(type);

    // disallow node with the name "NULL" or bad things might happen
    if (nodeID == gensym("NULL"))
    {
        std::cout << "Oops. Cannot create a node with the reserved name: 'NULL'" << std::endl;
        return NULL;
    }

    // ignore SoundConnection messages (these should be created by
    // the DSPNode::connect() method
    if (nodeType == "SoundConnection") return NULL;

    // Let's broadcast a createNode message BEFORE we actually do the creation.
    // Thus, if some messages are sent during instantiation, at least clients
    // will already have a placeholder for the node.
    SCENE_MSG("sss", "createNode", id, type);


    // check if a node with that name already exists:
    osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(nodeID->s_thing);

    if (n.valid())
    {
        if (n->nodeType != nodeType)
        {
            std::cout << "ERROR: Tried to create " << type << " with id '" << id << "', but that id already exists as an " << n->nodeType << "." << std::endl;
            return NULL;
        } else {
            //SCENE_MSG(this, "sss", "createNode", id, type);
            return n.get();
        }
    }

    osgIntrospection::Value sceneManagerPtr = osgIntrospection::Value(this);

    /*
    std::cout << "[DEBUG] These are all possible types:" << std::endl;
    const osgIntrospection::TypeMap &allTypes = osgIntrospection::Reflection::getTypes();
    for (osgIntrospection::TypeMap::const_iterator it = allTypes.begin (); it != allTypes.end (); ++it)
    {
        std::cout << ((*it).second)->getName() << " isAtomic? " << ((*it).second)->isAtomic() << std::endl;
    }
    */

    /*
    std::cout << "sceneManagerPtr::" << std::endl;
    std::cout << "is value a typed pointer? " << sceneManagerPtr.isTypedPointer() << std::endl;
    std::cout << "is value null? " << sceneManagerPtr.isNullPointer() << std::endl;
    std::cout << "is value empty? " << sceneManagerPtr.isEmpty() << std::endl;

    const osgIntrospection::Type &tt = sceneManagerPtr.getType();
    std::cout << "the type is:" << std::endl;
    //introspect_print_type(tt);
    std::cout << "name():" << tt.getStdTypeInfo().name() << std::endl;
    std::cout << "is type defined? " << tt.isDefined() << std::endl;
    */

    try {
        // Let's use osgIntrospection to create a node of the proper type:
        const osgIntrospection::Type &t = osgIntrospection::Reflection::getType(type);

        //std::cout << "... about to create node of type [" << t.getStdTypeInfo().name() << "]" << std::endl;
        //introspect_print_type(t);

        osgIntrospection::ValueList args;
        args.push_back(sceneManagerPtr);
        //args.push_back(this);
        args.push_back(nodeID->s_name);

        osgIntrospection::Value v = t.createInstance(args);


        /*
        std::cout << "is value a typed pointer? " << v.isTypedPointer() << std::endl;
        std::cout << "is value null? " << v.isNullPointer() << std::endl;
        std::cout << "is value empty? " << v.isEmpty() << std::endl;

        const osgIntrospection::Type &tt = v.getType();
        std::cout << "the reg type is:" << std::endl;
        introspect_print_type(tt);

        const osgIntrospection::Type &ttt = v.getInstanceType();
        std::cout << "the instantiated type is:" << std::endl;
        introspect_print_type(ttt);



        std::cout << "type of ReferencedNode = " << typeid(ReferencedNode).name() << std::endl;
        std::cout << "type of ReferencedNode* = " << typeid(ReferencedNode*).name() << std::endl;
        std::cout << "type of getInstance = " << typeid(osgIntrospection::getInstance<ReferencedNode>(v)).name() << std::endl;
        std::cout << "type returned from variant_cast = " << typeid(osgIntrospection::variant_cast<ReferencedNode>(v)).name() << std::endl;
        std::cout << "type returned from variant_cast* = " << typeid(osgIntrospection::variant_cast<ReferencedNode*>(v)).name() << std::endl;
        std::cout << "type returned from extract_raw_data = " << typeid(osgIntrospection::extract_raw_data<ReferencedNode>(v)).name() << std::endl;
        std::cout << "type returned from extract_raw_data (with reinterpret_cast)= " << typeid(reinterpret_cast<ReferencedNode*>(osgIntrospection::extract_raw_data<ReferencedNode>(v))).name() << std::endl;
        std::cout << "type returned from extract_raw_data (with dynamic_cast)= " << typeid(dynamic_cast<ReferencedNode*>(osgIntrospection::extract_raw_data<ReferencedNode>(v))).name() << std::endl;


        //ReferencedNode test = (osgIntrospection::getInstance<ReferencedNode>(v));
        //ReferencedNode test = osgIntrospection::variant_cast<ReferencedNode>(v);
        ReferencedNode *test = osgIntrospection::variant_cast<ReferencedNode*>(v);
        //ReferencedNode *test = osgIntrospection::extract_raw_data<ReferencedNode>(v);
        //ReferencedNode *test = reinterpret_cast<ReferencedNode*>(osgIntrospection::extract_raw_data<ReferencedNode>(v));
        //ReferencedNode *test = dynamic_cast<ReferencedNode*>(osgIntrospection::extract_raw_data<ReferencedNode>(v));

        //std::cout << "test type = " << typeid(test).name() << std::endl;
        //std::cout << "test id = " << test.id->s_name << std::endl;
        std::cout << "test id = " << test->id->s_name << std::endl;


        n = test;
        */
        n = osgIntrospection::variant_cast<ReferencedNode*>(v);

    }

    catch (osgIntrospection::Exception & ex)
    {

        std::cout << "ERROR: Type " << type << " is not defined." << std::endl;
        return NULL;
    }

    if (n.valid())
    {
        // Must call this for newly created nodes. the function registers the node
        // with an OSC callback, and sets a pointer to this SceneManager
        //n->registerNode(this);


        // NOTE: by pushing the referenced pointer onto the nodeList, we make sure
        // that at least one reference to the node is always maintianed. This allows
        // us to detach it from the scene graph, yet still keep its memory. However,
        // it also means that we have to explicitely remove it.
        //nodeList.push_back(n);
        nodeMap[n->nodeType].push_back(n);

        // broadcast (only if this is the server):
        SCENE_MSG("sss", "createNode", id, type);
        return n.get();
    }
    else
    {
        std::cout << "ERROR: Could not create " << nodeType << ". Invalid type?" << std::endl;
        return NULL;
    }

}

// *****************************************************************************
// returns a pointer to a node given an id:
ReferencedNode* SceneManager::getNode(std::string id)
{
    char *charID = (char*) id.c_str();
    return getNode(charID);
}
ReferencedNode* SceneManager::getNode(const char *id)
{
    osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(gensym(id)->s_thing);

    if (n.valid()) return n.get();
    else return NULL;
}

// *****************************************************************************
// returns a pointer to an node given an id and type:
ReferencedNode* SceneManager::getNode(const char *id, const char *type)
{
    osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(gensym(id)->s_thing);

    if (n.valid())
    {
        if (n->nodeType == std::string(type))
        {
            return n.get();
        }
    }

    return NULL;
}

// *****************************************************************************
ReferencedNode* SceneManager::getOrCreateNode(const char *id, const char *type)
{
    osg::ref_ptr<ReferencedNode> n = getNode(id, type);
    if (n.valid()) return n.get();
    else return createNode(id, type);
}

// *****************************************************************************

ReferencedStateSet* SceneManager::getStateSet(const char *id)
{
    osg::ref_ptr<ReferencedStateSet> s = dynamic_cast<ReferencedStateSet*>(gensym(id)->s_thing);

    if (s.valid()) return s.get();
    else return NULL;
}

ReferencedStateSet* SceneManager::createStateSet(const char *id, const char *type)
{
    t_symbol *theID = gensym(id);
    std::string theType = std::string(type);

    // disallow node with the name "NULL" or bad things might happen
    if (theID == gensym("NULL"))
    {
        std::cout << "Oops. Cannot create a state with the reserved name: 'NULL'" << std::endl;
        return NULL;
    }

    // check if a node with that name already exists:
    osg::ref_ptr<ReferencedStateSet> n = dynamic_cast<ReferencedStateSet*>(theID->s_thing);

    if (n.valid())
    {
        if (n->classType != theType)
        {
            std::cout << "ERROR: Tried to create " << type << " with id '" << id << "', but that id already exists as a " << n->classType << "." << std::endl;
            return NULL;
        } else {
            return n.get();
        }
    }

    osgIntrospection::Value sceneManagerPtr = osgIntrospection::Value(this);

    try {

        // Let's use osgIntrospection to create a node of the proper type:
        const osgIntrospection::Type &t = osgIntrospection::Reflection::getType(type);

        //std::cout << "... about to create node of type [" << t.getStdTypeInfo().name() << "]" << std::endl;
        //introspect_print_type(t);

        osgIntrospection::ValueList args;
        args.push_back(sceneManagerPtr);
        args.push_back((const char*)theID->s_name);
        osgIntrospection::Value v = t.createInstance(args);

        n = osgIntrospection::variant_cast<ReferencedStateSet*>(v);

    }

    catch (osgIntrospection::Exception & ex)
    {
        std::cout << "ERROR: Could not create " << type << " with id '" << id <<"'. Perhaps that type is not defined?" << std::endl;
        return NULL;
    }

    if (n.valid())
    {
        std::cout << "created new state: " << id << " of type " << type << std::endl;
        registerStateSet(n.get());
        // broadcast (only if this is the server):
        SCENE_MSG("sss", "createStateSet", id, type);
        return n.get();
    }
    else
    {
        std::cout << "ERROR: Could not create " << theType << ". Invalid type?" << std::endl;
        return NULL;
    }

}

ReferencedStateSet* SceneManager::createStateSet(const char *fname)
{
    // check if stateset already exists:
    osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(gensym(fname)->s_thing);
    if (ss.valid())
    {
        return ss.get();
    }

    // go through all existing statesets and see if any have fname as the
    // source for the image/video in question

    ReferencedStateSetMap::iterator sIt;
    ReferencedStateSetList::iterator sIter;
    for ( sIt=stateMap.begin(); sIt!=stateMap.end(); ++sIt )
    {
        for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
        {
            if ((*sIter)->s_thing)
            {
                ReferencedStateSet *s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
                if (getAbsolutePath(s->getPath())==getAbsolutePath(fname))
                {
                    std::cout << "already exists: " << fname << std::endl;
                    return s;
                }
            }
        }
    }


    // otherwise, we assume this is a filename, and try to create a stateset of
    // the appropriate type

    std::cout << "Trying to create statestet from filename: " << fname << std::endl;
    std::string fullPath = getAbsolutePath(fname);


    //std::string newID = osgDB::getStrippedName(fullPath); // no path, or ext
    std::string newID = osgDB::getSimpleFileName(fullPath); // no path, keep ext

    size_t pos;
    if ((pos=fullPath.find("shared_video_texture")) != std::string::npos)
    {
        // create the shared memory id from the filename:
        std::string shID = "shvid_"+fullPath.substr(pos+20, fullPath.rfind(".")-(pos+20));
        osg::ref_ptr<SharedVideoTexture> shvid = dynamic_cast<SharedVideoTexture*>(createStateSet(shID.c_str(), "SharedVideoTexture"));
        if (shvid.valid())
        {
            return shvid.get();
        }
    }

    // check if the path is a video, or a directory (potentially containing a
    // sequence of images)
    else if (isVideoPath(fullPath) || osgDB::getDirectoryContents(fullPath).size())
    {
        osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(createStateSet(newID.c_str(), "VideoTexture"));
        if (vid.valid())
        {
            vid->setPath(getRelativePath(fname).c_str());
            return vid.get();
        }
    }

    // otherwise, assume a static image texture:
    else
    {
        osg::ref_ptr<ImageTexture> img = dynamic_cast<ImageTexture*>(createStateSet(newID.c_str(), "ImageTexture"));
        if (img.valid())
        {
            img->setPath(getRelativePath(fname).c_str());
            return img.get();
        }
    }

    std::cout << "ERROR creating ReferencedStateSet from file: " << fname << std::endl;
    return NULL;
}

// *****************************************************************************
std::vector<SoundConnection*> SceneManager::getConnections()
{
    std::vector<SoundConnection*> allConnections;

    nodeMapType::iterator it;
    nodeListType::iterator iter;
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        std::string nodeType = (*it).first;

        const osgIntrospection::Type &t = osgIntrospection::Reflection::getType(nodeType);
        if (t.isDefined())
        {
            // check if the nodeType is a subclass of DSPNode:
            if (t.getBaseType(0).getName() == "DSPNode")
            {
                for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                {
                    osg::ref_ptr<DSPNode> dspNode = dynamic_cast<DSPNode*>((*iter).get());

                    if ((*iter).valid())
                    {
                        std::vector<SoundConnection*>::iterator connIter;
                        for (connIter = dspNode->connectTO.begin(); connIter != dspNode->connectTO.end(); connIter++)
                        {
                            allConnections.push_back((*connIter));
                        }
                    } // bad dsp node
                }
            }
        }
    }
    return allConnections;
}




// *****************************************************************************
void SceneManager::deleteNode(const char *id)
{
    // don't use ref_ptr here, otherwise node will stay alive until the end of
    // the function, and we want to ensure the destructor is called in doDelete
    ReferencedNode *n = getNode(id);


    if (n)
    {
        // for the deleteNode method, all children nodes will remain, so we just
        // change their parent to the "world":
        std::vector<ReferencedNode*>::iterator childIter = n->children.begin();
        while (childIter!=n->children.end()) // iterator should automatically advance once child has changed parent
        {
            (*childIter)->setParent("world");
        }

        doDelete(n);
        SCENE_MSG("ss", "deleteNode", id);

    } else if (ReferencedStateSet *s = getStateSet(id))
    {
        s->removeFromScene();
        sendNodeList("*");
        SCENE_MSG("ss", "deleteNode", id);
    }
    else std::cout << "ERROR: tried to delete " << id << ", but no node or state by that name exists." << std::endl;

    // if delete was successful and removed all other references to the node,
    // then by this point, the node will be deleted, and it's destructor will
    // have been called.
}


// *****************************************************************************
void SceneManager::deleteGraph(const char *id)
{
    // don't use ref_ptr here, otherwise node will stay alive until the end of
    // the function, and we want to ensure the destructor is called in doDelete
    ReferencedNode *n = getNode(id);

    if (n)
    {
        // for the deleteGraph method, we also delete all children:
        while (n->children.size())
        {
            char* childID = (*(n->children.begin()))->id->s_name;
            doDelete(*(n->children.begin()));
            SCENE_MSG("ss", "deleteNode", childID);
        }

        doDelete(n);
        SCENE_MSG("ss", "deleteNode", id);
    }
    else std::cout << "ERROR: tried to deleteGraph " << id << ", but that node does not exist." << std::endl;

    // if delete was successful and removed all other references to the node,
    // then by this point, the node will be deleted, and it's destructor will
    // have been called.
}

// *****************************************************************************
void SceneManager::doDelete(ReferencedNode *nodeToDelete)
{
    // hold on to a referenced pointer, while we remove all others
    osg::ref_ptr<ReferencedNode> n = nodeToDelete;

    // remove the node from the scenegraph:
    n->detach();

    // remove from our storage nodeMap:
    nodeMapType::iterator it;
    for ( it=nodeMap.begin(); it!=nodeMap.end(); ++it)
    {
        std::vector< osg::ref_ptr<ReferencedNode> >::iterator iter;
        iter = std::find( (*it).second.begin(), (*it).second.end(), n );
        if ( iter != (*it).second.end() ) (*it).second.erase(iter);
        //else std::cout << "ERROR: node " << n->id->s_name << " was not found on the nodeList in SceneManager::removeNode()" << std::endl;
    }

    // TODO: more efficient way (since we know the nodetype):
    /*
    nodeListType::iterator iter;
    iter = std::find( nodeMap[n->nodeType].second.begin(), nodeMap[n->nodeType].second.end(), n );
    if ( iter != nodeMap[n->nodeType].second.end() ) nodeMap[n->nodeType].second.erase(iter);
    */

    // have to unregister the callback function to remove the last ref_ptr:
    pthread_mutex_lock(&pthreadLock);
    n->setUserData( NULL );
    pthread_mutex_unlock(&pthreadLock);

    // now force the actual delete by nulling this referenced pointer. At that
    // time, the destructor for the node should be called
    //char *nodeID = n->id->s_name; // but remember the name for the broadcast
    n = NULL;
}


// *****************************************************************************
void SceneManager::clear()
{
    // first find all UserNodes and check move them to the worldNode:
    nodeListType::iterator iter;
    for (iter = nodeMap[std::string("UserNode")].begin(); iter != nodeMap[std::string("UserNode")].end(); iter++)
    {
        (*iter)->setParent("world");
    }


    /*
    // now go through all children of worldNode and do a deleteGraph on any node
    // that is not a UserNode:
    // ERROR: only does odd ones
    for (int i=0; i<worldNode->getNumChildren(); i++)
    {
        ReferencedNode *n = dynamic_cast<ReferencedNode*>(worldNode->getChild(i));
        if (n)
        {
            // as long as this isn't a UserNode, delete
            if (!dynamic_cast<UserNode*>(n))
            {
                deleteGraph(n->id->s_name);
            }
        }
    }
    */

    unsigned i = 0;
    ReferencedNode *n;
    while (i < worldNode->getNumChildren())
    {
        if ((n = dynamic_cast<UserNode*>(worldNode->getChild(i))))
        {
            // skip UserNodes
            i++;
        }
        else if ((n = dynamic_cast<ReferencedNode*>(worldNode->getChild(i))))
        {
            // delete the graph of any ReferencedNode:
            deleteGraph(n->id->s_name);
        }
        else
        {
            // it's possible that there are other nodes attached to worldNode,
            // so just skip them:
            i++;
        }
    }

    /*
    // now remove any remaining nodes attached to world (that are not UserNodes)
    // including their subgraph via deleteGraph()
    // ERROR: only does odd ones
    nodeMapType::iterator it;
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        if ((*it).first != "UserNode")
        {

            nodeListType::iterator iter;
            for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
            {
                if (strcmp((*iter)->getParent(),"world")==0)
                {
                    deleteGraph((*iter)->id->s_name);
                }
            }
        }
    }
    */

    /*
    ClearSceneVisitor visitor;
    worldNode->accept(visitor);
    */


    SCENE_MSG("s", "clear");

    sendNodeList("*");

    std::cout << "Cleared scene." << std::endl;
}

void SceneManager::clearUsers()
{
    while (nodeMap[std::string("UserNode")].size())
    {
        deleteGraph(nodeMap[std::string("UserNode")][0]->id->s_name);
    }

    SCENE_MSG("s", "clearUsers");

    sendNodeList("*");

    std::cout << "Cleared all users." << std::endl;

}

void SceneManager::clearStates()
{
    ReferencedStateSetMap::iterator it;
    ReferencedStateSetList::iterator iter;

    for ( it=stateMap.begin(); it!=stateMap.end(); ++it )
    {
        //for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
        while ((iter=(*it).second.begin()) != (*it).second.end())
        {
            if ((*iter)->s_thing)
            {
                ReferencedStateSet *s = dynamic_cast<ReferencedStateSet*>((*iter)->s_thing);
                s->removeFromScene();
            } else (*it).second.erase(iter);
        }
    }

    for ( it=stateMap.begin(); it!=stateMap.end(); ++it )
    {
        for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
        {
            std::cout << "why is " << (*iter)->s_type << " '" << (*iter)->s_name << "' still here?!" << std::endl;
        }
    }

    // TODO: separate sendNodeList to sendStateList as well
    sendNodeList("*");

    std::cout << "Cleared all states." << std::endl;
}


// *****************************************************************************
void SceneManager::refresh()
{
    sendNodeList("*");

    nodeMapType::iterator it;
    for (it = nodeMap.begin(); it != nodeMap.end(); ++it)
    {
        nodeListType::iterator iter;
        for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
        {
            (*iter)->stateDump();
        }
    }

    ReferencedStateSetMap::iterator sIt;
    for (sIt = stateMap.begin(); sIt != stateMap.end(); ++sIt)
    {
        ReferencedStateSetList::iterator sIter;
        for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
        {
            osg::ref_ptr<ReferencedStateSet> s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
            if (s.valid()) s->stateDump();
        }
    }

    // must do connections manually:
    std::vector<SoundConnection*> connections = getConnections();
    for (std::vector<SoundConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
    {
        (*iter)->stateDump();
    }

    SCENE_MSG("s", "refresh");
}

// *****************************************************************************
// This function returns a pointer to an SoundConnection object
/*
SoundConnection* SceneManager::getConnection(char *from, char *to)
{
    vector<SoundConnection*>::iterator iter;

    osg::ref_ptr<DSPNode> srcNode = dynamic_cast<DSPNode*>( getNode(from) );
    osg::ref_ptr<DSPNode> snkNode = dynamic_cast<DSPNode*>( getNode(to) );

    if ( srcNode.valid() && snkNode.valid() ) {
        for (iter = srcNode->connectTO.begin(); iter != srcNode->connectTO.end(); iter++)
        {
            if ((*iter)->sink->id == snkNode->id) return (*iter);
        }
    }

    // SoundConnection not found:
    return NULL;

}
*/

// *****************************************************************************
// Here is where we change the graph (ie, attach nodes to their new parents,
// detach them, move them to different parts of the scene graph, etc).
//
// IMPORTANT: We do not want to change the graph during a traversal because this
// will invalidate iterators that are on the stack (NodeVisitor uses nested
// calls to do traversal).

void SceneManager::update()
{

#ifdef WITH_SHARED_VIDEO
    //if (shTex.valid()) shTex->updateCallback();
#endif

}

// save scene as .osg
void SceneManager::exportScene (const char *nodeID, const char *filename)
{
    std::string fullPath = std::string(filename);
    if (fullPath.substr(fullPath.size()-4) != ".osg") fullPath += ".osg";

    // need a TextureVisitor to go over the graph and undo the unref on textures
    osgUtil::Optimizer::TextureVisitor texVisitor(true, false, false, false, false, 1.0);
    // TextureVisitor tv(true,true, // unref image
    //false,false, // client storage
    //false,1.0, // anisotropic filtering
    //this );

    //


    if (strcmp(nodeID,"world")==0)
    {
        worldNode->accept(texVisitor);
        osgDB::writeNodeFile(*worldNode.get(), fullPath);
        std::cout << "Exported entire scene to: " << fullPath << std::endl;
    }
    else {
        osg::ref_ptr<ReferencedNode> subgraph = getNode(nodeID);
        if (subgraph.valid())
        {
            subgraph->accept(texVisitor);
            osgDB::writeNodeFile(*subgraph.get(), fullPath);
            std::cout << "Exported subgraph starting at node '" << subgraph->id->s_name << "' to: " << fullPath << std::endl;
        }
        else std::cout << "Could not find node " << nodeID << ". Export failed." << std::endl;
    }

}


std::string SceneManager::getStateAsXML(std::vector<lo_message> nodeState)
{
    std::ostringstream output("");

    lo_arg **args;
    int i, argc;

    char *argTypes;


    // iterate through all state messages and write as xml output:
    std::vector<lo_message>::iterator nodeStateIterator = nodeState.begin();
    while (nodeStateIterator != nodeState.end())
    {

        argTypes = lo_message_get_types(*nodeStateIterator);
        argc = lo_message_get_argc(*nodeStateIterator);
        args = lo_message_get_argv(*nodeStateIterator);

        output << "<" << (char*)args[0] << " types=" << argTypes+1 << ">";

        for (i = 1; i<argc; i++) {
            if (i>1) output << " ";
            if (lo_is_numerical_type((lo_type)argTypes[i]))
            {
                output << (float) lo_hires_val( (lo_type)argTypes[i], args[i] );
            } else if (strlen((char*) args[i])) {
                output << (char*) args[i];
            } else {
                output << "NULL";
            }
        }

        output << "</" << (char*)args[0] << ">";

        lo_message_free(*nodeStateIterator); // do we need to do this? isn't it automatic if we do an erase()?
        nodeState.erase(nodeStateIterator); //note: iterator automatically advances after erase()
    }

    return output.str();
}

std::string SceneManager::getNodeAsXML(ReferencedNode *n, bool withUsers)
{
    // we can ignore UserNodes, and their entire subgraphs:
    if (!withUsers && (n->nodeType=="UserNode"))
    {
        return "";
    }

    std::ostringstream output("");

    // open tag for this node:
    output << "<" << n->nodeType << " id=" << n->id->s_name << ">\n";

    output << getStateAsXML( n->getState() );

    // check for children:
    if (!n->children.empty())
    {
        output << "<subgraph>\n";
        std::vector<ReferencedNode*>::iterator childIter;
        for (childIter = n->children.begin(); childIter != n->children.end(); childIter++)
        {
            output << getNodeAsXML(*childIter, withUsers);
        }
        output << "</subgraph>\n";
    }

    // remember to close tag:
    output << "</" << n->nodeType << ">\n";


    return output.str();
}

std::string SceneManager::getConnectionsAsXML()
{
    std::ostringstream output("");
    output << "<connections>\n";

    // go through all DSPNodes (actually, all nodes that are subclasses of
    // DSPNode), and save the connection info at end of file:

    nodeMapType::iterator it;
    nodeListType::iterator iter;
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        std::string nodeType = (*it).first;

        const osgIntrospection::Type &t = osgIntrospection::Reflection::getType(nodeType);
        if (t.isDefined())
        {
            // check if the nodeType is a subclass of DSPNode:
            if (t.getBaseType(0).getName() == "DSPNode")
            {

                for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                {
                    osg::ref_ptr<DSPNode> dspNode = dynamic_cast<DSPNode*>((*iter).get());

                    if ((*iter).valid())
                    {
                        std::vector<SoundConnection*>::iterator connIter;
                        for (connIter = dspNode->connectTO.begin(); connIter != dspNode->connectTO.end(); connIter++)
                        {
                            // open tag for this node:
                            output << "<SoundConnection id=" << (*connIter)->id->s_name << ">\n";

                            output << getStateAsXML( (*connIter)->getState() );

                            // close tag
                            output << "</SoundConnection>";
                        }
                    } //else std::cout << "bad dspnode: " << (*iter)->id->s_name << std::endl;
                }
            }
        }
    }

    output << "</connections>\n";
    return output.str();
}

/*
std::string SceneManager::getConnectionsAsXML(ReferencedNode *n)
{
    // just give connections for the node in question (and all children

    ostringstream output("");
    output << "<connection>\n";



    output << "</connection>\n";
    return output.str();
}
*/


// *****************************************************************************
bool SceneManager::saveXML(const char *s, bool withUsers)
{
    // convert filename into valid path:
    std::string filename = getSpinPath(s);
    // and make sure that there is an .xml extension:
    if (filename.substr(filename.length() - 4) != std::string(".xml"))
        filename+=".xml";

    // start with XML Header:
    std::ostringstream output("");
    output << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" ?>\n"
            << "<!DOCTYPE SPIN SYSTEM>\n"
            << "<spinScene>\n";


    /*
    vector< osg::ref_ptr<ReferencedNode> >::iterator iter;
    for (iter = nodeList.begin(); iter != nodeList.end() ; iter++)
    {
        // Add the state for this node only if it is a top-level node. Children
        // will be added recursively:
        if ((*iter)->parent == WORLD_SYMBOL)
            output << getNodeStateAsXML( (*iter).get() );
    }
    */

    nodeMapType::iterator it;
    nodeListType::iterator iter;

    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
        {
            if ((*iter)->parent == WORLD_SYMBOL)
            {
                output << getNodeAsXML((*iter).get(), withUsers);
            }
        }
    }


    output << "</spinScene>\n";


    output << getConnectionsAsXML();


    // now write to file:
    TiXmlDocument outfile( filename.c_str() );
    outfile.Parse( output.str().c_str() );
    if ( outfile.Error() )
    {
        // error!
        std::cout << "ERROR: failed to save " << filename << std::endl;
        return false;
    } else {
        // success!
        outfile.SaveFile();
        std::cout << "Saved scene to: " << filename << std::endl;
        return true;
    }

}

bool SceneManager::saveUsers(const char *s)
{
    // convert filename into valid path:
    std::string filename = getSpinPath(s);
    // and make sure that there is an .xml extension:
    if (filename.substr(filename.length() - 4) != std::string(".xml"))
        filename += ".xml";

    // start with XML Header:
    std::ostringstream output("");
    output << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" ?>\n"
            << "<!DOCTYPE SPIN SYSTEM>\n"
            << "<spinScene>\n";

    for (nodeListType::iterator iter = nodeMap["UserNode"].begin(); iter != nodeMap["UserNode"].end(); iter++)
    {
        output << getNodeAsXML((*iter).get(), true);
    }

    output << "</spinScene>\n";

    // now write to file:
    TiXmlDocument outfile( filename.c_str() );
    outfile.Parse( output.str().c_str() );
    if ( outfile.Error() )
    {
        // error!
        std::cout << "ERROR: failed to save " << filename << std::endl;
        return false;
    } else {
        // success!
        outfile.SaveFile();
        std::cout << "Saved users to: " << filename << std::endl;
        return true;
    }

}

bool SceneManager::createNodeFromXML(TiXmlElement *XMLnode, const char *parentNode= "")
{
    TiXmlElement *child1, *child2;
    char *types;
    std::string method;
    std::vector<std::string> argVector;
    osgIntrospection::ValueList args;
    float f;

    char *nodeType = (char*) XMLnode->Value();

    if (osgIntrospection::Reflection::getType(nodeType).isDefined())
    {
        if (XMLnode->Attribute("id"))
        {
            char *nodeID = (char*) XMLnode->Attribute("id");
            osg::ref_ptr<ReferencedNode> n = getOrCreateNode(nodeID, nodeType);

            // get node as an osgInrospection::Value (note that type will be ReferencedNode pointer):
            const osgIntrospection::Value introspectValue = osgIntrospection::Value(n.get());

            // the getInstanceType() method however, gives us the real type being pointed at:
            const osgIntrospection::Type &introspectType = introspectValue.getInstanceType();




            for ( child1 = XMLnode->FirstChildElement(); child1; child1 = child1->NextSiblingElement() )
            {
                // first check if this XML node is a list of Audiocape children:
                if (child1->Value() == std::string("subgraph"))
                {

                    for (child2 = child1->FirstChildElement(); child2; child2 = child2->NextSiblingElement() )
                    {
                        createNodeFromXML(child2, n->id->s_name);
                    }
                    continue;
                }

                // look for child nodes with a 'types' attribute
                if ((types = (char*) child1->Attribute("types")))
                {
                    method = child1->Value();
                    argVector = tokenize(child1->FirstChild()->Value());
                }
                else
                    continue;

                // special case if method is setParent():
                if (method == "setParent")
                {
                    if (!argVector.empty()) n->setParent((char*) argVector[0].c_str());
                    continue;
                }

                if (argVector.size() != strlen(types))
                {
                    std::cout << "ERROR: could not call '" << method << "' for node " << nodeID << ". Type mismatch." << std::endl;
                    continue;
                }

                args.clear();
                for (unsigned i=0; i<strlen(types); i++)
                {
                    if (lo_is_numerical_type((lo_type)types[i]))
                    {
                        if (fromString<float>(f, argVector[i])) args.push_back(f);
                    } else {
                        args.push_back( (const char*) argVector[i].c_str() );
                    }
                }

                // now we can finally call the method:
                try {
                    introspectType.invokeMethod(method, introspectValue, args, true); // the true means that it will try base classes as well
                }

                catch (const osgIntrospection::Exception & ex)
                {
                    std::cerr << "catch exception in loadXML: " << ex.what() << std::endl;
                }
            }

            if (parentNode)
                n->setParent(parentNode);
        } else {
            std::cout << "ERROR: Found XML node of type " << nodeType << " with no id attribute. Could not create." << std::endl;
        }

    } else {
        std::cout << "ERROR: Found XML node of type " << nodeType << ", but no such type is registered." << std::endl;
    }

    return true;
}

bool SceneManager::createConnectionsFromXML(TiXmlElement *XMLnode)
{

    // NOTE: This assumes all nodes have already been created, but will create
    //       connections if they do not exist.

    TiXmlElement *child;

    if (XMLnode->Value() == std::string("SoundConnection"))
    {

        //std::cout << "found <SoundConnection>: " << XMLnode->Attribute("id") << std::endl;
        //std::cout << "src: " << XMLnode->FirstChild("src")->FirstChild()->Value() << std::endl;
        //std::cout << "snk: " << XMLnode->FirstChild("snk")->FirstChild()->Value() << std::endl;

        osg::ref_ptr<ReferencedNode> n = getNode(XMLnode->FirstChild("src")->FirstChild()->Value());
        char *snkName = (char*)XMLnode->FirstChild("snk")->FirstChild()->Value();


        if (!n.valid()) return false;

        // we can call the connect() method again, to ensure the connection is actually made:
        osg::ref_ptr<DSPNode> srcNode = dynamic_cast<DSPNode*>(n.get());
        srcNode->connect(snkName);

        // check if it exists now (it might not, if the sink didn't exist)
        SoundConnection *conn = srcNode->getConnection(snkName);

        if (!conn) return false;

        // now we have it, so we can go through the rest of the xml children
        // and update the connection's parameters:

        for (child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
        {
            //std::cout << "child = " << child->Value() << ", value = " << child->FirstChild()->Value() << std::endl;

            // we know that each method takes exactly one float arg:
            float f;
            if (child->Value() == std::string("setThru"))
            {
                if (fromString<float>(f, child->FirstChild()->Value())) conn->setThru(f);
            }
            else if (child->Value() == std::string("setDistanceEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value())) conn->setDistanceEffect(f);
            }
            else if (child->Value() == std::string("setRolloffEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value())) conn->setRolloffEffect(f);
            }
            else if (child->Value() == std::string("setDopplerEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value())) conn->setDopplerEffect(f);
            }
            else if (child->Value() == std::string("setDiffractionEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value())) conn->setDiffractionEffect(f);
            }

        }
    }

    return false;
}

/*
bool SceneManager::createConnectionsFromXML(TiXmlElement *XMLnode)
{

    // NOTE: this assumes all nodes have already been created

    TiXmlElement *child1, *child2;

    for ( child1 = XMLnode->FirstChildElement(); child1; child1 = child1->NextSiblingElement() )
    {
        // first check children:
        if (child1->Value() == string("subgraph"))
        {
            for (child2 = child1->FirstChildElement(); child2; child2 = child2->NextSiblingElement() )
            {
                createConnectionsFromXML(child2);
            }
        }

        // now if this node has a "connect" message,
        else if (child1->Value() == string("connect"))
        {
            // don't care about nodeType (should be SoundNode or SoundSpace)
            //char *nodeType = (char*) XMLnode->Value();

            if (XMLnode->Attribute("id"))
            {
                osg::ref_ptr<ReferencedNode> n = getNode((char*) XMLnode->Attribute("id"));
                if (n.valid())
                {
                    osg::ref_ptr<DSPNode> srcNode = dynamic_cast<DSPNode*>(n.get());
                    if (srcNode.valid()) srcNode->connect((char*)child1->FirstChild()->Value());
                }
            }
        }

    } // for ( child1
}
*/


// *****************************************************************************
bool SceneManager::loadXML(const char *s)
{
    // convert filename into valid path:
    std::string filename = getSpinPath(s);
    // and make sure that there is an .xml extension:
    if (filename.substr(filename.length() - 4) != std::string(".xml"))
        filename+=".xml";


    std::cout << "Loading scene: " << filename << std::endl;

    TiXmlDocument doc( filename.c_str() );

    TiXmlNode *root = 0;
    TiXmlElement *child = 0;

    // Load the XML file and verify:
    if ( !doc.LoadFile() ) {
        std::cout << "ERROR: failed to load " << filename << ". Invalid XML format." << std::endl;
        return false;
    }

    // get the <spinScene> tag and verify:
    if (!(root = doc.FirstChild( "spinScene" )))
    {
        std::cout << "ERROR: failed to load " << filename << ". XML file has no <spinScene> tag." << std::endl;
        return false;
    }

    // okay.. we have a valid xml file.

    for( child = root->FirstChildElement(); child; child = child->NextSiblingElement() )
    {
        createNodeFromXML(child);
    }

    // Now see if there is a <connections> tag:
    if ((root = doc.FirstChild( "connections" )))
    {
        // go through the file again, making sure that connections get created:
        for( child = root->FirstChildElement(); child; child = child->NextSiblingElement() )
        {
            createConnectionsFromXML(child);
        }
    }


    this->update();

    std::cout << "Successfully loaded scene from " << filename << std::endl;
    return true;

}



// *****************************************************************************
// *****************************************************************************
// helper methods:





bool SceneManager::nodeSortFunction (osg::ref_ptr<ReferencedNode> n1, osg::ref_ptr<ReferencedNode> n2)
{
    return ( std::string(n1->id->s_name) < std::string(n2->id->s_name) );
}


// *****************************************************************************
// OSC callback functions below (need to be valid C function pointers, so they
// are declared here as static functions):




/**
 * Recursive function to invoke a method for a particular class, that will try
 * all base classes as well
 */
int invokeMethod(const osgIntrospection::Value classInstance, const osgIntrospection::Type &classType, std::string method, ValueList theArgs)
{

    // TODO: we should try to store this globally somewhere, so that we don't do
    // a lookup every time there is a message:

    /*
    const osgIntrospection::Type &ReferencedNodeType = osgIntrospection::Reflection::getType("ReferencedNode");


    if ((classType==ReferencedNodeType) || (classType.isSubclassOf(ReferencedNodeType)))
    {
    */
        try {
            classType.invokeMethod(method, classInstance, theArgs, true);
            // if we get this far, then the method invocation succeeded and
            // we can return:
            return 1;
        }
        catch (osgIntrospection::Exception & ex)
        {
            //std::cerr << "catch exception: " << ex.what() << std::endl;
        }

        // If the method wasn't found in the classInstance, then we need to go
        // through all base classes to see if method is contained in a parent class:
        for (int i=0; i<classType.getNumBaseTypes(); i++)
        {
            if (invokeMethod(classInstance, classType.getBaseType(i), method, theArgs)) return 1;
        }
   // }

    return 0;
}

