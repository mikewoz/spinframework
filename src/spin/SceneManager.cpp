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
#include "DSPNode.h"
#include "spinBaseContext.h"
#include "spinServerContext.h"
#include "ReferencedNode.h"
#include "SoundConnection.h"
#include "spinUtil.h"
#include "osgUtil.h"
#include "spinApp.h"

#include "ImageTexture.h"
#include "VideoTexture.h"
#include "Shader.h"
#include "ShapeNode.h"
#include "ModelNode.h"
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

#include <cppintrospection/Reflection>
#include <cppintrospection/Type>
#include <cppintrospection/Value>
#include <cppintrospection/variant_cast>
#include <cppintrospection/Exceptions>
#include <cppintrospection/MethodInfo>
#include <cppintrospection/PropertyInfo>
#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Attributes>
#include <cppintrospection/ExtendedTypeInfo>
#include "introspect_helpers.h"

#ifdef WITH_SPATOSC
#include <spatosc/spatosc.h>
#endif

extern pthread_mutex_t sceneMutex;



#ifdef WITH_BULLET
#include <btBulletDynamicsCommon.h>
#include "bulletUtil.h"
#include "CollisionShape.h"

extern ContactAddedCallback gContactAddedCallback;

// NOTE: this callback is only added on the server side!

float lastBtHitDepth = 0.0;

bool btCollisionCallback(btManifoldPoint& cp,
                        const btCollisionObject* colObj0,
                        int partId0,
                        int index0,
                        const btCollisionObject* colObj1,
                        int partId1,
                        int index1)
{
    using namespace spin;
    
    if (!colObj0->isStaticObject() && colObj0->isActive())
    {
        CollisionShape *n0 = (CollisionShape*)(colObj0->getUserPointer());
        CollisionShape *n1 = (CollisionShape*)(colObj1->getUserPointer());
    
        // world hit point:
        osg::Vec3 hitPoint0 = asOsgVec3( cp.getPositionWorldOnA() );
        osg::Vec3 hitPoint1 = asOsgVec3( cp.getPositionWorldOnB() );
        
        // returns a unit vector:
        osg::Vec3 normal = asOsgVec3( cp.m_normalWorldOnB );
    
        // penetration depth
        float depth = cp.getDistance(); 
        

        if (depth != lastBtHitDepth)
        {
            //std::cout << "Hit between " << n0->getID() << " and " << n1->getID() << " at: pos0) " << stringify(hitPoint0) << " pos1)" << stringify(hitPoint1) << ", normal: " << stringify(normal) << ", depth="<< depth << std::endl;
            
            // TODO: collide message is usually ssfff: collide nodeID incidence(x,y,z). The following provides just the surface normal, which is wrong:
            BROADCAST(n0, "ssfff", "collide", n1->getID().c_str(), normal.x(), normal.y(), normal.z());
            
            // We need to move the nodes so that they are not colliding any more.
            // We just move along the normal by depth, and add a little extra for
            // good measure:
            osg::Vec3 offset = (normal * -depth);// + (normal * 0.00001);
            n0->translate(offset.x(), offset.y(), offset.z());
        } //else std::cout << "skipping duplicate hit" << std::endl;
        
        lastBtHitDepth = depth;
    }

    // Returns false, telling Bullet that we did not modify the contact point
    // properties at all. We would return true if we changed friction or
    // something
    return false;
}

#endif


// -----------------------------------------------------------------------------

namespace spin
{

// constructors:

//SceneManager::SceneManager(const std::string &id)
SceneManager::SceneManager(std::string id)
{
    std::cout << "creating sceneManager with id: " << id << std::endl;
    std::cout << std::flush;
    
	resourcesPath = "../Resources";

	this->sceneID = id;

    graphicalMode = false;

    // initialize storage vectors:
    nodeMap.clear();
    stateMap.clear();

    // Set resourcesPath:
#ifdef OSX_COCOA
    resourcesPath = "../Resources";
#else
    std::string currentDir = getenv("PWD");
    //std::string currentDir = get_current_dir_name(); // requires #include <unistd.h>
    if ((currentDir.length() > 8) && (currentDir.substr(currentDir.length() - 9)) == std::string("/src/spin"))
    {
        resourcesPath = "../Resources";
    }
    else
    {
        // FIXME: this path should be replaced by PACKAGE_DATA/PACKAGE_NAME, not hard-coded
        resourcesPath = "/usr/local/share/spinFramework";
    }
#endif

    // get user defined env variable OSG_FILE_PATH
    osgDB::Registry::instance()->initDataFilePathList();

    // add all default resources paths to osg's file path list
    osgDB::FilePathList fpl = osgDB::getDataFilePathList();
    fpl.push_back( resourcesPath );
    fpl.push_back( resourcesPath + "/scripts/");
    fpl.push_back( resourcesPath + "/fonts/");
    fpl.push_back( resourcesPath + "/images/");
    fpl.push_back( resourcesPath + "/models/");
    fpl.push_back( resourcesPath + "/shaders/");
    osgDB::setDataFilePathList( fpl );


    //std::cout << "  SceneManager ID:\t\t" << id << std::endl;
    //std::cout << "  SceneManager receiving on:\t" << addr << ", port: " << port << std::endl;

    // discover all relevant nodeTypes by introspection, and fill the nodeMap
    // with empty vectors:

    try
    {
        {
        const cppintrospection::Type &ReferencedNodeType = introspector::getType("ReferencedNode");
        //nodeTypes.clear();
        const cppintrospection::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
        cppintrospection::TypeMap::const_iterator it;
        for (it = allTypes.begin(); it != allTypes.end(); it++)
        {
            if (((*it).second)->isDefined())
            {
                //std::cout << ((*it).second)->getName() << " isSubclassOf(ReferencedNode)? " << ((*it).second)->isSubclassOf(ReferencedNodeType) << std::endl;
                if (((*it).second)->isSubclassOf(ReferencedNodeType))
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
        const cppintrospection::Type &ReferencedStateSetType = introspector::getType("ReferencedStateSet");
        const cppintrospection::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
        cppintrospection::TypeMap::const_iterator it;
        for (it = allTypes.begin(); it != allTypes.end(); it++)
        {
            if (((*it).second)->isDefined())
            {
                if (((*it).second)->isSubclassOf(ReferencedStateSetType))
                {
                    std::string theType = ((*it).second)->getName();
                    ReferencedStateSetList emptyVector;
                    stateMap.insert(ReferencedStateSetPair(theType, emptyVector));
                }
            }
        }
        }
    }
    catch (const cppintrospection::Exception & ex)
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
    rootNode = new osg::ClearNode();
    rootNode->setName("root");
    //worldNode = new osg::ClearNode();
    //worldNode->setName("world");
    worldNode = new ReferencedNode(this, "world");
    rootNode->addChild(worldNode.get());
    
    worldStateSet_ = gensym("NULL");

    for (int i = 0; i < OSG_NUM_LIGHTS; i++)
    {
        activeLights[i] = false;
    }

    // why do we do this?:
    osg::StateSet* rootStateSet = new osg::StateSet;
    for (int i = 0; i < OSG_NUM_LIGHTS; i++)
    {
        rootStateSet->setMode(GL_LIGHT0 + i, osg::StateAttribute::ON);
        //rootStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
    }
    rootNode->setStateSet(rootStateSet);

    // To prevent same external texture from being loaded multiple times,
    // we use the osgDB::SharedStateManager:

    sharedStateManager = new osgDB::SharedStateManager;
	
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

#ifdef WITH_BULLET

    lastColState = false;

    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;
    
    // TODO: update based on bounds of OSG scene
    btVector3 worldAabbMin(-100, -100, -100);
    btVector3 worldAabbMax(100, 100, 100);
    //const int maxProxies = 32766;
    const int maxProxies = 1000;
    btAxisSweep3 *broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

    dynamicsWorld_ = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);


    // This callback is a global method of checking collisions after every 
    // pyhsics step. We register a global callback using gContactAddedCallback,
    // but the problem is that all objects move actually be moved first, which
    // means that they will move and then need to be moved back (this results in
    // jittery updates). Instead, we use the ContactResultCallback method , only
    // when nodes are updated in CollisionShape. See the following for more info
    // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Callbacks_and_Triggers
    if (spinApp::Instance().getContext()->isServer())
    {
        //gContactAddedCallback = btCollisionCallback;
    }
    
    dynamicsWorld_->setGravity(btVector3(0, 0, -10.0));
        
#endif


    // keep a timer for dynamics/physics calculation:
    dynamicsUpdateRate_; // in seconds
    lastTick_ = osg::Timer::instance()->tick();
}

// destructor
SceneManager::~SceneManager()
{
	// TODO
}

void SceneManager::registerStateSet(ReferencedStateSet *s)
{
    //stateList.push_back(s->id);
    stateMap[s->getClassType()].push_back(s);

    std::string oscPattern = "/SPIN/" + sceneID + "/" + s->getID();
    std::vector<lo_server>::iterator it;
    for (it = spinApp::Instance().getContext()->lo_rxServs_.begin(); it != spinApp::Instance().getContext()->lo_rxServs_.end(); ++it)
    {
    	lo_server_add_method((*it), oscPattern.c_str(), NULL,
            spinBaseContext::nodeCallback, (void*)s->getIDSymbol());
    }
	lo_server_add_method(spinApp::Instance().getContext()->lo_tcpRxServer_,
	                     oscPattern.c_str(), NULL,
	                     spinBaseContext::nodeCallback, (void*)s->getIDSymbol());
    SCENE_MSG_TCP("sss", "registerState", s->getID().c_str(), s->getClassType().c_str());
    sendNodeList("*");
}

void SceneManager::unregisterStateSet(ReferencedStateSet *s)
{
    std::string oscPattern = "/SPIN/" + sceneID + "/" + s->getID().c_str();
    std::vector<lo_server>::iterator it;
    for (it = spinApp::Instance().getContext()->lo_rxServs_.begin(); it != spinApp::Instance().getContext()->lo_rxServs_.end(); ++it)
    {
    	lo_server_del_method((*it), oscPattern.c_str(), NULL);
    }
	lo_server_del_method(spinApp::Instance().getContext()->lo_tcpRxServer_,
	                     oscPattern.c_str(), NULL);
    ReferencedStateSetList::iterator itr;
    itr = std::find( stateMap[s->getClassType()].begin(), stateMap[s->getClassType()].end(), s );
    if ( itr != stateMap[s->getClassType()].end() ) stateMap[s->getClassType()].erase(itr);

    SCENE_MSG_TCP("ss", "unregisterState", s->getID().c_str());

    sendNodeList("*");
}

void SceneManager::sendNodeTypes(lo_address txAddr)
{
    if (! spinApp::Instance().getContext()->isServer())
        return;
    
    if (nodeMap.size())
    {
        std::vector<lo_message> msgs;
        lo_message msg = lo_message_new();;
        lo_message_add_string(msg, "nodeTypes" );
        
        nodeMapType::iterator it;
        for (it = nodeMap.begin(); it != nodeMap.end(); ++it)
        {
            lo_message_add_string(msg, (*it).first.c_str() );
        }
        msgs.push_back(msg);
        
        // we use SceneBundle even though there is only one message because
        // we need to be able to send via TCP
        spinApp::Instance().SceneBundle(msgs, txAddr);
    }
}

void SceneManager::sendStateTypes(lo_address txAddr)
{    
    if (! spinApp::Instance().getContext()->isServer())
        return;

    if (stateMap.size())
    {
        std::vector<lo_message> msgs;
        lo_message msg = lo_message_new();;
        lo_message_add_string(msg, "stateTypes" );
        
        ReferencedStateSetMap::iterator it;
        for (it = stateMap.begin(); it != stateMap.end(); ++it)
        {
            lo_message_add_string(msg, (*it).first.c_str() );
        }
        msgs.push_back(msg);
        
        spinApp::Instance().SceneBundle(msgs, txAddr);
    }
}

void SceneManager::sendNodeList(std::string typeFilter, lo_address txAddr)
{
	if (! spinApp::Instance().getContext()->isServer())
        return;

    std::string OSCpath = "/SPIN/" + sceneID;
    lo_message msg;

    std::vector<lo_message> msgs;

    if ((typeFilter.empty()) or (typeFilter == "*"))
    {
        {
            // for each type, send an OSC message:
            nodeMapType::iterator it;
            nodeListType::iterator iter;
            for (it = nodeMap.begin(); it != nodeMap.end(); ++it)
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
                        lo_message_add_string(msg, (char*) (*iter)->getID().c_str() );
                    }
                }
                else
                {
                    lo_message_add_string(msg, "NULL");
                }
                msgs.push_back(msg);
            }
        }
        {
            // for each type, send an OSC message:
            ReferencedStateSetMap::iterator it;
            ReferencedStateSetList::iterator iter;

            for (it = stateMap.begin(); it != stateMap.end(); ++it)
            {
                //std::cout << "Nodes of type '" << (*it).first << "':";
                //for (i=0; i<(*it).second.size(); i++) std::cout << " " << (*it).second[i]->s_name;
                //std::cout << std::endl;

                msg = lo_message_new();

                lo_message_add_string(msg, "stateList" );
                lo_message_add_string(msg, (*it).first.c_str() );

                if ((*it).second.size())
                {
                    for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                    {
                        lo_message_add_string(msg, (char*) (*iter)->getID().c_str() );
                    }
                }
                else
                {
                    lo_message_add_string(msg, "NULL");
                }
                msgs.push_back(msg);
            }
        }
        // remember to send connections as well
        sendConnectionList(txAddr);
    }
    // just send a list for the desired type:
    else 
    {
        if (typeFilter == "SoundConnection")
            sendConnectionList(txAddr);
        else
        {
            {
                nodeMapType::iterator it;
                nodeListType::iterator iter;
                it = nodeMap.find(typeFilter);
                if (it != nodeMap.end())
                {
                    msg = lo_message_new();

                    lo_message_add_string(msg, "nodeList" );
                    lo_message_add_string(msg, (*it).first.c_str() );
                    if ( (*it).second.size() )
                    {
                        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                        {
                            lo_message_add_string(msg, (char*) (*iter)->getID().c_str() );
                        }
                    }
                    else 
                        lo_message_add_string(msg, "NULL");
                    msgs.push_back(msg);
                }
            }
            {
                ReferencedStateSetMap::iterator it;
                ReferencedStateSetList::iterator iter;
                it = stateMap.find(typeFilter);
                if (it != stateMap.end())
                {
                    msg = lo_message_new();

                    lo_message_add_string(msg, "stateList" );
                    lo_message_add_string(msg, (*it).first.c_str() );
                    if ((*it).second.size())
                    {
                        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
                            lo_message_add_string(msg, (char*) (*iter)->getID().c_str() );
                    }
                    else
                        lo_message_add_string(msg, "NULL");

                    msgs.push_back(msg);
                }
            }
        }
    }

    spinApp::Instance().SceneBundle(msgs, txAddr);
}

void SceneManager::sendConnectionList(lo_address txAddr)
{
    // need to manually send soundConnections, since they are not in the nodeMap
    lo_message msg = lo_message_new();
    lo_message_add_string(msg, "nodeList" );
    lo_message_add_string(msg, "SoundConnection" );

    std::vector<SoundConnection*> connections = getConnections();

    if (connections.size())
    {
        for (std::vector<SoundConnection*>::iterator iter = connections.begin(); iter != connections.end(); iter++)
            lo_message_add_string(msg, (*iter)->getID().c_str() );
    } else 
        lo_message_add_string(msg, "NULL" );

    if (txAddr == 0)
    {
        SCENE_LO_MSG(msg);
    }
    else
    {
        SCENE_LO_MSG_TCP(msg, txAddr);
    }
}

void SceneManager::debugContext()
{
    std::cout << "\n\n--------------------------------------------------------------------------------" << std::endl;
    std::cout << "-- SPIN CONTEXT DEBUG:" << std::endl;
	
    spinApp::Instance().getContext()->debugPrint();
}

void SceneManager::debugNodes()
{
    std::cout << "\n\n--------------------------------------------------------------------------------" << std::endl;
    std::cout << "-- NODE LIST for scene with id '" << sceneID << "':" << std::endl;
    nodeMapType::iterator it;
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        std::cout << "-> " << (*it).first << "s:" << std::endl;

        nodeListType::iterator iter;
        for (iter = (*it).second.begin(); iter != (*it).second.end(); iter++)
        {
            std::cout << "    " << (*iter)->getID() << " (parents=";
            if ((*iter)->getNumParents())
            {
                for (int i=0; i<(*iter)->getNumParents(); i++)
                {
                    std::cout << " " << (*iter)->getParentID(i);
                }
                std::cout << ")" << std::endl;
            }
            else
                std::cout << "NONE)" << std::endl;
        }
    }
}

void SceneManager::debugStateSets()
{
    std::cout << "\n\n--------------------------------------------------------------------------------" << std::endl;
    std::cout << "-- STATE LIST for scene with id '" << sceneID << "': " << std::endl;
    ReferencedStateSetMap::iterator sIt;
    for ( sIt=stateMap.begin(); sIt!=stateMap.end(); ++sIt )
    {
        std::cout << "-> " << (*sIt).first << "s:" << std::endl;

        ReferencedStateSetList::iterator sIter;
        for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
        {
            if ((*sIter).valid())
            {
                //ReferencedStateSet *s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
                std::cout << "    " << (*sIter)->getID() << " (parents=";
                for (unsigned i = 0; i < (*sIter)->getNumParents(); i++)
                {
                    std::cout << " " << (*sIter)->getParent(i)->getName();
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
}

void SceneManager::debugSceneGraph()
{
    std::cout << "\n\n--------------------------------------------------------------------------------" << std::endl;
    std::cout << "-- SCENE GRAPH:" << std::endl;
    DebugVisitor visitor;
    visitor.print(this->worldNode.get());
}

void SceneManager::debug()
{
    debugContext();
    debugNodes();
    debugStateSets();
    debugSceneGraph();
}

// *****************************************************************************

std::vector<std::string> SceneManager::getAllNodeTypes()
{
    std::vector<std::string> result;
    for (nodeMapType::iterator it = nodeMap.begin(); it != nodeMap.end(); ++it)
    {
        result.push_back(it->first);
    }
    return result;
}

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
    if (nodeType == "SoundConnection")
        return NULL;

    // Let's broadcast a createNode message BEFORE we actually do the creation.
    // Thus, if some messages are sent during instantiation, at least clients
    // will already have a placeholder for the node.
    SCENE_MSG_TCP("sss", "createNode", id, type);

    // check if a node with that name already exists:
    osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(nodeID->s_thing);

    if (n.valid())
    {
        if (n->getNodeType() != nodeType)
        {
            std::cout << "ERROR: Tried to create " << type << " with id '" << id << "', but that id already exists as an " << n->getNodeType() << "." << std::endl;
            return NULL;
        } else {
            return n.get();
        }
    }

    cppintrospection::Value sceneManagerPtr = cppintrospection::Value(this);

    /*
       std::cout << "[DEBUG] These are all possible types:" << std::endl;
       const cppintrospection::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
       for (cppintrospection::TypeMap::const_iterator it = allTypes.begin (); it != allTypes.end (); ++it)
       {
		   if ((*it).second->isDefined())
       			std::cout << ((*it).second)->getName() << " isAtomic? " << ((*it).second)->isAtomic() << std::endl;
       }
     */

    /*
       std::cout << "sceneManagerPtr::" << std::endl;
       std::cout << "is value a typed pointer? " << sceneManagerPtr.isTypedPointer() << std::endl;
       std::cout << "is value null? " << sceneManagerPtr.isNullPointer() << std::endl;
       std::cout << "is value empty? " << sceneManagerPtr.isEmpty() << std::endl;

       const cppintrospection::Type &tt = sceneManagerPtr.getType();
       std::cout << "the type is:" << std::endl;
    //introspect_print_type(tt);
    std::cout << "name():" << tt.getStdTypeInfo().name() << std::endl;
    std::cout << "is type defined? " << tt.isDefined() << std::endl;
     */

    try
    {
        std::string fullTypeName = introspector::prependNamespace(nodeType);
        // Let's use cppintrospection to create a node of the proper type:
        const cppintrospection::Type &t = introspector::getType(nodeType);

        //std::cout << "... about to create node of type [" << t.getStdTypeInfo().name() << "]" << std::endl;
        //introspect_print_type(t);

        cppintrospection::ValueList args;
        args.push_back(sceneManagerPtr);
        //args.push_back(this);
        args.push_back((const char*)nodeID->s_name);

        cppintrospection::Value v = t.createInstance(args);

        /*
           std::cout << "is value a typed pointer? " << v.isTypedPointer() << std::endl;
           std::cout << "is value null? " << v.isNullPointer() << std::endl;
           std::cout << "is value empty? " << v.isEmpty() << std::endl;

           const cppintrospection::Type &tt = v.getType();
           std::cout << "the reg type is:" << std::endl;
           introspect_print_type(tt);

           const cppintrospection::Type &ttt = v.getInstanceType();
           std::cout << "the instantiated type is:" << std::endl;
           introspect_print_type(ttt);



           std::cout << "type of ReferencedNode = " << typeid(ReferencedNode).name() << std::endl;
           std::cout << "type of ReferencedNode* = " << typeid(ReferencedNode*).name() << std::endl;
           std::cout << "type of getInstance = " << typeid(cppintrospection::getInstance<ReferencedNode>(v)).name() << std::endl;
           std::cout << "type returned from variant_cast = " << typeid(cppintrospection::variant_cast<ReferencedNode>(v)).name() << std::endl;
           std::cout << "type returned from variant_cast* = " << typeid(cppintrospection::variant_cast<ReferencedNode*>(v)).name() << std::endl;
           std::cout << "type returned from extract_raw_data = " << typeid(cppintrospection::extract_raw_data<ReferencedNode>(v)).name() << std::endl;
           std::cout << "type returned from extract_raw_data (with reinterpret_cast)= " << typeid(reinterpret_cast<ReferencedNode*>(cppintrospection::extract_raw_data<ReferencedNode>(v))).name() << std::endl;
           std::cout << "type returned from extract_raw_data (with dynamic_cast)= " << typeid(dynamic_cast<ReferencedNode*>(cppintrospection::extract_raw_data<ReferencedNode>(v))).name() << std::endl;


        //ReferencedNode test = (cppintrospection::getInstance<ReferencedNode>(v));
        //ReferencedNode test = cppintrospection::variant_cast<ReferencedNode>(v);
        ReferencedNode *test = cppintrospection::variant_cast<ReferencedNode*>(v);
        //ReferencedNode *test = cppintrospection::extract_raw_data<ReferencedNode>(v);
        //ReferencedNode *test = reinterpret_cast<ReferencedNode*>(cppintrospection::extract_raw_data<ReferencedNode>(v));
        //ReferencedNode *test = dynamic_cast<ReferencedNode*>(cppintrospection::extract_raw_data<ReferencedNode>(v));

        //std::cout << "test type = " << typeid(test).name() << std::endl;
        //std::cout << "test id = " << test.getID() << std::endl;
        std::cout << "test id = " << test->getID() << std::endl;


        n = test;
         */
        n = cppintrospection::variant_cast<ReferencedNode*>(v);
    }

    catch (cppintrospection::Exception & ex)
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
        nodeMap[n->getNodeType()].push_back(n);

        // broadcast (only if this is the server):
        SCENE_MSG_TCP("sss", "createNode", id, type);
        sendNodeList(type);
        return n.get();
    }
    else
    {
        std::cout << "ERROR: Could not create " << nodeType << ". Invalid type?" << std::endl;
        return NULL;
    }
}

// returns a pointer to a node given an id:
ReferencedNode* SceneManager::getNode(std::string id)
{
    return getNode(id.c_str());
}
ReferencedNode* SceneManager::getNode(const char *id)
{
/*
    osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(gensym(id)->s_thing);

    if (n.valid()) return n.get();
    else return NULL;
*/
    return dynamic_cast<ReferencedNode*>(gensym(id)->s_thing);

}

// returns a pointer to an node given an id and type:
ReferencedNode* SceneManager::getNode(const char *id, const char *type)
{
/*
    osg::ref_ptr<ReferencedNode> n = dynamic_cast<ReferencedNode*>(gensym(id)->s_thing);

    if (n.valid())
    {
        if (n->nodeType == std::string(type))
        {
            return n.get();
        }
    }

    return NULL;
*/

    ReferencedNode *n = dynamic_cast<ReferencedNode*>(gensym(id)->s_thing);
    
    if (n)
    {
        if (n->getNodeType() == std::string(type))
        {
            return n;
        }
    }
    return NULL;
}

ReferencedNode* SceneManager::getOrCreateNode(const char *id, const char *type)
{
    osg::ref_ptr<ReferencedNode> n = getNode(id, type);
    if (n.valid())
        return n.get();
    else
        return createNode(id, type);
}

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
        if (n->getClassType() != theType)
        {
            std::cout << "ERROR: Tried to create " << type << " with id '" << id << "', but that id already exists as a " << n->getClassType() << "." << std::endl;
            return NULL;
        } else {
            return n.get();
        }
    }

    cppintrospection::Value sceneManagerPtr = cppintrospection::Value(this);

    try {

        // Let's use cppintrospection to create a stateset of the proper type:
        const cppintrospection::Type &t = introspector::getType(type);

        //std::cout << "... about to create node of type [" << t.getStdTypeInfo().name() << "]" << std::endl;
        //introspect_print_type(t);

        cppintrospection::ValueList args;
        args.push_back(sceneManagerPtr);
        args.push_back((const char*)theID->s_name);
        cppintrospection::Value v = t.createInstance(args);

        n = cppintrospection::variant_cast<ReferencedStateSet*>(v);

    }

    catch (cppintrospection::Exception & ex)
    {
        std::cout << "ERROR: Could not create " << type << " with id '" << id <<"'. Perhaps that type is not defined?" << std::endl;
        return NULL;
    }

    if (n.valid())
    {
        std::cout << "created new state: " << id << " of type " << type << std::endl;
        registerStateSet(n.get());
        // broadcast (only if this is the server):
        SCENE_MSG_TCP("sss", "createStateSet", id, type);
        sendNodeList("type");
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
    
    // mikewoz: 2012-04-20: I commented this out, because someone may want
    // to load a texture multiple times ... and this doesn't make sense anymore
    // with shaders. It's not correct to assume that every stateset has the
    // notion of path.
    /*
    ReferencedStateSetMap::iterator sIt;
    ReferencedStateSetList::iterator sIter;
    for ( sIt=stateMap.begin(); sIt!=stateMap.end(); ++sIt )
    {
        for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
        {
            if ((*sIter).valid())
            {
                //ReferencedStateSet *s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
                if (getAbsolutePath((*sIter)->getPath())==getAbsolutePath(fname))
                {
                    std::cout << "already exists: " << fname << std::endl;
                    return (*sIter).get();
                }
            }
        }
    }
    */

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

    // shader
    else if (isShaderPath(fullPath))
    {
        osg::ref_ptr<Shader> shdr = dynamic_cast<Shader*>(createStateSet(newID.c_str(), "Shader"));
        if (shdr.valid())
        {
            shdr->setShader(getRelativePath(fname).c_str());
            return shdr.get();
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

void SceneManager::setWorldStateSet(const char *s)
{
    worldNode->setStateSet(s);

    /*
    if (std::string(s)=="NULL")
    {
        worldNode->setStateSet(new osg::StateSet());
        SCENE_MSG("ss", "setWorldStateSet", "NULL");
    }
    else
    {
        ReferencedStateSet *ss = getStateSet(s);
        if (ss)
        {
            if (ss->getIDSymbol() == worldStateSet_)
                return; // we're already using that stateset
        
            worldStateSet_ = ss->getIDSymbol();
            worldNode->setStateSet(ss);
            SCENE_MSG("ss", "setWorldStateSet", s);
        }
	}
    */
}

std::vector<ReferencedNode*> SceneManager::findNodes(const char *pattern)
{
	std::vector<ReferencedNode*> matches;

	// first, check if the patterns does in fact have a wildcard. If not, we can
	// assume the pattern is a node name:
	std::string patternStr = std::string(pattern);
	if (patternStr.find_first_of("*?") != std::string::npos)
	{
        ReferencedNode* test = getNode(pattern);
		if (test) matches.push_back(test);
	}

	// otherwise, we have to go through the entire list of nodes
	// and match with the id:
	else
	{
		// nodes:
		nodeMapType::iterator nIt;
		nodeListType::iterator nIter;
		for (nIt = nodeMap.begin(); nIt != nodeMap.end(); ++nIt)
		{
		    for (nIter = (*nIt).second.begin(); nIter != (*nIt).second.end(); ++nIter)
		    {
				if (wildcardMatch(pattern, (*nIter)->getID().c_str()))
				{
					matches.push_back(*nIter);
				}
		    }
		}
    }
	return matches;
}


std::vector<ReferencedStateSet*> SceneManager::findStateSets(const char *pattern)
{
	std::vector<ReferencedStateSet*> matches;

	// first, check if the patterns does in fact have a wildcard. If not, we can
	// assume the pattern is a stateset name:
	std::string patternStr = std::string(pattern);
	if (patternStr.find_first_of("*?") != std::string::npos)
	{
        ReferencedStateSet* test = getStateSet(pattern);
		if (test) matches.push_back(test);
	}

	// otherwise, we have to go through the entire list of statesets
	// and match with the id:
	else
	{
		// statesets:
		ReferencedStateSetMap::iterator sIt;
		ReferencedStateSetList::iterator sIter;
		for ( sIt=stateMap.begin(); sIt!=stateMap.end(); ++sIt )
		{
		    for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
		    {
				if (wildcardMatch(pattern, (*sIter)->getID().c_str()))
				{
					matches.push_back(*sIter);		
			    }
			}
		}
	}
	return matches;
}


/*
nodeListType SceneManager::findNodes(const char *pattern)
{
	nodeListType matches;
    nodeMapType::iterator it;
	nodeListType::iterator iter;
	
	patternStr = std::string(pattern);
	if (patternStr.find_first_of("*?") != std::string::npos)
	{
		matches.push_back(getNode(patternStr));	
	}

	else
	{
		for (it = nodeMap.begin(); it != nodeMap.end(); ++it)
		{
		    for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
		    {
				if (wildcardMatch(pattern, (*iter)->getID().c_str())
					matches.push_back(*iter);		
		    }
		}
	}
	
	return matches;
}

ReferencedStateSetList SceneManager::findStateSets(const char *pattern)
{
	ReferencedStateSetList matches;
    ReferencedStateSetMap::iterator it;
    ReferencedStateSetList::iterator iter;

	patternStr = std::string(pattern);
	if (patternStr.find_first_of("*?") != std::string::npos)
	{
		// no wildcards, just return getNode(pattern)
		matches.push_back(getStateSet(patternStr));
	}

	else
	{
	
		for ( it=stateMap.begin(); it!=stateMap.end(); ++it )
		{
		    for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
		    {
				if (wildcardMatch(pattern, (*iter)->getID().c_str())
					matches.push_back(*iter);		
		    }
		}
	}

	return matches;
}
*/

std::vector<SoundConnection*> SceneManager::getConnections()
{
    std::vector<SoundConnection*> allConnections;

    nodeMapType::iterator it;
    nodeListType::iterator iter;
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        std::string nodeType = (*it).first;

        const cppintrospection::Type &t = introspector::getType(nodeType);
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

void SceneManager::deleteNode(const char *id)
{
    // don't use ref_ptr here, otherwise node will stay alive until the end of
    // the function, and we want to ensure the destructor is called in doDelete
    ReferencedNode *n = getNode(id);

    if (n)
    {
        // For the deleteNode method, we keep children nodes and just detach
        // them from the node that will be deleted. If they have other parents,
        // they will remain visible; if not, they will become orphaned from the
        // scene graph, but it is the user's responsibility to clean up.
        std::vector<ReferencedNode*> children = n->getChildren();
        std::vector<ReferencedNode*>::iterator childIter = children.begin();
        for (childIter = children.begin(); childIter != children.end(); ++childIter)
        //while (childIter!=children.end()) // iterator should automatically advance once child has changed parent
        {
            (*childIter)->detachFrom(id);
        }

        doDelete(n);
        SCENE_MSG_TCP("ss", "deleteNode", id);

    }
    else if (ReferencedStateSet *s = getStateSet(id))
    {
		doDelete(s);
        sendNodeList("*");
        SCENE_MSG_TCP("ss", "deleteNode", id);
    }
    else
        std::cout << "ERROR: tried to delete " << id << ", but no node or state by that name exists." << std::endl;

    // if delete was successful and removed all other references to the node,
    // then by this point, the node will be deleted, and it's destructor will
    // have been called.
}

void SceneManager::deleteGraph(const char *id)
{
    // don't use ref_ptr here, otherwise node will stay alive until the end of
    // the function, and we want to ensure the destructor is called in doDelete
    ReferencedNode *n = getNode(id);

    if (n)
    {
        // for the deleteGraph method, we also delete all children:
        std::vector<ReferencedNode*> children = n->getChildren();
        std::vector<ReferencedNode*>::iterator childIter;
        for (childIter = children.begin(); childIter != children.end(); ++childIter)
        //while (n->children_.size())
        {
            std::string childID = (*childIter)->getID();
            doDelete(*childIter);
            SCENE_MSG_TCP("ss", "deleteNode", childID.c_str());
        }

        doDelete(n);
        SCENE_MSG_TCP("ss", "deleteNode", id);
    }
    else
        std::cout << "ERROR: tried to deleteGraph " << id << ", but that node does not exist." << std::endl;

    // if delete was successful and removed all other references to the node,
    // then by this point, the node will be deleted, and it's destructor will
    // have been called.
}

void SceneManager::doDelete(ReferencedNode *nodeToDelete)
{
    // hold on to a referenced pointer, while we remove all others
    osg::ref_ptr<ReferencedNode> n = nodeToDelete;

    // remove the node from the scenegraph:
    n->detachFrom("*");

    // remove from our storage nodeMap:
    nodeMapType::iterator it;
    for (it = nodeMap.begin(); it != nodeMap.end(); ++it)
    {
        std::vector< osg::ref_ptr<ReferencedNode> >::iterator iter;
        iter = std::find( (*it).second.begin(), (*it).second.end(), n );
        if ( iter != (*it).second.end() )
            (*it).second.erase(iter);
        //else std::cout << "ERROR: node " << n->getID() << " was not found on the nodeList in SceneManager::removeNode()" << std::endl;
    }

    // TODO: more efficient way (since we know the nodetype):
    /*
       nodeListType::iterator iter;
       iter = std::find( nodeMap[n->nodeType].second.begin(), nodeMap[n->nodeType].second.end(), n );
       if ( iter != nodeMap[n->nodeType].second.end() ) nodeMap[n->nodeType].second.erase(iter);
     */

    // have to unregister the callback function to remove the last ref_ptr:
    pthread_mutex_lock(&sceneMutex);
    // TODO:2011-04-01:aalex: use 0 not NULL?
    n->setUserData(NULL);
    pthread_mutex_unlock(&sceneMutex);

    // now force the actual delete by nulling this referenced pointer. At that
    // time, the destructor for the node should be called.
    n = NULL;
    
    // IMPORTANT: There is one node that will never be destroyed this way: the
    // userNode in spinApp. This stays there no matter what since there is a
    // ref_ptr that maintains a reference count in spinApp.
    //
    // ^ Becaue of this, we have to be careful to re-attach the user again when
    // userRefresh is called.
}

void SceneManager::doDelete(ReferencedStateSet *s)
{
	// hold on to a referenced pointer, while we remove all others
    osg::ref_ptr<ReferencedStateSet> ss = s;
	// remove from the scene
	s->removeFromScene();
	// clear the stateset
	s->clear();
	// unregister from sceneManager (this removes it from the stateMap storage):
	unregisterStateSet(s);
	// by nulling the ref_ptr in s_thing, we will remove the last reference to
	// the stateset (other than the one in local scope)
	s->getIDSymbol()->s_thing = 0;
	// now force the actual delete by nulling this referenced pointer.
	ss = NULL; // destructor is called
}

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
    deleteGraph(n->getID().c_str());
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
            deleteGraph(n->getID().c_str());
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
    deleteGraph((*iter)->getID().c_str());
    }
    }
    }
    }
     */

    /*
       ClearSceneVisitor visitor;
       worldNode->accept(visitor);
     */
     
     
     

    SCENE_MSG_TCP("s", "clear");
    sendNodeList("*");

#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spinApp::Instance().audioScene->deleteAllNodes();
	}
#endif
    
    std::cout << "Cleared scene." << std::endl;
}

void SceneManager::clearUsers()
{
    while (nodeMap[std::string("UserNode")].size())
    {
        deleteGraph(nodeMap[std::string("UserNode")][0]->getID().c_str());
    }
    SCENE_MSG_TCP("s", "clearUsers");
    sendNodeList("*");
    std::cout << "Cleared all users." << std::endl;
}

void SceneManager::clearStates()
{
    ReferencedStateSetMap::iterator it;
    ReferencedStateSetList::iterator iter;

    for ( it=stateMap.begin(); it!=stateMap.end(); ++it )
    {
        while ((iter=(*it).second.begin()) != (*it).second.end())
        {
			doDelete((*iter).get());
			/*
			//osg::ref_ptr<ReferencedStateSet> s = dynamic_cast<ReferencedStateSet*>((*iter)->s_thing);
			// removeFromScene will remove all of OSG's references, leavind the
			// referenced count at just one (ie, the one in our stateMap)
			if ((*iter).valid()) (*iter)->removeFromScene();

			// as soon as we erase it, OSG will call the stateset destructor
			(*it).second.erase(iter);
			*/
        }
    }
    
    SCENE_MSG_TCP("s", "clearStates");

    // TODO: separate sendNodeList to sendStateList as well
    sendNodeList("*");

    std::cout << "Cleared all states." << std::endl;
}

void SceneManager::refreshAll()
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
            //osg::ref_ptr<ReferencedStateSet> s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
            if ((*sIter).valid())
                (*sIter)->stateDump();
        }
    }

    // must do connections manually:
    std::vector<SoundConnection*> connections = getConnections();
    for (std::vector<SoundConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
    {
        (*iter)->stateDump();
    }
    
#ifdef WITH_SPATOSC
	if (spinApp::Instance().hasAudioRenderer)
	{
	    spinApp::Instance().audioScene->forceRefresh();
	}
#endif


    // Announce that a refresh has been completed
    SCENE_MSG_TCP("s", "refresh");
}

void SceneManager::refreshSubscribers(const std::map<std::string, lo_address> &clients)
{
    for (std::map<std::string, lo_address>::const_iterator client = clients.begin(); 
            client != clients.end(); 
            ++client)
    {
        // TODO: just send to subscribers
        sendNodeList("*", client->second);

        nodeMapType::iterator it;
        for (it = nodeMap.begin(); it != nodeMap.end(); ++it)
        {
            nodeListType::iterator iter;
            for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
                (*iter)->stateDump(client->second);
        }

        ReferencedStateSetMap::iterator sIt;
        for (sIt = stateMap.begin(); sIt != stateMap.end(); ++sIt)
        {
            ReferencedStateSetList::iterator sIter;
            for (sIter = (*sIt).second.begin(); sIter != (*sIt).second.end(); ++sIter)
            {
                //osg::ref_ptr<ReferencedStateSet> s = dynamic_cast<ReferencedStateSet*>((*sIter)->s_thing);
                if ((*sIter).valid())
                    (*sIter)->stateDump(client->second);
            }
        }

        // must do connections manually:
        std::vector<SoundConnection*> connections = getConnections();
        for (std::vector<SoundConnection*>::iterator iter = connections.begin(); iter != connections.end(); ++iter)
            (*iter)->stateDump(client->second);
    }
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
    osg::Timer_t tick = osg::Timer::instance()->tick();
    double dt =osg::Timer::instance()->delta_s(lastTick_,tick);

#ifdef WITH_SHARED_VIDEO
    // it's possible that a SharedVideoTexture is in the sceneManager, but not
    // currently applied on any geometry. In this canse, it will not be be
    // seen in the update traversal of the scene graph, and it's updateCallback
    // will not be called. As a result, frames don't get consumed, and we need
    // to consume all the frames before we are back in sync. If the framerate
    // of the viewer is low, this can take a very long time and appears like a
    // huge delay.
    ReferencedStateSetList::iterator sIter;
    for (sIter = stateMap[std::string("SharedVideoTexture")].begin(); sIter != stateMap[std::string("SharedVideoTexture")].end(); sIter++)
    {
        if (!(*sIter)->getNumParents())
            (*sIter)->updateCallback();
    }
#endif

    
	if (spinApp::Instance().getContext()->isServer())
    {
        spinServerContext *spinserv = dynamic_cast<spinServerContext*>(spinApp::Instance().getContext());
        if (spinserv->shouldAutoClean())
	    {
		    // check if any UserNodes have stopped pinging, and remove them
            // (and their subgraph) if necessary:

		    nodeListType::iterator iter;
	        for (iter = nodeMap[std::string("UserNode")].begin(); iter != nodeMap[std::string("UserNode")].end(); )
	        {
			    if ((*iter)->scheduleForDeletion_)
			    {
				    // this user stopped pinging, so we should remove him
                    // from the subgraph.
				    spinApp::Instance().sceneManager_->deleteGraph((*iter)->getID().c_str());
			    }
                else
                    iter++;
	        }
	    }
    }
    
#ifdef WITH_BULLET
    if (1)//(dt >= dynamicsUpdateRate_)
    {
        // only do on server side?
        if (spinApp::Instance().getContext()->isServer())
        {
            //dynamicsWorld_->performDiscreteCollisionDetection();
            dynamicsWorld_->stepSimulation(dt);
            dynamicsWorld_->updateAabbs(); // <- is this necessary?
        }
    }
#endif

    lastTick_ = tick;
}

// save scene as .osg
void SceneManager::exportScene (const char *nodeID, const char *filename)
{
    std::string fullPath = std::string(filename);
    if (fullPath.substr(fullPath.size()-4) != ".osg")
        fullPath += ".osg";

    // need a TextureVisitor to go over the graph and undo the unref on textures
    osgUtil::Optimizer::TextureVisitor texVisitor(true, false, false, false, false, 1.0);
    // TextureVisitor tv(true,true, // unref image
    //false,false, // client storage
    //false,1.0, // anisotropic filtering
    //this );

    if (strcmp(nodeID, "world") == 0)
    {
        worldNode->accept(texVisitor);
        osgDB::writeNodeFile(*worldNode.get(), fullPath);
        std::cout << "Exported entire scene to: " << fullPath << std::endl;
    }
    else
    {
        osg::ref_ptr<ReferencedNode> subgraph = getNode(nodeID);
        if (subgraph.valid())
        {
            subgraph->accept(texVisitor);
            osgDB::writeNodeFile(*subgraph.get(), fullPath);
            std::cout << "Exported subgraph starting at node '" << subgraph->getID() << "' to: " << fullPath << std::endl;
        }
        else
            std::cout << "Could not find node " << nodeID << ". Export failed." << std::endl;
    }
}

std::string SceneManager::getStateAsXML(std::vector<lo_message> nodeState)
{
    std::ostringstream output("");
    lo_arg **args;
    int i;
    int argc;
    char *argTypes;

    // iterate through all state messages and write as xml output:
    std::vector<lo_message>::iterator nodeStateIterator = nodeState.begin();
    while (nodeStateIterator != nodeState.end())
    {
        argTypes = lo_message_get_types(*nodeStateIterator);
        argc = lo_message_get_argc(*nodeStateIterator);
        args = lo_message_get_argv(*nodeStateIterator);

        output << "<" << (char*)args[0] << " types=" << (argTypes + 1) << ">";

        for (i = 1; i < argc; i++)
        {
            if (i > 1)
                output << " ";
            if (lo_is_numerical_type((lo_type)argTypes[i]))
            {
                output << (float) lo_hires_val( (lo_type)argTypes[i], args[i]);
            }
            else if (strlen((char*) args[i]))
            {
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
    if (! withUsers && (n->getNodeType()=="spin::UserNode"))
    {
        return "";
    }
    std::ostringstream output("");
    // open tag for this node:
    output << "<" << n->getNodeType() << " id=" << n->getID() << ">\n";
    output << getStateAsXML( n->getState() );

    // check for children:
    std::vector<ReferencedNode*> children = n->getChildren();
    if (!children.empty())
    {
        output << "<subgraph>\n";
        
        std::vector<ReferencedNode*>::iterator childIter;
        for (childIter = children.begin(); childIter != children.end(); childIter++)
        {
            output << getNodeAsXML(*childIter, withUsers);
        }
        output << "</subgraph>\n";
    }

    // remember to close tag:
    output << "</" << n->getNodeType() << ">\n";
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
        const cppintrospection::Type &t = introspector::getType(nodeType);
        if (t.isDefined())
        {
            // check if the nodeType is a subclass of DSPNode:
            if (t.getBaseType(0).getName() == introspector::prependNamespace("DSPNode"))
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
                            output << "<SoundConnection id=" << (*connIter)->getID() << ">\n";
                            output << getStateAsXML((*connIter)->getState());
                            // close tag
                            output << "</SoundConnection>";
                        }
                    } //else std::cout << "bad dspnode: " << (*iter)->getID() << std::endl;
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

std::vector<t_symbol*> SceneManager::getSavableStateSets(ReferencedNode *n, bool withUsers)
{
	std::vector<t_symbol*> statesetsToSave;

	// we ignore UserNodes, and their entire subgraphs:
	if (! withUsers && (n->getNodeType() == "UserNode"))
		return statesetsToSave;

	// check for children:
    std::vector<ReferencedNode*> children = n->getChildren();
	std::vector<ReferencedNode*>::iterator childIter;
	for (childIter = children.begin(); childIter != children.end(); ++childIter)
	{
		statesetsToSave = getSavableStateSets(*childIter, withUsers);
	}

	if (n->getNodeType() == "ModelNode")
	{
		ModelNode *mdl = dynamic_cast<ModelNode*>(n);
		if (mdl)
		{
			statesetsToSave.insert( statesetsToSave.begin(), mdl->_statesetList.begin(), mdl->_statesetList.end());
		}
	}    
    else
    {
        GroupNode *grp = dynamic_cast<GroupNode*>(n);
		if (grp)
		{
            t_symbol *ss = grp->getStateSetSymbol();
            if (ss->s_thing)
                statesetsToSave.push_back(ss);
		}
    }
    
	return statesetsToSave;
}

// *****************************************************************************
bool SceneManager::saveXML(const char *s, bool withUsers)
{
    nodeMapType::iterator it;
    nodeListType::iterator iter;
    std::vector<t_symbol*> statesetsToSave;
    std::vector<t_symbol*>::iterator sIt;

    // convert filename into valid path:
    std::string filename = getSpinPath(s);
    // and make sure that there is an .xml extension:
    if (filename.substr(filename.length() - 4) != std::string(".xml"))
        filename+=".xml";

    // start with XML Header:
    std::ostringstream output("");
    // FIXME: why not UTF-8?
    output << "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" ?>\n"
        << "<!DOCTYPE SPIN SYSTEM>\n";

    // Statesets need to be first, so we make a first pass over the scene graph
    // to write all statesets. Note that we must do a full traversal rather than
    // just checking ShapeNodes and ModelNodes, because those nodes may be
    // attached under a user subgraph, and we need to be able to ignore those.
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
        {
            for (int i=0; i < (*iter)->getNumParents(); i++)
            {
                if ((*iter)->getParentID(i) == "world")
                {
                    std::vector<t_symbol*> tmp = getSavableStateSets((*iter).get(), withUsers);
                    statesetsToSave.insert( statesetsToSave.begin(), tmp.begin(), tmp.end());
                }
            }
        }
    }

    // Note: several nodes may use the same stateset, so we need remove
    // duplicates from the stateset list we just collected:
    sIt = std::unique( statesetsToSave.begin(), statesetsToSave.end() );

    // now write the statesets:
    //std::cout << "saving " << statesetsToSave.size() << " statesets" << std::endl;
    output << "<statesets>\n";
    for (sIt = statesetsToSave.begin(); sIt != statesetsToSave.end(); ++sIt)
    {
    	ReferencedStateSet *ss = dynamic_cast<ReferencedStateSet*>((*sIt)->s_thing);
    	if (ss)
    	{
    		output << "<" << ss->getClassType() << " id=" << ss->getID() << ">\n";
			output << getStateAsXML( ss->getState() );
			output << "</" << ss->getClassType() << ">\n";
    	}
    }
    output << "</statesets>\n";

    // now save nodes:
    output << "<spinScene>\n";
    for (it = nodeMap.begin(); it != nodeMap.end(); it++)
    {
        for (iter = (*it).second.begin(); iter != (*it).second.end(); ++iter)
        {
            for (int i=0; i < (*iter)->getNumParents(); i++)
            {
                if ((*iter)->getParentID(i) == "world")
                {
                    output << getNodeAsXML((*iter).get(), withUsers);
                }
            }
        }
    }
    output << "</spinScene>\n";
    output << getConnectionsAsXML();

    // now write to file:
    TiXmlDocument outfile( filename.c_str() );
    outfile.Parse( output.str().c_str() );
    if (outfile.Error())
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

    for (nodeListType::iterator iter = nodeMap["spin::UserNode"].begin(); iter != nodeMap["spin::UserNode"].end(); iter++)
    {
        output << getNodeAsXML((*iter).get(), true);
    }
    output << "</spinScene>\n";

    // now write to file:
    TiXmlDocument outfile(filename.c_str());
    outfile.Parse(output.str().c_str());
    if (outfile.Error())
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
    cppintrospection::ValueList args;
    float f;

    char *nodeType = (char*) XMLnode->Value();

    if (introspector::getType(nodeType).isDefined())
    {
        if (XMLnode->Attribute("id"))
        {
            char *nodeID = (char*) XMLnode->Attribute("id");
            osg::ref_ptr<ReferencedNode> n = getOrCreateNode(nodeID, nodeType);

            // get node as an cppintrospection::Value (note that type will be ReferencedNode pointer):
            const cppintrospection::Value introspectValue = cppintrospection::Value(n.get());

            // the getInstanceType() method however, gives us the real type being pointed at:
            const cppintrospection::Type &introspectType = introspectValue.getInstanceType();

            for ( child1 = XMLnode->FirstChildElement(); child1; child1 = child1->NextSiblingElement() )
            {
                // first check if this XML node is a list of Audiocape children:
                if (child1->Value() == std::string("subgraph"))
                {

                    for (child2 = child1->FirstChildElement(); child2; child2 = child2->NextSiblingElement() )
                    {
                        createNodeFromXML(child2, n->getID().c_str());
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
                    if (! argVector.empty())
                        n->setParent((char*) argVector[0].c_str());
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
                        if (fromString<float>(f, argVector[i]))
                            args.push_back(f);
                    }
                    else
                    {
                        args.push_back((const char*) argVector[i].c_str());
                    }
                }

                // now we can finally call the method:
                try
                {
                    introspectType.invokeMethod(method, introspectValue, args, true); // the true means that it will try base classes as well
                }

                catch (const cppintrospection::Exception & ex)
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


bool SceneManager::createStateSetFromXML(TiXmlElement *XMLnode)
{
    TiXmlElement *child;
    char *types;
    std::string method;
    std::vector<std::string> argVector;
    cppintrospection::ValueList args;
    float f;

    char *classType = (char*) XMLnode->Value();

    if (introspector::getType(std::string(classType)).isDefined())
    {
        if (XMLnode->Attribute("id"))
        {
            char *statesetID = (char*) XMLnode->Attribute("id");
            osg::ref_ptr<ReferencedStateSet> ss = createStateSet(statesetID, classType);

            // get node as an cppintrospection::Value (note that type will be ReferencedNode pointer):
            const cppintrospection::Value introspectValue = cppintrospection::Value(ss.get());

            // the getInstanceType() method however, gives us the real type being pointed at:
            const cppintrospection::Type &introspectType = introspectValue.getInstanceType();

            for ( child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
            {
                // look for child nodes with a 'types' attribute
                if ((types = (char*) child->Attribute("types")))
                {
                    method = child->Value();
                    argVector = tokenize(child->FirstChild()->Value());
                }
                else
                    continue;

                if (argVector.size() != strlen(types))
                {
                    std::cout << "ERROR: could not call '" << method << "' for stateset " << statesetID << ". Type mismatch." << std::endl;
                    continue;
                }

                args.clear();
                for (unsigned i = 0; i < strlen(types); i++)
                {
                    if (lo_is_numerical_type((lo_type)types[i]))
                    {
                        if (fromString<float>(f, argVector[i]))
                            args.push_back(f);
                    }
                    else
                    {
                        args.push_back((const char*) argVector[i].c_str());
                    }
                }

                // now we can finally call the method:
                try
                {
                    introspectType.invokeMethod(method, introspectValue, args, true); // the true means that it will try base classes as well
                }
                catch (const cppintrospection::Exception & ex)
                {
                    std::cerr << "catch exception in loadXML: " << ex.what() << std::endl;
                }
            }
        }
        else
        {
            std::cout << "ERROR: Found XML stateset of type " << classType << " with no id attribute. Could not create." << std::endl;
        }

    }
    else
    {
        std::cout << "ERROR: Found XML stateset of type " << classType << ", but no such type is registered." << std::endl;
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

        if (! n.valid())
            return false;

        // we can call the connect() method again, to ensure the connection is actually made:
        osg::ref_ptr<DSPNode> srcNode = dynamic_cast<DSPNode*>(n.get());
        srcNode->connect(snkName);

        // check if it exists now (it might not, if the sink didn't exist)
        SoundConnection *conn = srcNode->getConnection(snkName);

        if (! conn)
            return false;

        // now we have it, so we can go through the rest of the xml children
        // and update the connection's parameters:

        for (child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
        {
            //std::cout << "child = " << child->Value() << ", value = " << child->FirstChild()->Value() << std::endl;

            // we know that each method takes exactly one float arg:
            float f;
            if (child->Value() == std::string("setThru"))
            {
                if (fromString<float>(f, child->FirstChild()->Value()))
                    conn->setThru(f);
            }
            else if (child->Value() == std::string("setDistanceEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value()))
                    conn->setDistanceEffect(f);
            }
            else if (child->Value() == std::string("setRolloffEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value()))
                    conn->setRolloffEffect(f);
            }
            else if (child->Value() == std::string("setDopplerEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value()))
                    conn->setDopplerEffect(f);
            }
            else if (child->Value() == std::string("setDiffractionEffect"))
            {
                if (fromString<float>(f, child->FirstChild()->Value()))
                    conn->setDiffractionEffect(f);
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
    if (! doc.LoadFile())
    {
        std::cout << "ERROR: failed to load " << filename << ". Invalid XML format." << std::endl;
        return false;
    }

    // Now see if there is a <statesets> tag:
    if ((root = doc.FirstChild("statesets")))
    {
        for (child = root->FirstChildElement(); child; child = child->NextSiblingElement())
        {
            createStateSetFromXML(child);
        }
    }

    // get the <spinScene> tag and verify:
    if (! (root = doc.FirstChild("spinScene")))
    {
        std::cout << "ERROR: failed to load " << filename << ". XML file has no <spinScene> tag." << std::endl;
        return false;
    }

    for (child = root->FirstChildElement(); child; child = child->NextSiblingElement())
    {
        createNodeFromXML(child);
    }

    // Now see if there is a <connections> tag:
    if (root = doc.FirstChild("connections"))
    {
        // go through the file again, making sure that connections get created:
        for (child = root->FirstChildElement(); child; child = child->NextSiblingElement())
        {
            createConnectionsFromXML(child);
        }
    }

    this->update();

    std::cout << "Successfully loaded scene from " << filename << std::endl;
    return true;
}


// *****************************************************************************
void SceneManager::setGravity(float x, float y, float z)
{
#ifdef WITH_BULLET
    dynamicsWorld_->setGravity(btVector3(x, y, z));
#endif
}

void SceneManager::setUpdateRate(float seconds)
{
    dynamicsUpdateRate_ = seconds;
}

#ifdef WITH_BULLET
void detectCollision( bool& lastColState, btCollisionWorld* cw )
{
    unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
    if( ( numManifolds == 0 ) && (lastColState == true ) )
    {
        osg::notify( osg::ALWAYS ) << "No collision." << std::endl;
        lastColState = false;
    }
    else {
        for( unsigned int i = 0; i < numManifolds; i++ )
        {
            btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
            unsigned int numContacts = contactManifold->getNumContacts();
            for( unsigned int j=0; j<numContacts; j++ )
            {
                btManifoldPoint& pt = contactManifold->getContactPoint( j );
                if( ( pt.getDistance() <= 0.f ) && ( lastColState == false ) )
                {
                    // grab these values for the contact normal arrows:
                    osg::Vec3 pos = asOsgVec3( pt.getPositionWorldOnA() ); // position of the collision on object A
                    osg::Vec3 normal = asOsgVec3( pt.m_normalWorldOnB ); // returns a unit vector
                    float pen = pt.getDistance(); //penetration depth

                    osg::Quat q;
                    q.makeRotate( osg::Vec3( 0, 0, 1 ), normal );

                    osg::notify( osg::ALWAYS ) << "Collision detected." << std::endl;

                    osg::notify( osg::ALWAYS ) << "\tPosition: " << stringify(pos) << std::endl;
                    osg::notify( osg::ALWAYS ) << "\tNormal: " << stringify(normal) << std::endl;
                    osg::notify( osg::ALWAYS ) << "\tPenetration depth: " << pen << std::endl;
                    //osg::notify( osg::ALWAYS ) << q.w() <<","<< q.x() <<","<< q.y() <<","<< q.z() << std::endl;
                    lastColState = true;
                }
                else if( ( pt.getDistance() > 0.f ) && ( lastColState == true ) )
                {
                    osg::notify( osg::ALWAYS ) << "No collision." << std::endl;
                    lastColState = false;
                }
            }
        }
    }
}
#endif



// *****************************************************************************
// helper methods:

bool SceneManager::nodeSortFunction (osg::ref_ptr<ReferencedNode> n1, osg::ref_ptr<ReferencedNode> n2)
{
    return (n1->getID() < n2->getID());
}

namespace introspector
{

int invokeMethod(const cppintrospection::Value classInstance, const cppintrospection::Type &classType, std::string method, cppintrospection::ValueList theArgs)
{

    // TODO: we should try to store this globally somewhere, so that we don't do
    // a lookup every time there is a message:

    /*
       const cppintrospection::Type &ReferencedNodeType = cppintrospection::Reflection::getType("ReferencedNode");


       if ((classType==ReferencedNodeType) || (classType.isSubclassOf(ReferencedNodeType)))
       {
     */
    try
    {
        classType.invokeMethod(method, classInstance, theArgs, true);
        // if we get this far, then the method invocation succeeded and
        // we can return:
        return 1;
    }
    catch (cppintrospection::Exception & ex)
    {
        //std::cerr << "catch exception: " << ex.what() << std::endl;
    }

    // If the method wasn't found in the classInstance, then we need to go
    // through all base classes to see if method is contained in a parent class:
    for (int i = 0; i < classType.getNumBaseTypes(); i++)
    {
        if (introspector::invokeMethod(classInstance, classType.getBaseType(i), method, theArgs))
            return 1;
    }
    // }

    return 0;
}

// TODO: Move to introspector.cpp
std::string prependNamespace(const std::string &name)
{
    // FIXME: is this the fastest way to do this?
    return std::string("spin::").append(name);
}

// TODO: Move to introspector.cpp
const cppintrospection::Type& getType(const std::string &name)
{
    return cppintrospection::Reflection::getType(prependNamespace(name));
}

} // end of namespace introspector

} // end of namespace spin

