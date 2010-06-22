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

#ifndef __SceneManager_H
#define __SceneManager_H

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>

#include <osgDB/SharedStateManager>
#include <osgIntrospection/Value>

#include "ReferencedNode.h"
#include "ReferencedStateSet.h"

#include "UserNode.h"

#ifdef WITH_SHARED_VIDEO
#include "SharedVideoTexture.h"
#endif

#include "lo/lo.h"
#include "tinyxml.h"

typedef std::vector< osg::ref_ptr<ReferencedNode> > nodeListType;
typedef std::map< std::string, nodeListType > nodeMapType;
typedef std::pair< std::string, nodeListType > nodeMapPair;

typedef std::vector<t_symbol*> ReferencedStateSetList;
typedef std::map< std::string, ReferencedStateSetList > ReferencedStateSetMap;
typedef std::pair< std::string, ReferencedStateSetList > ReferencedStateSetPair;

// forward declarations:
class MediaManager;
class GroupNode;
class UserNode;
class SoundConnection;
class spinLog;

/**
 * \brief The main class that maintains the scene and handles OSC messages.
 *
 * The SceneManager class should only be instantiated once for each process
 * or thread. However, this should rarely be done directly because the
 * spinContext instance (listener or server) have a better interface
 * for including SPIN in an application.
*/
class SceneManager
{

    public:

        SceneManager(std::string id, std::string addr, std::string port);
        ~SceneManager();

        void init();
        void debug();

        lo_server rxServ;

        void setGraphical(bool b) { graphicalMode = b; }
        bool isGraphical() { return (bool) graphicalMode; }

        //void setLogFile(const char *logfile);
        void setLog(spinLog& log);

        void registerStateSet(ReferencedStateSet *s);
        void unregisterStateSet(ReferencedStateSet *s);

        void sendNodeList(std::string type);
        void sendConnectionList();

        ReferencedNode *createNode(std::string id, std::string type);
        ReferencedNode *createNode(const char *id, const char *type);
        ReferencedNode *getNode(std::string id);
        ReferencedNode *getNode(const char *id);
        ReferencedNode *getNode(const char *id, const char *type);
        ReferencedNode *getOrCreateNode(const char *id, const char *type);

        ReferencedStateSet* getStateSet(const char *id);
        ReferencedStateSet* createStateSet(const char *id, const char *type);
        ReferencedStateSet* createStateSet(const char *fname);

        std::vector<SoundConnection*> getConnections();

        /**
         * This method removes a node from the scene, however the actual work is
         * done by the doDelete() method.
         *
         * NOTE: All children of the node will remain, but will be re-attached
         * to the 'world' node instead. If you want to destroy all children as
         * well, then use deleteGraph().
         */
        void deleteNode(const char *id);

        /**
         * deleteGraph() operates similarly to deleteNode(), except that all
         * children are also deleted.
         */
        void deleteGraph(const char *id);

        /**
         * The doDelete method performs all of the necessary steps to remove a
         * node from the scene: The node's detach() method is called, which will
         * actually remove it from the scene graph, and eliminate the OSC
         * callback. The callbackUpdate() function is unregistered. And finally,
         * a message is broadcasted to all clients.
         */
        void doDelete(ReferencedNode *n);

        /**
         * Clears scene elements that are not part of any user's subgraphs
         */
        void clear();

        /**
         * Clears only the users from the scene (and any attached nodes in their
         * subgraphs
         */
        void clearUsers();

        /**
         * Forces a removal of all states from the scene graph. This should only
         * be used to clean up upon exit.
         */
        void clearStates();

        /**
         * The refresh method results in a broadcast of all nodelists so that
         * clients can create any missing nodes. Then, the full node state is
         * broadcasted, for ALL nodes.
         */
        void refresh();


        /**
         * The update method is where any thread-safe changes to the scene graph
         * should go. The method is guaranteed to be called only when there are
         * no traversals being performed.
         */
        void update();




        osg::Matrix getWorldCoords(t_symbol *id);

        void exportScene (const char *nodeID, const char *filename);


        std::string sceneID;

        osg::ref_ptr<osg::Group> rootNode;
        osg::ref_ptr<osg::Group> worldNode;
        osg::ref_ptr<osg::Geode> gridGeode;


        /**
         * The scene manager can operate in graphical mode or non-graphical.
         * > For graphical mode, the full scene graph structure is instantiated,
         *   and can thus be renderered by an OSG rendering process.
         * > Non-graphical mode only maintains the basic features in the scene
         *   graph that are needed for GUIs and information servers. Eg, lights
         *   are not created.
         */
        bool graphicalMode;

        osg::ref_ptr<GroupNode> globalObserver;

        bool activeLights[OSG_NUM_LIGHTS];

        std::string getStateAsXML(std::vector<lo_message> nodeState);
        std::string getNodeAsXML(ReferencedNode *n, bool withUsers);

        std::string getConnectionsAsXML();
        bool saveXML(const char *filename, bool withUsers);
        bool saveUsers(const char *s);

        bool createNodeFromXML(TiXmlElement *XMLnode, const char *parentNode);
        bool createConnectionsFromXML(TiXmlElement *XMLnode);
        bool loadXML(const char *filename);

        std::string resourcesPath;

        MediaManager *mediaManager;

        osg::ref_ptr<osgDB::SharedStateManager> sharedStateManager;

        //pthread_mutex_t pthreadLock;// = PTHREAD_MUTEX_INITIALIZER;


    private:
        static bool nodeSortFunction (osg::ref_ptr<ReferencedNode> n1, osg::ref_ptr<ReferencedNode> n2);
        //std::vector< osg::ref_ptr<ReferencedNode> > nodeList;
        nodeMapType nodeMap; // the nodeList arranged by type
        ReferencedStateSetMap stateMap;
};





int invokeMethod(const osgIntrospection::Value classInstance, const osgIntrospection::Type &classType, std::string method, osgIntrospection::ValueList theArgs);

int SceneManagerCallback_log(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int SceneManagerCallback_debug(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int SceneManagerCallback_node(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int SceneManagerCallback_admin(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

//int SceneManagerCallback_script(const char* symName,  char *types, lo_arg *argv, int argc);


/**
 * The SoundConnection node is an exception that does not extend ReferencedNode.
 * As a result, it cannot use the automated OSC interfacing system, or the WX
 * property generation stuff. TODO: fix this so that osgIntrospection can be
 * used (ie, subclass connections from ReferencedNode or some other an osg class)
 */
int SceneManagerCallback_conn(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


void oscParser_error(int num, const char *msg, const char *path);

#endif
