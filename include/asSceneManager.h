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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#ifndef __ASSCENEMANAGER_H
#define __ASSCENEMANAGER_H

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>

#include <osgIntrospection/Value>

#include "asGlobals.h"
#include "asReferenced.h"
#include "asMediaManager.h"
#include "userNode.h"
#include "vessLog.h"

#include "lo/lo.h"
#include "tinyxml.h"




typedef std::vector< osg::ref_ptr<asReferenced> > nodeListType;
typedef std::map< std::string, nodeListType > nodeMapType;
typedef std::pair< std::string, nodeListType > nodeMapPair;



class asBasicNode;
class asSoundConnection;

/**
 * \brief The main class that maintains the scene and handles OSC messages.
 * 
 * The asSceneManager class should only be instantiated once for each process
 * or thread. However, this should rarely be done directly because the
 * vessThreads classes (vessListener and vessMaster) have a better interface
 * for including vess in an application.
*/
class asSceneManager
{

	public:

		asSceneManager(std::string id, std::string addr, std::string port);
		~asSceneManager();

		void init();
		void debug();

		lo_address rxAddr;
		lo_server_thread rxServ;

		lo_address txAddr;
		lo_server  txServ;
		//lo_server_thread  txServ;

		bool isSlave() { return (bool) !txServ; }
		
		void setGraphical(bool b) { graphicalMode = b; }
		bool isGraphical() { return (bool) graphicalMode; }
		
		//void setLogFile(const char *logfile);
		void setLog(vessLog& log);

		void setTXaddress (std::string addr, std::string port);
		void sendSceneMessage(const char *types, ...);
		void sendNodeList(std::string type);
		void sendNodeMessage(t_symbol *nodeSym, lo_message msg);
		void sendNodeMessage(t_symbol *nodeSym, const char *types, ...);

        asReferenced *createNode(std::string id, std::string type);
		asReferenced *createNode(const char *id, const char *type);
		asReferenced *getNode(std::string id);
		asReferenced *getNode(const char *id);
		asReferenced *getNode(const char *id, const char *type);
		asReferenced *getOrCreateNode(const char *id, const char *type);

		/**
		 * This method removes a node from the scene, however the actual work is
		 * done by the doDelete() method.
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
		void doDelete(asReferenced *n);
		void clear();
		void clearUsers();
		void refresh();

		//asSoundConnection* getConnection(char *from, char *to);

		void updateGraph();
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

		osg::ref_ptr<asBasicNode> globalObserver;

		bool activeLights[OSG_NUM_LIGHTS];


		/**
		 * We keep a list of all nodeTypes so that we can fill GUIs
		 */
		std::vector<std::string> nodeTypes;

		std::string getStateAsXML(std::vector<lo_message> nodeState);
		std::string getNodeAsXML(asReferenced *n);
		std::string getConnectionsAsXML();
		bool saveXML(const char *filename);

		bool createNodeFromXML(TiXmlElement *XMLnode, const char *parentNode);
		bool createConnectionsFromXML(TiXmlElement *XMLnode);
		bool loadXML(const char *filename);

		void setGrid(int gridSize);

		asMediaManager *mediaManager;

		//pthread_mutex_t pthreadLock;// = PTHREAD_MUTEX_INITIALIZER;


	private:
		std::vector< osg::ref_ptr<asReferenced> > nodeList;
		nodeMapType nodeMap; // the nodeList arranged by type

};



static bool nodeSortFunction (osg::ref_ptr<asReferenced> n1, osg::ref_ptr<asReferenced> n2);


static int invokeMethod(const osgIntrospection::Value classInstance, const osgIntrospection::Type &classType, std::string method, osgIntrospection::ValueList theArgs);

int asSceneManagerCallback_log(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int asSceneManagerCallback_debug(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int asSceneManagerCallback_node(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int asSceneManagerCallback_admin(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

/**
 * The asSoundConnection node is an exception that does not extend asReferenced.
 * As a result, it cannot use the automated OSC interfacing system, or the WX
 * property generation stuff. TODO: fix this so that osgIntrospection can be
 * used (ie, subclass connections from asReferenced or some other an osg class)
 */
int asSceneManagerCallback_conn(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


void oscParser_error(int num, const char *msg, const char *path);


/*
#define BROADCAST(pNode, types, ...) \
		fprintf (stderr, "broadcasting message: %s %s %.2f %.2f %.2f\n", ("/node/"+string(pNode->id->s_name)).c_str(), ##__VA_ARGS__); \
       	lo_send(sceneManager->txAddr, ("/node/"+string(pNode->id->s_name)).c_str(), types, ##__VA_ARGS__)
*/

/*
#define BROADCAST(pNode, types, ...) \
        lo_send(sceneManager->txAddr, ("/node/"+string(pNode->id->s_name)).c_str(), types, ##__VA_ARGS__)
*/


#define BROADCAST(pNode, types, ...) \
	if (sceneManager->txServ) lo_send_from(sceneManager->txAddr, sceneManager->txServ, LO_TT_IMMEDIATE, ("/vess/"+sceneManager->sceneID+"/"+std::string(pNode->id->s_name)).c_str(), types, ##__VA_ARGS__)


class asSceneClearVisitor : public osg::NodeVisitor
{
	public:

		//asSceneClearVisitor() : osg::NodeVisitor(TRAVERSE_NONE) {}
		asSceneClearVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}
		 

		virtual void apply(osg::Node &node)
		{
			traverse(node);
		}
		virtual void apply(osg::Group &node)
		{
			asReferenced *n;
			
			if (dynamic_cast<userNode*>(&node))
			{
				// if this is a userNode, do nothing
			}
			else if (n=dynamic_cast<asReferenced*>(&node))
			{
				// If it's any other asReferenced node, delete it, and it's
				// subgraph. This assumes of course, that there are no userNodes
				// in the subgraph.
				n->sceneManager->deleteGraph(n->id->s_name);
			}
			else
			{
				// if this is some other osg::Group, then traverse it:
				traverse(node);
			}
		}
};



class asSceneUpdateVisitor : public osg::NodeVisitor
{
	public:

		asSceneUpdateVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}

		virtual void apply(osg::Node &node)
		{
			traverse(node);
		}
		virtual void apply(osg::Group &node)
		{
			asReferenced *n;
			if (n=dynamic_cast<asReferenced*>(&node)) {
				n->callbackUpdate();
			}
			traverse(node);
		}
};




#endif
