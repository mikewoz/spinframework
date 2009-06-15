// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================



#ifndef __ASSCENEMANAGER_H
#define __ASSCENEMANAGER_H

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>

#include "asGlobals.h"
#include "asReferenced.h"
#include "asMediaManager.h"
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

		void deleteNode(const char *id);
		void doDelete(asReferenced *n);
		void clear();
		void refresh();

		//asSoundConnection* getConnection(char *from, char *to);

		void updateGraph();
		osg::Matrix getWorldCoords(t_symbol *id);




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
		bool isGraphical;

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

int oscCallback_conn(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
int oscCallback_node(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
void oscParser_error(int num, const char *msg, const char *path);

static int oscCallback_debug(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscCallback_admin(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

/*
// replaced by oscCallback_admin:
static int oscCallback_createNode(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscCallback_deleteNode(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscCallback_sceneDebug(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscCallback_sceneClear(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscCallback_sceneLoad(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscCallback_sceneSave(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
*/

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
	if (sceneManager->txServ) lo_send_from(sceneManager->txAddr, sceneManager->txServ, LO_TT_IMMEDIATE, ("/vess/"+sceneManager->sceneID+"/"+string(pNode->id->s_name)).c_str(), types, ##__VA_ARGS__)


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
