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

#ifndef VESSTHREADS_H_
#define VESSTHREADS_H_

#include "asUtil.h"
#include "asCameraManager.h"
#include "asSceneManager.h"
#include "asMediaManager.h"



/**
 * \brief A class to facilitate communication with VESS from any application.
 *
 * An instance of this class should be included in any process that needs to
 * maintain scene state.
 *
 * By instantiating this class, we load the OSG nodekit for VESS -- otherwise
 * known as libSPIN. Furthermore, all of the network handlers are automatically
 * generated in order to receive OSC messages from VESS and update state.
 *
 * Note that this class can be instantiated in different modes, depending on
 * whether the process is to act as a server or a client.
 *
 */
class vessThread
{

	public:

		enum vessMode { LISTENER_MODE, SERVER_MODE };

		vessThread(vessMode initMode=LISTENER_MODE);
		~vessThread();

		bool setMode(vessMode m);

		bool start();
		void stop();


		void sendSceneMessage(lo_message msg);
		void sendSceneMessage(const char *types, ...);

		void sendNodeMessage(t_symbol *nodeSym, lo_message msg);
		void sendNodeMessage(t_symbol *nodeSym, const char *types, ...);



		bool isRunning() { return running; }

		void setID(std::string s) { id = s; }
		void setRxAddr(std::string s) { rxAddr = s; }
		void setRxPort(std::string s) { rxPort = s; }
		void setTxAddr(std::string s) { txAddr = s; }
		void setTxPort(std::string s) { txPort = s; }

		vessMode mode;

		std::string id;
		std::string rxAddr, rxPort;
		std::string txAddr, txPort;
		std::string infoAddr, infoPort;

		lo_address lo_txAddr;

		lo_address lo_infoAddr;
		lo_server  lo_infoServ;

		asSceneManager *sceneManager;
		asMediaManager *mediaManager;

	    bool running;

	    /**
	     * We store a funciton pointer in the class, which can be dynamically
	     * swapped depending on vessMode (ie, different thread for server mode
	     * versus listener mode).
	     */
	    void *(*threadFunction) (void*);

	private:

		// pthread stuff
		pthread_t pthreadID; // id of child thread
		pthread_attr_t pthreadAttr;


};



/**
 * \brief A class to facilitate communication with VESS from any application.
 *
 * An instance of this class should be included in any process that needs to
 * maintain scene state or communicate to a VESS server.
 *
 * By instantiating this class, we load the VESS NodeKit library, and create all
 * the proper network handlers to receive OSC messages from VESS and update
 * internal state.
 *
 * Be sure to start the listener with start().
 *
 * The vessMaster class should be used if the process is to act as a server
 * rather than a client.
 *
*/

/*
class vessListener
{

	public:

		vessListener();
		~vessListener();

		virtual void start();
		virtual void stop();

		void nodeMessage(t_symbol *nodeSym, lo_message msg);
		void sceneMessage(lo_message msg);


		bool isRunning() { return running; }

		void setID(std::string s) { id = s; }
		void setRxAddr(std::string s) { rxAddr = s; }
		void setRxPort(std::string s) { rxPort = s; }
		void setTxAddr(std::string s) { txAddr = s; }
		void setTxPort(std::string s) { txPort = s; }

		std::string id;
		std::string rxAddr, rxPort;
		std::string txAddr, txPort;
		std::string infoAddr, infoPort;

		lo_address lo_txAddr;

		lo_address lo_infoAddr;
		lo_server  lo_infoServ;

		asSceneManager *sceneManager;
		asMediaManager *mediaManager;

	    bool running;


	    void *(*threadFunction) (void*);

	private:
		// pthread stuff
		pthread_t pthreadID; // id of child thread
		pthread_attr_t pthreadAttr;


};
*/


/**
 * \brief The vessMaster class is an extension of vessListener, which broadcasts
 * all received messages for other listeners on the network.
 *
 * Be careful to only have one vessMaster per network, or infinite update loops
 * can occur. Of course, one can get around this by giving each vessMaster a
 * unique id via the setID() method
 */

/*
class vessMaster : public vessListener
{

	public:

		vessMaster();
		~vessMaster();
};
*/


/**
 * The vessListenerThread is a simple thread that starts a sceneManager and
 * listens to incoming VESS messages. It does NOT re-transmit those messages,
 * and it does NOT perform an update traversal.
 */
static void *vessListenerThread(void *arg);

/**
 * The vessServerThread is mainly differentiated from a listener thread by the
 * fact taht all received messages are re-transmit upon processing. This allows
 * all clients to keep up-to-date whenever new state information is received by
 * the server.
 *
 * Additionally, the vessServerThread will periodically broadcast a ping on
 * infoport, and will perform an update traversal on the scene graph for any
 * nodes who need periodic (scheduled) processing.
 */
static void *vessServerThread(void *arg);




#endif
