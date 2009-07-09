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
class vessListener
{

	public:

		vessListener();
		~vessListener();

		virtual void start();
		virtual void stop();

		void sendMessage(const char *OSCpath, lo_message msg);

		bool isRunning() { return running; }

		void setID(std::string s) { id = s; }
		void setRxAddr(std::string s) { rxAddr = s; }
		void setRxPort(std::string s) { rxPort = s; }

		std::string id;
		std::string rxAddr, rxPort;
		std::string infoAddr, infoPort;

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

/**
 * \brief The vessMaster class is an extension of vessListener, which broadcasts
 * all received messages for other listeners on the network.
 *
 * Be careful to only have one vessMaster per network, or infinite update loops
 * can occur. Of course, one can get around this by giving each vessMaster a
 * unique id via the setID() method
 */
class vessMaster : public vessListener
{

	public:

		vessMaster();
		~vessMaster();

		void setTxAddr(std::string s) { txAddr = s; }
		void setTxPort(std::string s) { txPort = s; }

		std::string txAddr, txPort;

};


static void *vessMasterThread(void *arg);
static void *vessListenerThread(void *arg);


#endif
