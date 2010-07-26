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

#ifndef __spinBaseContext_H
#define __spinBaseContext_H

#include <boost/python.hpp>
#include <osg/Timer>

#include "UserNode.h"
#include "spinUtil.h"


/**
 * \brief A class to facilitate communication with SPIN from any application.
 *
 * This class should not be used directly. Instead, use spinClientContext or
 * spinServerContext.
 *
 * By instantiating this class, we load the OSG nodekit for SPIN -- otherwise
 * known as libSPIN. Furthermore, all of the network handlers are automatically
 * generated in order to receive OSC messages from SPIN and update state.
 *
 * Note that this class can be instantiated in different modes, depending on
 * whether the process is to act as a server or a client.
 *
 */

// Forward declarations
class spinLog;

class spinBaseContext
{
    public:
		spinBaseContext();
		virtual ~spinBaseContext();

		static void sigHandler(int signum);

		/**
		 * All threads need to stop according to the following flag:
		 */
		static bool signalStop;

        enum spinContextMode { SERVER_MODE, CLIENT_MODE };

        bool isServer() { return (bool)(mode==SERVER_MODE); }

		virtual bool start() = 0;

        /**
         * Starts the context thread (passed as *threadFunction from a derived
         * class)
         */
        bool startThread( void *(*threadFunction) (void*) );

        /**
         * Stops the currently running thread
         */
        void stop();

        bool isRunning() { return running; }

        /** Multicast group and port number on which the server listens for messages that alters the scene graph. */
        lo_address lo_rxAddr;
        /** Multicast group and port number to which the server sends the messages that clients sent to it to alter the scene graph. */
        lo_address lo_txAddr;
        /** Multicast group and port number to which the server sends the addresses and port numbers on which it sends and receives. */
        lo_address lo_infoAddr;
        /** Multicast group and port number to which the server sends messages to synchronize stuff for which timing matters. */
        lo_address lo_syncAddr;
        lo_server lo_infoServ_;
        lo_server lo_rxServ_;
        static int connectionCallback(const char *path, const char *types, lo_arg **argv, 
                int argc, void *data, void *user_data);
        static int nodeCallback(const char *path, const char *types, lo_arg **argv, 
                int argc, void *data, void *user_data);
        static int sceneCallback(const char *path, const char *types, lo_arg **argv, 
                int argc, void *data, void *user_data);
        static int logCallback(const char *path, const char *types, lo_arg **argv, 
                int argc, void *data, void *user_data);
        static int debugCallback(const char *path, const char *types, lo_arg **argv, 
                int argc, void *data, void *user_data);

	static void oscParser_error(int num, const char *msg, const char *path);

    protected:
        lo_server lo_tcpRxServer_;
        bool running;
        spinContextMode mode;
        void setLog(spinLog &log);
        virtual void createServers() = 0;
        /**
         * All contexts would probably like to listen to infoPort broadcasts.
         * A client can listen for info about the server, such as the correct
         * ports to send messages. The server can listen to pings from clients
         * to determine if they are still alive and connected.
         */


    private:

        // pthread stuff
        pthread_t pthreadID; // id of child thread
        pthread_attr_t pthreadAttr;

};

#endif
