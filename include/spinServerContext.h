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

#ifndef __spinServerContext_H
#define __spinServerContext_H

#include "spinBaseContext.h"


/**
 * \brief A server-side spinContext for maintaining an instance of SPIN and
 * updating all clients
 *
 *
 */
class spinServerContext : public spinBaseContext
{

    public:

		spinServerContext();
		~spinServerContext();

		bool start();


        /**
         * Starts the thread that sends synchronization timecode (syncThread)
         */
        void startSyncThread();


    protected:


    private:


        /**
         * The spinServerThread is mainly differentiated from a client thread by
         * the fact that all received messages are re-transmit upon processing.
         * This allows clients to keep up-to-date whenever new state information
         * is received by the server.
         *
         * Additionally, the spinServerThread will periodically broadcast a ping
         * on infoport, and will perform an update traversal on the scene graph
         * for any nodes who need periodic (scheduled) processing.
         */
        static void *spinServerThread(void *arg);

        /**
         * The syncThread sends timecode on an independent multicast UDP port
         */
        static void *syncThread(void *arg);

        /**
         * The server uses infoCallback to monitor /ping/user messages coming
         * from SPIN clients on the network. A series of timers are used to
         * see if a client is still alive (ie, still sending ping messages). If
         * not, the server will remove the user subgraph and broadcast this to
         * change to all other clients.
         */
        static int infoCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);



        pthread_t syncThreadID; // id of sync thread
        pthread_attr_t syncthreadAttr;
};
#endif
