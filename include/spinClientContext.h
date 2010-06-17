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

#ifndef __spinClientContext_H
#define __spinClientContext_H

#include "spinBaseContext.h"


/**
 * \brief A client-side spinContext for listening to a remote server
 *
 *
 */
class spinClientContext : public spinBaseContext
{
    public:

		spinClientContext();
		~spinClientContext();

		bool start();

        lo_server_thread lo_syncServ;

    private:
        /**
         * The spinClientThread is a simple thread that starts a sceneManager and
         * listens to incoming SPIN messages. It does NOT re-transmit those messages,
         * and it does NOT perform an update traversal.
         */
        static void *spinClientThread(void *arg);

        /**
         * The sceneCallback is used for to listen to userRefresh messages.
         * If this client is running before the server comes up, the server will
         * send a userRefresh message, and this client can re-send his user info
		 */
        static int sceneCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

        /**
         * The syncCallback allows a client to listen to timecode originating at
         * the server, and adjust its internal clock to match the server
         */
        static int syncCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
        
        /**
         * The client uses infoCallback to find out which port it can connect to 
         * on a server. 
         */
        static int infoCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

};
#endif
