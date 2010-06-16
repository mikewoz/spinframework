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

#include <string>
#include <iostream>
#include <pthread.h>
#include <signal.h>


#include <osgDB/Registry>
#include <osgIntrospection/Type>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <boost/python.hpp>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "SceneManager.h"
#include "spinBaseContext.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "spinLog.h"
#include "nodeVisitors.h"

#define UNUSED(x) ( (void)(x) )

// TODO: make mutex a static member (perhaps of spinApp)?
pthread_mutex_t pthreadLock = PTHREAD_MUTEX_INITIALIZER;


bool spinBaseContext::signalStop = false;



spinBaseContext::spinBaseContext()
{
	signalStop = true;
    running = false;

    signal(SIGINT, sigHandler);


    // set default addresses (can be overridden):
    lo_infoAddr = lo_address_new("224.0.0.1", "54320");
    lo_rxAddr = lo_address_new("224.0.0.1", "54323");
    lo_txAddr = lo_address_new("224.0.0.1", "54324");
    lo_syncAddr = lo_address_new("224.0.0.1", "54321");


    // override infoPort based on environment variable:
    char *infoPortStr = getenv("AS_INFOPORT");
    if (infoPortStr)
    {
        std::string tmpStr = std::string(infoPortStr);
        std::string infoAddr = tmpStr.substr(0,tmpStr.rfind(":"));
        std::string infoPort = tmpStr.substr(tmpStr.find(":")+1);
        lo_infoAddr = lo_address_new(infoAddr.c_str(), infoPort.c_str());
    }

    // set up infoPort listener thread:
    if (isMulticastAddress(lo_address_get_hostname(lo_infoAddr)))
    {
        lo_infoServ = lo_server_thread_new_multicast(lo_address_get_hostname(lo_infoAddr), lo_address_get_port(lo_infoAddr), oscParser_error);

    } else if (isBroadcastAddress(lo_address_get_hostname(lo_infoAddr)))
    {
        lo_infoServ = lo_server_thread_new(lo_address_get_port(lo_infoAddr), oscParser_error);
        int sock = lo_server_get_socket_fd(lo_server_thread_get_server(lo_infoServ));
        int sockopt = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &sockopt, sizeof(sockopt));

    } else {
        lo_infoServ = lo_server_thread_new(lo_address_get_port(lo_infoAddr), oscParser_error);
    }
    lo_server_thread_start(lo_infoServ);

}

spinBaseContext::~spinBaseContext()
{
    this->stop();

    if (lo_infoServ)
    {
        lo_server_thread_stop(lo_infoServ);
        lo_server_thread_free(lo_infoServ);
    }

    if (lo_rxAddr) lo_address_free(lo_rxAddr);
    if (lo_txAddr) lo_address_free(lo_txAddr);
    if (lo_infoAddr) lo_address_free(lo_infoAddr);
    if (lo_syncAddr) lo_address_free(lo_syncAddr);
}



void spinBaseContext::sigHandler(int signum)
{
    std::cout << " Caught signal: " << signum << std::endl;

    spinApp &spin = spinApp::Instance();

    // unlock mutex so we can clean up:
    pthread_mutex_unlock(&pthreadLock);

    // TODO: we really shouldn't do anything like this here. Can we get rid of
    // this?:
    if (spin.userNode.valid())
    {
        ReferencedNode *heldPointer = spin.userNode.get();
        //spin.userNode.release();
        spin.userNode = NULL;
        // now deleting the node in the sceneManager should release the last instance:
        spin.sceneManager->doDelete(heldPointer);
    }

    spinBaseContext::signalStop = true;

}



bool spinBaseContext::startThread( void *(*threadFunction) (void*) )
{
    std::cout << "  INFO channel:\t\t\t" << lo_address_get_url(lo_infoAddr) << std::endl;
    std::cout << "  SYNC channel:\t\t\t" << lo_address_get_url(lo_syncAddr) << std::endl;

    signalStop = false;

    // create thread:
    if (pthread_attr_init(&pthreadAttr) < 0)
    {
        std::cout << "spinBaseContext: could not prepare child thread" << std::endl;
        return false;
    }
    if (pthread_attr_setdetachstate(&pthreadAttr, PTHREAD_CREATE_DETACHED) < 0)
    {
        std::cout << "spinBaseContext: could not prepare child thread" << std::endl;
        return false;
    }
    if (pthread_create( &pthreadID, &pthreadAttr, threadFunction, this) < 0)
    {
        std::cout << "spinBaseContext: could not create new thread" << std::endl;
        return false;
    }

    //pthread_join(pthreadID, NULL); // if not DETACHED thread

    // wait until the thread gets into it's loop before returning:
    while (!running) usleep(10);

    return true;
}

void spinBaseContext::stop()
{
    if (isRunning())
    {
        std::cout << "Stopping spinBaseContext..." << std::endl;
        signalStop = true;
        while (running) usleep(10);
    }
}



