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


#include "spinApp.h"
#include "spinServerContext.h"
#include "SceneManager.h"
#include "spinLog.h"
#include "nodeVisitors.h"

extern pthread_mutex_t pthreadLock;

spinServerContext::spinServerContext()
{
	// Important: fist thing to do is set the context mode (client vs server)
    mode = SERVER_MODE;

	// Next, tell spinApp that this is the current context running:
    spinApp &spin = spinApp::Instance();
    spin.setContext(this);




	// override sender and receiver addresses in server mode:
    lo_rxAddr = lo_address_new(getMyIPaddress().c_str(), "54324");
    lo_txAddr = lo_address_new("226.0.0.1", "54323");

    // add info channel callback (receives pings from client apps):
    lo_server_thread_add_method(lo_infoServ, NULL, NULL, infoCallback, this);

}

spinServerContext::~spinServerContext()
{

}

bool spinServerContext::start()
{
	return startThread(&spinServerThread);
}

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************


void spinServerContext::startSyncThread()
{
    // create thread:
    if (pthread_attr_init(&syncthreadAttr) < 0)
    {
        std::cout << "spinServerContext: could not prepare sync thread" << std::endl;
    }
    if (pthread_attr_setdetachstate(&syncthreadAttr, PTHREAD_CREATE_DETACHED) < 0)
    {
        std::cout << "spinServerContext: could not prepare sync thread" << std::endl;
    }
    if (pthread_create( &syncThreadID, &syncthreadAttr, syncThread, this) < 0)
    {
        std::cout << "spinServerContext: could not create sync thread" << std::endl;
    }
}

// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// *****************************************************************************

void *spinServerContext::spinServerThread(void *arg)
{
	spinServerContext *thiss = (spinServerContext*)(arg);
	spinApp &spin = spinApp::Instance();


    spin.sceneManager = new SceneManager(spin.getSceneID(), lo_address_get_hostname(spin.getContext()->lo_rxAddr), lo_address_get_port(spin.getContext()->lo_rxAddr));
    spin.sceneManager->setTXaddress(lo_address_get_hostname(spin.getContext()->lo_txAddr), lo_address_get_port(spin.getContext()->lo_txAddr));

    if ( !spin.initPython() )
        printf("Python initialization failed.\n");
    std::string cmd = "sys.path.append('" + spin.sceneManager->resourcesPath + "/scripts')";

    spin.execPython(cmd);
    spin.execPython("import spin");


    // create log filename based on datetime:
    time_t t = time(NULL);
    tm* tmp = localtime(&t);
    char dateString[128];
    strftime(dateString, sizeof(dateString), "%Y-%m-%d_%H-%M-%S", tmp);

    // start spinLog, and disable console printing:
    std::string logFilename = SPIN_DIRECTORY + "/log/spinLog_" + std::string(dateString) + ".txt";
    spinLog log(logFilename.c_str());
    log.enable_cout(false);
    spin.sceneManager->setLog(log);


    std::string myIP = getMyIPaddress();
    osg::Timer_t lastTick = osg::Timer::instance()->tick();
    osg::Timer_t frameTick = lastTick;

    // convert ports to integers for sending:
    int i_rxPort, i_txPort, i_syncPort, i_tcpPort;
    fromString<int>(i_rxPort, lo_address_get_port(spin.getContext()->lo_rxAddr));
    fromString<int>(i_txPort, lo_address_get_port(spin.getContext()->lo_txAddr));
    fromString<int>(i_syncPort, lo_address_get_port(spin.getContext()->lo_syncAddr));
    fromString<int>(i_tcpPort, lo_address_get_port(spin.getContext()->lo_tcpAddr));

    UpdateSceneVisitor visitor;


    //lo_server_thread_add_method(spin->sceneManager->rxServ, NULL, NULL, sceneCallback, spin);

    // start sync (timecode) thread:
    thiss->startSyncThread();


    thiss->running = true;
    while (!spinBaseContext::signalStop)
    {
        frameTick = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
        {
            spin.InfoMessage("/SPIN/__server__", "ssisiii", spin.getSceneID().c_str(),
                    myIP.c_str(), i_rxPort,
                    lo_address_get_hostname(spin.getContext()->lo_txAddr), i_txPort,
                    i_syncPort,
                    i_tcpPort,
                    LO_ARGS_END);
            //lo_send_from(spin->lo_infoAddr, spin->lo_infoServ, LO_TT_IMMEDIATE, "/ping/SPIN", "ssisi", spin->id.c_str(), myIP.c_str(), i_rxPort, spin->txAddr.c_str(), i_txPort);
            lastTick = frameTick;
        }

        pthread_mutex_lock(&pthreadLock);
        visitor.apply(*(spin.sceneManager->rootNode.get())); // only server should do this
        pthread_mutex_unlock(&pthreadLock);

        usleep(1000);
    }
    thiss->running = false;


    // clean up:
    delete spin.sceneManager;

    pthread_exit(NULL);
}

/**
 * syncThread is an independent thread that just sends
 * timecode information on the sync port (multicast)
 */
void *spinServerContext::syncThread(void *arg)
{
	spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();


    osg::Timer_t startTick = timer->tick();
    timer->setStartTick(startTick);
    spin.setSyncStart(startTick);
    osg::Timer_t frameTick = startTick;

    //while (spin.isRunning())
    while (1)
    {
        //usleep(1000000 * 0.25); // 1/4 second sleep
        usleep(1000000 * 0.5); // 1/2 second sleep

        frameTick = timer->tick();

        //std::cout << "sync time: " << timer->time_s() << "s = " << timer->time_m() << "ms ...  tick = " << (frameTick - startTick ) << std::endl;
        //lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sh", "sync", (long long)(frameTick - startTick) );
        //lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sh", "sync", (long long)(timer->time_m()) );
        lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sd", "sync", (double)(timer->time_m()) );
    }
    return 0;
}

int spinServerContext::infoCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    //spinApp &spin = spinApp::Instance();


    // TODO: monitor /ping/user messages, keep timeout handlers, and remove
   	// users who are no longer pinging


    return 1;
}
