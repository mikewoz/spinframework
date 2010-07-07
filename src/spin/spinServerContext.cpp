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

extern pthread_mutex_t sceneMutex;

spinServerContext::spinServerContext()
{
    // Important: fist thing to do is set the context mode (client vs server)
    mode = SERVER_MODE;

    // Next, tell spinApp that this is the current context running:
    spinApp &spin = spinApp::Instance();

    // override sender and receiver addresses in server mode:
    lo_rxAddr = lo_address_new(getMyIPaddress().c_str(), "54324");
    lo_txAddr = lo_address_new("226.0.0.1", "54323");

    std::cout << "RX SceneManager receiving on:\t" <<
        lo_address_get_url(lo_rxAddr) << std::endl;

    // now that we've overridden addresses, we can call setContext
    spin.setContext(this);
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

// FIXME: Push this up to base context
void spinServerContext::createServers()
{
    // passing null means we'll be assigned a random port, which we can access later with lo_server_get_port
    lo_tcpRxServer_ = lo_server_new_with_proto(NULL, LO_TCP, oscParser_error);
    std::cout << "  TCP channel:\t\t\t" << lo_server_get_url(lo_tcpRxServer_) <<
        std::endl;

    // set up OSC event listener:

    if (isMulticastAddress(lo_address_get_hostname(lo_rxAddr)))
        lo_rxServ_ = lo_server_new_multicast(lo_address_get_hostname(lo_rxAddr), lo_address_get_port(lo_rxAddr), oscParser_error);
    else
        lo_rxServ_ = lo_server_new(lo_address_get_port(lo_rxAddr), oscParser_error);

#if 0
    // add OSC callback methods to match various incoming messages:
    // oscCallback_debug() will match any path and args:
    lo_server_add_method(lo_rxServ_, NULL, NULL, debugCallback, NULL);
#endif

    // set up infoPort listener thread:
    if (isMulticastAddress(lo_address_get_hostname(lo_infoAddr)))
    {
        lo_infoServ = lo_server_new_multicast(lo_address_get_hostname(lo_infoAddr), lo_address_get_port(lo_infoAddr), oscParser_error);
    } else if (isBroadcastAddress(lo_address_get_hostname(lo_infoAddr)))
    {
        lo_infoServ = lo_server_new(lo_address_get_port(lo_infoAddr), oscParser_error);
        int sock = lo_server_get_socket_fd(lo_infoServ);
        int sockopt = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &sockopt, sizeof(sockopt));

    } else {
        lo_infoServ = lo_server_new(lo_address_get_port(lo_infoAddr), oscParser_error);
    }
    // add info channel callback (receives pings from client apps):
    //lo_server_add_method(lo_infoServ, NULL, NULL, infoCallback, this);

    // add tcp channel callback (receives subscribe messages from client apps):
    lo_server_add_method(lo_tcpRxServer_, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), NULL, tcpCallback, this);

    // add scene callback
    lo_server_add_method(lo_rxServ_, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), NULL, sceneCallback, NULL);

}

void *spinServerContext::spinServerThread(void *arg)
{
    spinServerContext *context = (spinServerContext*)(arg);
    spinApp &spin = spinApp::Instance();
    context->createServers();
    spin.createScene();

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
    context->setLog(log);

    std::string myIP = getMyIPaddress();
    osg::Timer_t lastTick = osg::Timer::instance()->tick();
    osg::Timer_t frameTick = lastTick;

    // convert ports to integers for sending:
    int i_rxPort, i_txPort, i_syncPort;
    fromString<int>(i_rxPort, lo_address_get_port(context->lo_rxAddr));
    fromString<int>(i_txPort, lo_address_get_port(context->lo_txAddr));
    fromString<int>(i_syncPort, lo_address_get_port(context->lo_syncAddr));

    UpdateSceneVisitor visitor;

    // start sync (timecode) thread:
    context->startSyncThread();

    context->running = true;
    static const int TIMEOUT = 0;
    while (!spinBaseContext::signalStop)
    {
        frameTick = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
        {
            spin.InfoMessage("/SPIN/__server__", "ssiisii",
                             spin.getSceneID().c_str(),
                             myIP.c_str(), // server's IP address
                             i_rxPort, // server's receiving port
                             lo_server_get_port(context->lo_tcpRxServer_), // server's receiving TCP port
                             lo_address_get_hostname(context->lo_txAddr), // server multicasting group
                             i_txPort,  // server multicast port
                             i_syncPort, // server multicast port for sync (timecode)
                             LO_ARGS_END);

            lastTick = frameTick;
        }

        pthread_mutex_lock(&sceneMutex);
        visitor.apply(*(spin.sceneManager->rootNode.get())); // only server should do this
        pthread_mutex_unlock(&sceneMutex);

        lo_server_recv_noblock(context->lo_infoServ, TIMEOUT);
        lo_server_recv_noblock(context->lo_rxServ_, TIMEOUT);
        lo_server_recv_noblock(context->lo_tcpRxServer_, TIMEOUT);

        // Need to sleep a little bit so that updates have time. 2 reasons:
        //
        // 1) the CPU used for this thread is huge, because it never rests
        // 2) Without a sleep, the attach() method in ReferencedNode takes a
        // long time. This is probably true for any functions that grab a mutex.
        // Maybe we don't need a mutex anymore, since it's all done in one
        // thread?
        //
        // NOTE: if the sleep is too long, messages will start to accumulate,
        // and will be processed with delay.
        //
        //
        usleep(10);

    }
    context->running = false;

    pthread_exit(NULL);
}

/**
 * syncThread is an independent thread that just sends
 * timecode information on the sync port (multicast)
 */
void *spinServerContext::syncThread(void * /*arg*/)
{
    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();


    osg::Timer_t startTick = timer->tick();
    timer->setStartTick(startTick);
    spin.setSyncStart(startTick);
    osg::Timer_t frameTick = startTick;

    //while (spin.isRunning())
    while (true)
    {
        //usleep(1000000 * 0.25); // 1/4 second sleep
        usleep(1000000 * 0.5); // 1/2 second sleep

        frameTick = timer->tick();

        //std::cout << "sync time: " << timer->time_s() << "s = " << timer->time_m() << "ms ...  tick = " << (frameTick - startTick ) << std::endl;

        // send the TICK!  not the time in ms.
        lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sh", "sync", (long long)(frameTick - startTick) );
        //lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sh", "sync", (long long)(timer->time_m()) );
        //lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sd", "sync", (double)(timer->time_m()) );
    }
    return 0;
}

int spinServerContext::tcpCallback(const char * path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    spinServerContext *context = static_cast<spinServerContext*>(user_data);
    std::string method(reinterpret_cast<const char*>(argv[0]));

    if (method == "subscribe")
    {
        std::string clientID(reinterpret_cast<const char*>(argv[1]));

        // if client with this id already exists, free its address
        if (context->tcpClientAddrs_.find(clientID) != context->tcpClientAddrs_.end())
        {
            std::cerr << "WARNING: new client has same ID as existing client, freeing old client\n";
            lo_address_free(context->tcpClientAddrs_[clientID]);
        }
        context->tcpClientAddrs_[clientID] = lo_address_new_with_proto(LO_TCP,
                    reinterpret_cast<const char*>(argv[2]),
                    reinterpret_cast<const char*>(argv[3]));
        std::cout << "Got new subscriber " << clientID << "@" <<
        lo_address_get_url(context->tcpClientAddrs_[clientID]) << std::endl;
    }

    return 1;
}


void spinServerContext::refreshSubscribers()
{
	// TODO: call getState on all nodes in sceneManager and send them over TCP
	// to each of the subscribers (ie, everyone in tcpClientAddrs_)
	// other option: use ReferencedNode::stateDump, and SceneManager::refresh
	// to send messages...

	std::cout << "Got spinServerContext::refreshSubsribers" << std::endl;
    spinApp::Instance().sceneManager->refreshSubscribers(tcpClientAddrs_);
}

