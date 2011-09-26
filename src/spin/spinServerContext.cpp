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
#include "spinDefaults.h"

#include "SoundConnection.h" // for TCP wildcard check


extern pthread_mutex_t sceneMutex;

namespace spin
{

spinServerContext::spinServerContext() : syncThreadID(0)
{
    using namespace spin_defaults;
    // Important: fist thing to do is set the context mode (client vs server)
    mode = SERVER_MODE;

    // Next, tell spinApp that this is the current context running:
    spinApp &spin = spinApp::Instance();

    // override sender and receiver addresses in server mode:
    lo_rxAddrs_.push_back(lo_address_new(getMyIPaddress().c_str(), SERVER_RX_UDP_PORT));
    lo_txAddrs_.push_back(lo_address_new(MULTICAST_GROUP, SERVER_TX_UDP_PORT));

    // now that we've overridden addresses, we can call setContext
    spin.setContext(this);
}

spinServerContext::~spinServerContext()
{
    using std::map;
    using std::string;
    for (map<string, lo_address>::iterator iter = tcpClientAddrs_.begin();
            iter != tcpClientAddrs_.end();
            ++iter)
        lo_address_free(iter->second);
}

bool spinServerContext::start()
{
    return startThread(&spinServerThread);
}

void spinServerContext::debugPrint()
{
    spinBaseContext::debugPrint();

    std::cout << "  Receiving TCP on:\t\t" << lo_server_get_url(lo_tcpRxServer_) << std::endl;
    std::cout << "  Sending SYNC to:\t\t" << lo_address_get_url(lo_syncAddr); 
    if (lo_address_get_ttl(lo_syncAddr)>0)
        std::cout << " TTL=" << lo_address_get_ttl(lo_syncAddr);
    std::cout << std::endl;

    if (autoCleanup_)
        std::cout << "  Auto-clean inactive users:\tENABLED" << std::endl;
    else
        std::cout << "  Auto-clean inactive users:\tDISABLED" << std::endl;

    if (tcpClientAddrs_.size())
    {
        std::cout << "\nServer has " << tcpClientAddrs_.size() << " subscribers:" << std::endl;
        std::map<std::string, lo_address>::const_iterator client;
        for (client = tcpClientAddrs_.begin(); client != tcpClientAddrs_.end(); ++client)
        {
            std::cout << "  " << client->first << ": " << lo_address_get_url(client->second) << std::endl;
        }
    }
}

void spinServerContext::addCommandLineOptions(osg::ArgumentParser *arguments)
{
    // first, include any base class command line options:
    spinBaseContext::addCommandLineOptions(arguments);

    using namespace spin_defaults;

    arguments->getApplicationUsage()->addCommandLineOption("--recv-udp-msg <host> <port>", "Set the address/port for listening to UDP messages from clients. This argument may be repeated for multiple multicast groups and/or ports (Default: " + std::string(MULTICAST_GROUP) + " " + std::string(CLIENT_RX_UDP_PORT) + ")");
    arguments->getApplicationUsage()->addCommandLineOption("--send-udp-msg <host> <port>", "Set the address/port for UDP multicast of scene events, or this argument may be repeated for several unicast connections (Default: " + std::string(MULTICAST_GROUP) + " " + std::string(SERVER_RX_UDP_PORT) + ")");
    arguments->getApplicationUsage()->addCommandLineOption("--recv-tcp-msg <port>", "Set the port where we listen for subscription requests from clients. Clients may also send scene events to this port is they desire reliability. (Default: " + std::string(SERVER_TCP_PORT) + ")");
    arguments->getApplicationUsage()->addCommandLineOption("--send-udp-sync <host> <port>", "Set the address/port for timecode (sync) messages (Default: " + std::string(MULTICAST_GROUP) + " " + std::string(SYNC_UDP_PORT) + ")");
    arguments->getApplicationUsage()->addCommandLineOption("--ttl <number>", "Set the TTL (time to live) for multicast packets in order to hop across routers (Default: 1)");
    arguments->getApplicationUsage()->addCommandLineOption("--disable-auto-cleanup", "Disables the auto-cleanup of user nodes if they stop sending ping messages.");

}

int spinServerContext::parseCommandLineOptions(osg::ArgumentParser *arguments)
{
    if (!spinBaseContext::parseCommandLineOptions(arguments))
        return 0;

	bool passed_addrs = false;
    std::string addr, port;

    while (arguments->read("--send-udp-msg", addr, port)) {
		if (!passed_addrs) this->lo_txAddrs_.clear();
		this->lo_txAddrs_.push_back(lo_address_new(addr.c_str(), port.c_str()));
		passed_addrs = true;
	}

	passed_addrs = false;
	while (arguments->read("--recv-udp-msg", addr, port)) {
		if (!passed_addrs) this->lo_rxAddrs_.clear();
		this->lo_rxAddrs_.push_back(lo_address_new(addr.c_str(), port.c_str()));
		passed_addrs = true;
	}

	if (arguments->read("--recv-tcp-msg", this->tcpPort_))
    {
        // if user provides manual tcp port, then disallow spin from assigining
        // an automatic port (eg, in the case where that port is busy)
        autoPorts_ = false;
    }

	while (arguments->read("--send-udp-sync", addr, port)) {
		this->lo_syncAddr = lo_address_new(addr.c_str(), port.c_str());
	}

    int ttl=1;
    while (arguments->read("--ttl", ttl)) {
        this->setTTL(ttl);
    }

    if (arguments->read("--disable-auto-cleanup"))
        autoCleanup_ = false;

    return 1;
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
    if (pthread_attr_setdetachstate(&syncthreadAttr, PTHREAD_CREATE_JOINABLE) < 0)
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
	std::vector<lo_server>::iterator servIter;

    /*
    // passing null means we'll be assigned a random port, which we can access later with lo_server_get_port
    lo_tcpRxServer_ = lo_server_new_with_proto(spin_defaults::SERVER_TCP_PORT, LO_TCP, oscParser_error);
    // liblo will try a random free port if the default failed
    if (lo_tcpRxServer_ == 0)
    {
        std::cerr << "TCP server creation on port " << spin_defaults::SERVER_TCP_PORT << 
            " failed, trying a random port" << std::endl;
        lo_tcpRxServer_ = lo_server_new_with_proto(NULL, LO_TCP, oscParser_error);
    }

    std::cout << "  Receiving on TCP channel:\t" << lo_server_get_url(lo_tcpRxServer_) <<
        std::endl;
*/

    spinBaseContext::createServers();
#if 0
    // add OSC callback methods to match various incoming messages:
    // oscCallback_debug() will match any path and args:
    for (servIter = lo_rxServs_.begin(); servIter != lo_rxServs_.end(); ++it)
    {
    	lo_server_add_method((*servIter), NULL, NULL, debugCallback, NULL);
    }
#endif

    // add info channel callback (receives pings from client apps):
    //lo_server_add_method(lo_infoServ, NULL, NULL, infoCallback, this);

    // add tcp channel callback (receives subscribe messages from client apps):
    lo_server_add_method(lo_tcpRxServer_, NULL, NULL, tcpCallback, this);

    // add scene callback
    for (servIter = lo_rxServs_.begin(); servIter != lo_rxServs_.end(); ++servIter)
    {
    	lo_server_add_method((*servIter), std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(),
    			NULL, sceneCallback, NULL);
    }

	lo_server_add_method(lo_tcpRxServer_, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(),
    			NULL, sceneCallback, NULL);
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
    fromString<int>(i_rxPort, lo_address_get_port(context->lo_rxAddrs_[0]));
    fromString<int>(i_txPort, lo_address_get_port(context->lo_txAddrs_[0]));
    fromString<int>(i_syncPort, lo_address_get_port(context->lo_syncAddr));

    UpdateSceneVisitor visitor;
    
    context->running = true;

    // start sync (timecode) thread:
    context->startSyncThread();

    // print some info about addresses/ports to console:
    context->debugPrint();

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
                             lo_address_get_hostname(context->lo_txAddrs_[0]), // server multicasting group
                             i_txPort,  // server multicast port
                             i_syncPort, // server multicast port for sync (timecode)
                             LO_ARGS_END);

            lastTick = frameTick;
        }

        spin.sceneManager->update();

        pthread_mutex_lock(&sceneMutex);
        visitor.apply(*(spin.sceneManager->rootNode.get())); // only server should do this
        pthread_mutex_unlock(&sceneMutex);

        int recv = 0; // bytes received (note: might not be accurate for TCP)
        for (std::vector<lo_server>::iterator it = context->lo_rxServs_.begin(); it != context->lo_rxServs_.end(); ++it)
        	recv += lo_server_recv_noblock((*it), TIMEOUT);
        recv += lo_server_recv_noblock(context->lo_infoServ_, TIMEOUT);
        recv += lo_server_recv_noblock(context->lo_tcpRxServer_, TIMEOUT);

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

        if (recv == 0)
        	usleep(1000);
    }
    context->running = false;
    if (context->syncThreadID != 0)
       pthread_join(context->syncThreadID, NULL);

	spin.destroyScene();
    return arg;
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

    while (spin.getContext()->isRunning())
    {
        //usleep(1000000 * 0.25); // 1/4 second sleep
        usleep(1000000 * 0.5); // 1/2 second sleep

        frameTick = timer->tick();

        //std::cout << "sync time: " << timer->time_s() << "s = " << timer->time_m() << "ms ...  tick = " << (frameTick - startTick ) << std::endl;

        // send the TICK! (in milliseconds)
        //lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sh", "sync", (long long)(frameTick - startTick) );
        lo_send( spin.getContext()->lo_syncAddr, ("/SPIN/" + spin.getSceneID()).c_str(), "sd", "sync", (double)(timer->delta_m(startTick, frameTick)) );
    }
    return 0;
}

int spinServerContext::tcpCallback(const char * path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    if (!argc)
    {
        std::cout << "ERROR: got message for " << path << " without any method or arguments" << std::endl;
        return 1;
    }

    spinServerContext *context = static_cast<spinServerContext*>(user_data);
    std::string method(reinterpret_cast<const char*>(argv[0]));

#if 0
    printf("server got TCP message: %s", path);
    for (int i=0; i<argc; i++) {
    	printf(" ");
		lo_arg_pp((lo_type) types[i], argv[i]);
	}
    printf("\n");
#endif

    // WARNING: tcpCallback is registered to match ANY path, so we must manually
    // check if it is within the /SPIN namespace, and if it matches the sceneID:

    std::string spinToken, sceneString, nodeString;
    std::istringstream pathstream(path);
    pathstream.get(); // ignore leading slash
    getline(pathstream, spinToken, '/');
    getline(pathstream, sceneString, '/');
    getline(pathstream, nodeString, '/');


    if ((spinToken!="SPIN") || !wildcardMatch(sceneString.c_str(),spinApp::Instance().getSceneID().c_str()) )
    {
    	std::cout << "Warning: server is ignoring TCP message: " << path << std::endl;
    	return 1;
    }

    if ((method == "subscribe") && (argc==4))
    {
        std::string clientID(reinterpret_cast<const char*>(argv[1]));

        // if client with this id already exists, free its address
        if (context->tcpClientAddrs_.find(clientID) != context->tcpClientAddrs_.end())
        {
            std::cerr << "WARNING: new client has same ID as existing client, freeing old client\n";
            lo_address_free(context->tcpClientAddrs_[clientID]);
        }

        // port needs to be a string, and the user might send it as either, so
        // we need to check:

        std::string portString;

        if (lo_is_numerical_type((lo_type)types[3]))
        {
        	portString = stringify( (int) lo_hires_val( (lo_type)types[3], argv[3] ) );
        }
        else portString = reinterpret_cast<const char*>(argv[3]);

        context->tcpClientAddrs_[clientID] = lo_address_new_with_proto(LO_TCP,
                    reinterpret_cast<const char*>(argv[2]),
                    portString.c_str());
        std::cout << "Got new subscriber " << clientID << "@" <<
        lo_address_get_url(context->tcpClientAddrs_[clientID]) << std::endl;

        // send a message to the new client to indicate that subscription
        // request was successful:
        //SCENE_MSG("si", "subscribed", 1);
        lo_send(context->tcpClientAddrs_[clientID], ("/SPIN/"+spinApp::Instance().getSceneID()).c_str(), "si", "subscribed", 1, LO_ARGS_END);
    }

    else if (method == "optimize")
    {
    	// TODO: we should really have a SCENE_MSG in spinApp that takes a
    	// txaddr as an argument. For now, we are just using the bundle message
    	std::map<std::string, lo_address>::const_iterator client;
        for (client = context->tcpClientAddrs_.begin();
             client != context->tcpClientAddrs_.end();
             ++client)
        {
        	std::vector<lo_message> msgs;
        	lo_message msg = lo_message_new();
        	lo_message_add(msg, "ss", "optimize", reinterpret_cast<const char*>(argv[1]));
        	msgs.push_back(msg);
			spinApp::Instance().SceneBundle(msgs, client->second);
        }

    }

    else if (method == "getState")
    {
    	osg::ref_ptr<ReferencedNode> n = spinApp::Instance().sceneManager->getNode(reinterpret_cast<const char*>(argv[1]));

    	if (n.valid())
    	{
    		std::vector<lo_message> state = n->getState();

			std::cout << "WARNING: UNIMPLEMENTED: TCP request 'getState' for node " << reinterpret_cast<const char*>(argv[1]) << std::endl;

			// TODO: send node bundle back to sender

    	}

    }

	// WE DON'T NEED TO DO ANYHING MORE. WE REGISTER WITH THE TCP SERVER NOW...
	// FOR THE SCENE, AND EACH NODE, STATESET, AND SOUNDCONNECTION
	
	
    // any other scene message just gets forwarded to the generic (UDP)
    // sceneCallback
    else if (nodeString.empty())
    {
        //spinBaseContext::sceneCallback(path, types, argv, argc, (void*) data, (void*) user_data);
    }


    else
    {
		// OLD WAY (wildcards don't work)
		/*
	    ReferencedNode* n = spinApp::Instance().sceneManager->getNode(nodeID);
	    if (n) spinBaseContext::nodeCallback(path, types, argv, argc, (void*) data, (void*) n->id);

	    else
	    {
	    	ReferencedStateSet* s = spinApp::Instance().sceneManager->getStateSet(nodeID.c_str());
	    	if (s) spinBaseContext::nodeCallback(path, types, argv, argc, (void*) data, (void*) s->id);
	    }
		*/

		
		// WE DON'T NEED TO DO THIS ANYMORE. WE REGISTER THE TCP CALLBACK WITH
		// EACH NODE, STATESET, AND SOUNDCONNECTION
		
		// The nodeString might have a wildcard, so here we call the method on
		// any nodes (or statesets) that match:
		/*
		std::vector<t_symbol*> matched = spinApp::Instance().sceneManager->findNodes(nodeString.c_str());

		std::vector<t_symbol*>::iterator iter;
		for (iter = matched.begin(); iter != matched.end(); ++iter)
		{
			spinBaseContext::nodeCallback(path, types, argv, argc, (void*) data, (void*) (*iter));
		}


		// connections are different:
		std::vector<SoundConnection*> conns = spinApp::Instance().sceneManager->getConnections();
		
		std::vector<SoundConnection*>::iterator cIter;
		for ( cIter=conns.begin(); cIter!=conns.end(); ++cIter )
		{
			if (wildcardMatch(nodeString.c_str(), (*cIter)->id->s_name))
			{
				std::cout << " ... matched connection: " << (*cIter)->id->s_name << std::endl;
				spinBaseContext::connectionCallback(path, types, argv, argc, (void*) data, (void*) (*iter));
			}
		}
		*/

	}

    return 1;
}

void spinServerContext::refreshSubscribers()
{
	// TODO: call getState on all nodes in sceneManager and send them over TCP
	// to each of the subscribers (ie, everyone in tcpClientAddrs_)
	// other option: use ReferencedNode::stateDump, and SceneManager::refresh
	// to send messages...

    spinApp::Instance().sceneManager->refreshSubscribers(tcpClientAddrs_);
}

} // end of namespace spin


