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
#include <sstream>
#include <boost/lexical_cast.hpp>

#include "spinClientContext.h"
#include "spinApp.h"
#include "SceneManager.h"
#include "spinDefaults.h"

namespace spin
{


spinClientContext::spinClientContext() : 
    lo_syncServ(NULL),
    doSubscribe_(true)
{
    using namespace spin_defaults;
    // Important: first thing to do is set the context mode (client vs server)
    mode = CLIENT_MODE;

    tcpPort_ = CLIENT_TCP_PORT;

    // start by assuming the spinserver is on localhost:
    lo_serverTCPAddr = lo_address_new_with_proto(LO_TCP, "localhost", SERVER_TCP_PORT);

    // Next, tell spinApp that this is the current context running:
    spinApp &spin = spinApp::Instance();
    spin.setContext(this);
    lo_rxAddrs_.push_back(lo_address_new(MULTICAST_GROUP, CLIENT_RX_UDP_PORT));
    lo_txAddrs_.push_back(lo_address_new(MULTICAST_GROUP, CLIENT_TX_UDP_PORT));
}

spinClientContext::~spinClientContext()
{
    lo_address_free(lo_serverTCPAddr);
    lo_server_free(lo_syncServ);
}

bool spinClientContext::start()
{
    return startThread(&spinClientThread);
}

void spinClientContext::debugPrint()
{
    spinBaseContext::debugPrint();

    std::cout << "  Sending TCP to:\t\t" << lo_address_get_url(lo_serverTCPAddr) << std::endl;
    std::cout << "  Receiving SYNC on:\t\t" << lo_address_get_url(lo_syncAddr) << " TTL=" << lo_address_get_ttl(lo_syncAddr) << std::endl;


    // print other client-specific details:
    std::cout << "  UserNode id:\t\t\t" << spinApp::Instance().getUserID() << std::endl;
}

void spinClientContext::addCommandLineOptions(osg::ArgumentParser *arguments)
{
    // first, include any base class command line options:
    spinBaseContext::addCommandLineOptions(arguments);

    using namespace spin_defaults;

    arguments->getApplicationUsage()->addCommandLineOption("--recv-udp-msg <host> <port>", "Set the receiving address/port for UDP messages from the server. The address can be a multicast address, or 'localhost'. (Default: " + std::string(MULTICAST_GROUP) + " " + std::string(CLIENT_RX_UDP_PORT) + ")");
    arguments->getApplicationUsage()->addCommandLineOption("--send-udp-msg <host> <port>", "Specify the address/port of the server's UDP channel. This is where we stream high-throughput scene events, such as position updates (Default: " + std::string(MULTICAST_GROUP) + " " + std::string(SERVER_RX_UDP_PORT) + ")");
	arguments->getApplicationUsage()->addCommandLineOption("--send-tcp-msg <host> <port>", "Specify the address/port of the server's TCP channel. This is wwhere we send subscription requests, and scene events that require reliable transmission (Default: localhost " + std::string(SERVER_TCP_PORT) + ")");
	arguments->getApplicationUsage()->addCommandLineOption("--recv-tcp-msg <port>", "Set the desired receiving TCP port when subscribing to the server. ie, spinserver will connect back to this port once we have subscribed (Default: " + std::string(CLIENT_TCP_PORT) + ")");
	arguments->getApplicationUsage()->addCommandLineOption("--recv-udp-sync <address> <port>", "Set the address/port for timecode (sync) messages (Default: " + std::string(MULTICAST_GROUP) + " " + std::string(SYNC_UDP_PORT) + ")");
    arguments->getApplicationUsage()->addCommandLineOption("--ttl <number>", "Set the TTL (time to live) for multicast packets in order to hop across routers (Default: 1)");

}

void spinClientContext::parseCommandLineOptions(osg::ArgumentParser *arguments)
{
    spinBaseContext::parseCommandLineOptions(arguments);

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

	arguments->read("--recv-tcp-msg", this->tcpPort_);
	
    while (arguments->read("--send-tcp-msg", addr, port)){
        lo_serverTCPAddr = lo_address_new_with_proto(LO_TCP, addr.c_str(), port.c_str());
    }

	while (arguments->read("--send-udp-sync", addr, port)) {
		this->lo_syncAddr = lo_address_new(addr.c_str(), port.c_str());
	}

    int ttl=1;
    while (arguments->read("--ttl", ttl)) {
        this->setTTL(ttl);
    }

}

// FIXME: Push this up to base context
void spinClientContext::createServers()
{
	std::vector<lo_server>::iterator servIter;

    using boost::lexical_cast;
    using std::string;

    /*
    // passing null means we'll be assigned a random port, which we can access later with lo_server_get_port
    lo_tcpRxServer_ = lo_server_new_with_proto(NULL, LO_TCP, oscParser_error);
    std::cout << "  TCP channel:\t\t\t" << lo_server_get_url(lo_tcpRxServer_) <<
        std::endl;
*/

    spinBaseContext::createServers();

#if 0
    // add OSC callback methods to match various incoming messages:
    // oscCallback_debug() will match any path and args:

    for (servIter = lo_rxServs_.begin(); servIter != lo_rxServs_.end(); ++servIter)
    {
    	lo_server_add_method((*servIter), NULL, NULL, debugCallback, NULL);
    }
#endif

    lo_server_add_method(lo_infoServ_, NULL, NULL, infoCallback, this);
    lo_server_add_method(lo_tcpRxServer_, NULL, NULL, tcpCallback, this);

    for (servIter = lo_rxServs_.begin(); servIter != lo_rxServs_.end(); ++servIter)
    {
    	lo_server_add_method((*servIter),
            std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(),
            NULL, sceneCallback, NULL);
    }

    // sync (timecode) receiver:
    if (isMulticastAddress(lo_address_get_hostname(lo_syncAddr)))
    {
        lo_syncServ = lo_server_new_multicast(lo_address_get_hostname(lo_syncAddr), lo_address_get_port(lo_syncAddr), oscParser_error);
        if (lo_syncServ == 0)
        {
            std::cerr << "Sync server creation on port " << lo_address_get_port(lo_syncAddr) << 
                " failed, trying a random port" << std::endl;
            std::string addr(lo_address_get_hostname(lo_syncAddr));
            lo_address_free(lo_syncAddr);
            lo_syncServ = lo_server_new_multicast(addr.c_str(), NULL, oscParser_error);
            lo_syncAddr = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(lo_syncServ)).c_str());
        }
    } else {
        lo_syncServ = lo_server_new(lo_address_get_port(lo_syncAddr), oscParser_error);
        if (lo_syncServ == 0)
        {
            std::cerr << "Sync server creation on port " << lo_address_get_port(lo_syncAddr) << 
                " failed, trying a random port" << std::endl;
            std::string addr(lo_address_get_hostname(lo_syncAddr));
            lo_address_free(lo_syncAddr);
            lo_syncServ = lo_server_new(NULL, oscParser_error);
            lo_syncAddr = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(lo_syncServ)).c_str());
        }
    }
    lo_server_add_method(lo_syncServ, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), 
            NULL, syncCallback, NULL);
}

int spinClientContext::pollUpdates()
{
	static const int TIMEOUT = 0;
	int recv = 0; // bytes received (note: might not be accurate for TCP)
    for (std::vector<lo_server>::iterator it = lo_rxServs_.begin(); it != lo_rxServs_.end(); ++it)
    	recv += lo_server_recv_noblock((*it), TIMEOUT);
	recv += lo_server_recv_noblock(lo_syncServ, TIMEOUT);
	recv += lo_server_recv_noblock(lo_infoServ_, TIMEOUT);
	recv += lo_server_recv_noblock(lo_tcpRxServer_, TIMEOUT);

	return recv;
}

void *spinClientContext::spinClientThread(void *arg)
{
    spinClientContext *context = (spinClientContext*)(arg);
    spinApp &spin = spinApp::Instance();
    context->createServers();
    spin.createScene();

    if ( !spin.initPython() )
        printf("Python initialization failed.\n");
    std::string cmd = "sys.path.append('" + spin.sceneManager->resourcesPath + "/scripts')";

    spin.execPython(cmd);
    spin.execPython("import spin");

    osg::Timer_t lastTick = osg::Timer::instance()->tick();
    osg::Timer_t frameTick = lastTick;


    context->running = true;

    // registerUser needs the context to be running (since it sends messages)
    spin.registerUser();

    context->debugPrint();

    // TIMEOUT in liblo was 10; We set it to zero, and sleep only if there are
    // no received messages. ie, if there are messages, we eat them as fast as
    // possible; otherwise, we let the CPU relax a bit
    static const int TIMEOUT = 0;
    while (!spinBaseContext::signalStop)
    {
        int recv = context->pollUpdates();
        
        if (recv == 0)
        	usleep(10);

        // just send a ping so the server knows we are still here
        frameTick = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
        {
        	spin.NodeMessage(spin.getUserID().c_str(), "s", "ping", LO_ARGS_END);
            lastTick = frameTick;
        }
    }

    std::cout << "Exiting spin client thread\n";
    context->running = false;

	spin.destroyScene();
    return arg;
}

int spinClientContext::syncCallback(const char * /*path*/, const char *types, lo_arg **argv, int argc,
        lo_message /*msg*/, void * /*user_data*/)
{
    spinApp &spin = spinApp::Instance();
    osg::Timer* timer = osg::Timer::instance();

    osg::Timer_t masterTick, slaveTick, off, startTick;

    // the incoming message should look like this: /SPIN/default sync 234.723

    if (argc != 2) return 1;

    std::string theMethod;
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else return 1;

    if (theMethod != "sync") return 1;

    if (lo_is_numerical_type((lo_type)types[1])) {
        masterTick = (osg::Timer_t) lo_hires_val((lo_type)types[1], argv[1]);
        //std::cout << "MASTERTICK = " << masterTick << std::endl;

        startTick = spin.getSyncStart();
        if (startTick == 0) {
            ///printf("setSyncStart....\n");
            startTick = timer->tick() - masterTick;
            timer->setStartTick(startTick);
            spin.setSyncStart(startTick);
        }

        slaveTick = timer->tick() - startTick;

        if (slaveTick > masterTick) { // EARLY!
            off = slaveTick - masterTick;
            startTick += off * 0.1;  /////SYNC_CONVERGE_FACTOR
            timer->setStartTick(startTick);
            spin.setSyncStart(startTick);// useless?

        } else if (slaveTick < masterTick) { // WE'RE LATE!
            off = masterTick - slaveTick;
            startTick -= off * 0.1;
            timer->setStartTick(startTick);
            spin.setSyncStart(startTick);

        }

        //std::cout << "start = " <<  startTick << " m=" <<  masterTick << " s=" << slaveTick << " o=" <<  off << std::endl;
        //std::cout << "got new sync time: " << timer->time_s() << std::endl;
    }

    return 1;
}

/// this comes from the server's multicast info message
int spinClientContext::infoCallback(const char * /*path*/, const char * /*types*/,
        lo_arg ** argv, int argc, void * /*data*/, void * user_data)
{
    spinClientContext *context = static_cast<spinClientContext*>(user_data);
    if (argc != 7)
        return 1;
    std::string theirSceneID(reinterpret_cast<const char*>(argv[0]));

    // make sure my sceneID matches the sceneID whose info message this is
    if (spinApp::Instance().getSceneID() == theirSceneID and context->doSubscribe_)
    {
        std::ostringstream sstr;
        sstr << argv[3]->i;    // convert to string
        context->lo_serverTCPAddr = lo_address_new_with_proto(LO_TCP,
                reinterpret_cast<const char*>(argv[1]),
                sstr.str().c_str());
        context->subscribe();
    }

    return 1;
}


/// this handles tcp communication from the server
int spinClientContext::tcpCallback(const char *path, const char *types,
        lo_arg **argv, int argc, void *data, void *user_data)
{
    // WARNING: tcpCallback is registered to match ANY path, so we must manually
    // check if it is within the /SPIN namespace, and if it matches the sceneID:

    std::string spinToken, sceneID, nodeID;
    std::istringstream pathstream(path);
    pathstream.get(); // ignore leading slash
    getline(pathstream, spinToken, '/');
    getline(pathstream, sceneID, '/');
    getline(pathstream, nodeID, '/');

    if ((spinToken!="SPIN") || (sceneID!=spinApp::Instance().getSceneID()))
    {
    	std::cout << "Warning: server is ignoring TCP message: " << path << std::endl;
    	return 1;
    }

	// It's a valid message, so we just forward it to the regular OSC callback
    // methods:
/*
    if (nodeID.empty())
    {
    	// scene message:
        spinBaseContext::sceneCallback(path, types, argv, argc, (void*) data, (void*) user_data);
    }
    else
    {
    	// node message:
	    ReferencedNode* n = spinApp::Instance().sceneManager->getNode(nodeID);
	    if (n)
	    {
	    	spinBaseContext::nodeCallback(path, types, argv, argc, (void*) data, (void*) n->id);
	    }
	}
*/


    return 1;
}

void spinClientContext::subscribe()
{
	// Tthis should also be called whenever client gets a UserRefresh...
	// the idea is that if the server crashed and came back online, it sends a
	// userRefresh message and the client can re-subscribe.

    // TODO: the server should send a message to confirm the subscription, and
    // doSubscribe_ should only be set to false then. Otherwise, it's possible
    // that the subscription will fail, and future info channel messages will
    // have no effect because the flag is not set.

    if (!lo_tcpRxServer_)
    {
        std::cout << "ERROR: tried to subscribe, but no TCP receive port is specified" << std::endl;
        return;
    }

	std::stringstream sstr;
	// convert to port number to string
	sstr << lo_server_get_port(lo_tcpRxServer_);

	lo_send(lo_serverTCPAddr, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), "ssss",
			"subscribe", spinApp::Instance().getUserID().c_str(),
			getMyIPaddress().c_str(),
			sstr.str().c_str());

    doSubscribe_ = false;
}

} // end of namespace spin


