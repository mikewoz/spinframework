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

#include "spinClientContext.h"
#include "spinApp.h"
#include "SceneManager.h"


spinClientContext::spinClientContext() : 
    doSubscribe_(true),
    lo_syncServ(NULL),
    lo_serverTCPAddr(NULL)
{
    // Important: fist thing to do is set the context mode (client vs server)
    mode = CLIENT_MODE;

    // Next, tell spinApp that this is the current context running:
    spinApp &spin = spinApp::Instance();
    spin.setContext(this);
    lo_rxAddr = lo_address_new("226.0.0.1", "54323");
    lo_txAddr = lo_address_new("226.0.0.1", "54324");
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

// FIXME: Push this up to base context
void spinClientContext::createServers()
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
    lo_server_add_method(lo_infoServ, NULL, NULL, infoCallback, this);
    lo_server_add_method(lo_tcpRxServer_, NULL, NULL, tcpCallback, this);

    lo_server_add_method(lo_rxServ_,
            std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(),
            NULL, sceneCallback, NULL);
    std::cout << "  SceneManager receiving on:\t" <<
        lo_address_get_url(lo_rxAddr) << std::endl;

    // sync (timecode) receiver:
    if (isMulticastAddress(lo_address_get_hostname(lo_syncAddr)))
    {
        lo_syncServ = lo_server_new_multicast(lo_address_get_hostname(lo_syncAddr), lo_address_get_port(lo_syncAddr), oscParser_error);
    } else {
        lo_syncServ = lo_server_new(lo_address_get_port(lo_syncAddr), oscParser_error);
    }
    lo_server_add_method(lo_syncServ, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), 
            NULL, syncCallback, NULL);
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

    // convert port to integers for sending:
    std::string myIP = getMyIPaddress();
    int i_rxPort;
    fromString<int>(i_rxPort, lo_address_get_port(context->lo_rxAddr));

    context->running = true;

    // registerUser needs the context to be running (since it sends messages)
    spin.registerUser();

    // TIMEOUT in liblo was 10 ; zero here assumes that there is some other
    // processing or sleeping going on to manage CPU and mutex acquisition
    static const int TIMEOUT = 0;
    while (!spinBaseContext::signalStop)
    {
        lo_server_recv_noblock(context->lo_syncServ, TIMEOUT);
        lo_server_recv_noblock(context->lo_infoServ, TIMEOUT);
        lo_server_recv_noblock(context->lo_rxServ_, TIMEOUT);
        lo_server_recv_noblock(context->lo_tcpRxServer_, TIMEOUT);
        // do nothing (assume the app is doing updates - eg, in a draw loop)

        // TODO: this should sleep?
        usleep(10);

        // just send a ping so the server knows we are still here
        frameTick = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
        {
        	spin.NodeMessage(spin.getUserID().c_str(), "s", "ping", LO_ARGS_END);
        	/*
            if (spin.userNode.valid()) 
            {
            	spin.InfoMessage("/SPIN/__user__", "ssi", (char*) spin.userNode->id->s_name, myIP.c_str(), i_rxPort, LO_ARGS_END);
            }
            */
            lastTick = frameTick;
        }
    }

    std::cout << "Exiting spin client thread\n";
    context->running = false;

	spin.destroyScene();
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
	// For now, we just take anything received from TCP and forward it to the
	// regular OSC callback methods.
    if (std::string(path) == std::string("/SPIN/" +
                spinApp::Instance().getSceneID()))
        spinBaseContext::sceneCallback(path, types, argv, argc, (void*) data, (void*) user_data);
	else
		spinBaseContext::nodeCallback(path, types, argv, argc, (void*) data, (void*) user_data);

    return 1;
}

void spinClientContext::subscribe()
{
	// TODO: this should also be called whenever client gets a UserRefresh...
	// the idea is that if the server crashed and came back online, it sends a
	// userRefresh message and the client can re-subscribe.
	//
	// ie, UserRefresh should set doSubscribe_ to true.


	std::stringstream sstr;
	// convert to port number to string
	sstr << lo_server_get_port(lo_tcpRxServer_);

	lo_send(lo_serverTCPAddr, std::string("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), "ssss",
			"subscribe", spinApp::Instance().getUserID().c_str(),
			getMyIPaddress().c_str(),
			sstr.str().c_str());

    doSubscribe_ = false;
}

