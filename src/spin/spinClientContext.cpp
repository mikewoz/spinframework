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


spinClientContext::spinClientContext() : doSubscribe_(true)
{
	// Important: fist thing to do is set the context mode (client vs server)
    mode = CLIENT_MODE;

	// Next, tell spinApp that this is the current context running:
    spinApp &spin = spinApp::Instance();
    spin.setContext(this);
    lo_server_add_method(lo_infoServ, NULL, NULL, infoCallback, this);
    lo_server_add_method(lo_tcpRxServer_, NULL, NULL, tcpCallback, this);

    lo_rxAddr = lo_address_new("226.0.0.1", "54323");
    lo_txAddr = lo_address_new("226.0.0.1", "54324");
}

spinClientContext::~spinClientContext()
{}

bool spinClientContext::start()
{
	return startThread(&spinClientThread);
}

void *spinClientContext::spinClientThread(void *arg)
{
	spinClientContext *context = (spinClientContext*)(arg);
    spinApp &spin = spinApp::Instance();
	spin.createScene();
    spin.registerUser();

    // register our special scene callback:
    lo_server_add_method(spin.sceneManager->rxServ, ("/SPIN/" + spin.getSceneID()).c_str(), NULL, sceneCallback, NULL);

    // sync (timecode) receiver:
    if (isMulticastAddress(lo_address_get_hostname(spin.getContext()->lo_syncAddr)))
    {
    	context->lo_syncServ = lo_server_new_multicast(lo_address_get_hostname(context->lo_syncAddr), lo_address_get_port(context->lo_syncAddr), oscParser_error);
    } else {
    	context->lo_syncServ = lo_server_new(lo_address_get_port(context->lo_syncAddr), oscParser_error);
    }
    lo_server_add_method(context->lo_syncServ, std::string("/SPIN/" + spin.getSceneID()).c_str(), NULL, syncCallback, &spin);

    osg::Timer_t lastTick = osg::Timer::instance()->tick();
    osg::Timer_t frameTick = lastTick;

    // convert port to integers for sending:
    std::string myIP = getMyIPaddress();
    int i_rxPort;
    fromString<int>(i_rxPort, lo_address_get_port(spin.getContext()->lo_rxAddr));

    context->running = true;
    static const int TIMEOUT = 10;
    while (!spinBaseContext::signalStop)
    {
        lo_server_recv_noblock(context->lo_syncServ, TIMEOUT); 
        lo_server_recv_noblock(context->lo_infoServ, TIMEOUT); // was 250 ms before
        lo_server_recv_noblock(spin.sceneManager->rxServ, TIMEOUT); 
        lo_server_recv_noblock(context->lo_tcpRxServer_, TIMEOUT); 
        // do nothing (assume the app is doing updates - eg, in a draw loop)

        // just send a ping so the server knows we are still here
        frameTick = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
        {
            if (spin.userNode.valid()) 
                spin.InfoMessage("/ping/user", "ssi", (char*) spin.userNode->id->s_name, myIP.c_str(), i_rxPort, LO_ARGS_END);
            lastTick = frameTick;
        }
    }

    std::cout << "Exitting spin client thread\n";
    context->running = false;

    // clean up:
    lo_server_free(context->lo_syncServ);
    pthread_exit(NULL);
}

int spinClientContext::sceneCallback(const char * /*path*/, const char *types, lo_arg **argv, 
        int argc, lo_message /*msg*/, void * /*user_data*/)
{
    spinApp &spin = spinApp::Instance();

    // make sure there is at least one argument (ie, a method to call):
    if (!argc) return 1;

    // get the method (argv[0]):
    std::string theMethod;
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else return 1;

    // bundle all other arguments
    std::vector<float> floatArgs;
    std::vector<const char*> stringArgs;
    for (int i=1; i<argc; i++)
    {
        if (lo_is_numerical_type((lo_type)types[i]))
        {
            floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
        } else {
            stringArgs.push_back( (const char*) argv[i] );
        }
    }

    // now, here are some special messages that we need to look for:

    if (theMethod=="userRefresh")
    {
        if (spin.userNode.valid()) 
            spin.SceneMessage("sss", "createNode", spin.userNode->id->s_name, "UserNode", LO_ARGS_END);
    }

    return 1;
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
        ////std::cout << "MASTERTICK = " << masterTick << std::endl;

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
int spinClientContext::tcpCallback(const char * /*path*/, const char * /*types*/, 
        lo_arg ** argv, int argc, void * /*data*/, void * user_data)
{
    // TODO: add some methods!
    return 1;
}

void spinClientContext::subscribe()
{
    // FIXME: can only subscribe with a valid user name
    if (spinApp::Instance().userNode.valid()) 
    {
        std::stringstream sstr;
        // convert to port number to string
        sstr << lo_server_get_port(lo_tcpRxServer_);

        lo_send(lo_serverTCPAddr, "/SPIN/__client__", "ssss",
                "subscribe", spinApp::Instance().userNode->getID().c_str(), getMyIPaddress().c_str(),
                sstr.str().c_str());
    }
    doSubscribe_ = false;
}

