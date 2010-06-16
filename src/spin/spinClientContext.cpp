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

#include "spinClientContext.h"
#include "spinApp.h"
#include "SceneManager.h"



spinClientContext::spinClientContext()
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

}

bool spinClientContext::start()
{
	return startThread(&spinClientThread);
}



// *****************************************************************************
// *****************************************************************************
// *****************************************************************************


void *spinClientContext::spinClientThread(void *arg)
{
	spinClientContext *thiss = (spinClientContext*)(arg);
    spinApp &spin = spinApp::Instance();

    spin.sceneManager = new SceneManager(spin.getSceneID(), lo_address_get_hostname(spin.getContext()->lo_rxAddr), lo_address_get_port(spin.getContext()->lo_rxAddr));

    // register our special scene callback:
    lo_server_thread_add_method(spin.sceneManager->rxServ, ("/SPIN/" + spin.getSceneID()).c_str(), NULL, sceneCallback, NULL);


    // sync (timecode) receiver:
    if (isMulticastAddress(lo_address_get_hostname(spin.getContext()->lo_syncAddr)))
    {
    	thiss->lo_syncServ = lo_server_thread_new_multicast(lo_address_get_hostname(spin.getContext()->lo_syncAddr), lo_address_get_port(spin.getContext()->lo_syncAddr), oscParser_error);
    } else {
    	thiss->lo_syncServ = lo_server_thread_new(lo_address_get_port(spin.getContext()->lo_syncAddr), oscParser_error);
    }
    lo_server_thread_add_method(thiss->lo_syncServ, std::string("/SPIN/"+spin.getSceneID()).c_str(), NULL, syncCallback, &spin);
    lo_server_thread_start(thiss->lo_syncServ);


    osg::Timer_t lastTick = osg::Timer::instance()->tick();
    osg::Timer_t frameTick = lastTick;

    // convert port to integers for sending:
    std::string myIP = getMyIPaddress();
    int i_rxPort;
    fromString<int>(i_rxPort, lo_address_get_port(spin.getContext()->lo_rxAddr));

    thiss->running = true;
    while (!spinBaseContext::signalStop)
    {


        usleep(1000000 * 0.25); // 1/4 second sleep

        // do nothing (assume the app is doing updates - eg, in a draw loop)

        // just send a ping so the server knows we are still here
        frameTick = osg::Timer::instance()->tick();
        if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
        {
            if (spin.userNode.valid()) spin.InfoMessage("/ping/user", "ssi", (char*) spin.userNode->id->s_name, myIP.c_str(), i_rxPort, LO_ARGS_END);
            lastTick = frameTick;
        }

    }
    thiss->running = false;

    // clean up:
    lo_server_thread_stop(thiss->lo_syncServ);
    lo_server_thread_free(thiss->lo_syncServ);
    delete spin.sceneManager;

    pthread_exit(NULL);
}


int spinClientContext::sceneCallback(const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *user_data)
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
        if (spin.userNode.valid()) spin.SceneMessage("sss", "createNode", spin.userNode->id->s_name, "UserNode", LO_ARGS_END);
    }

    return 1;
}


int spinClientContext::syncCallback(const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *user_data)
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
