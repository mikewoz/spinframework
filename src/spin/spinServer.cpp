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

#include <osgViewer/CompositeViewer>
#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osg/Timer>

#include "SceneManager.h"
#include "spinApp.h"
#include "spinServerContext.h"
#include "spinLog.h"
#include "config.h"

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	spinServerContext *server = new spinServerContext();

	// *************************************************************************
	// ARGUMENTS::

	osg::ArgumentParser arguments(&argc,argv);

	std::string sceneID = spinApp::Instance().getSceneID();

	std::string txHost = lo_address_get_hostname(server->lo_txAddr);
	std::string txPort = lo_address_get_port(server->lo_txAddr);
	std::string rxHost = lo_address_get_hostname(server->lo_rxAddrs_[0]);
	std::string rxPort = lo_address_get_port(server->lo_rxAddrs_[0]);
	std::string syncPort = lo_address_get_port(server->lo_syncAddr);

	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a server for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] <scene-to-load.xml>");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
	arguments.getApplicationUsage()->addCommandLineOption("--version", "Display the version number and exit.");

	arguments.getApplicationUsage()->addCommandLineOption("--scene-id <uniqueID>", "Specify a unique ID for this scene (Default: the local host name)"); 
    //FIXME: Printing current local host in help is no good for the man page generation
    //'" + sceneID + "')");
	arguments.getApplicationUsage()->addCommandLineOption("--tx-addr <hostname> <port>", "Set the transmission address where the server sends updates to (Default: " + txHost + " " + txPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("--rx-addr <hostname> <port>", "Set the receiving address for incoming OSC messages (Default: <local host name> " + rxPort + ")");
    // FIXME: rxHost (see comment above)
	arguments.getApplicationUsage()->addCommandLineOption("--sync-port <port>", "Set the port on which we send the sync timecode (Default: " + syncPort + ")");


	// PARSE ARGS:

	// if user request help or version write it out to cout and quit.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 0;
	}
    if (arguments.read("--version"))
    {
        std::cout << VERSION << std::endl;
        return 0;
    }
    // otherwise, try to start the server:
	osg::ArgumentParser::Parameter param_spinID(sceneID);
	arguments.read("--scene-id", param_spinID);
	spinApp::Instance().setSceneID(sceneID);

	while (arguments.read("--tx-addr", txHost, txPort)) {
		server->lo_txAddr = lo_address_new(txHost.c_str(), txPort.c_str());
	}

	bool passed_rxAddrs = false;
	while (arguments.read("--rx-addr", rxHost, rxPort)) {
		if (!passed_rxAddrs) server->lo_rxAddrs_.clear();
		server->lo_rxAddrs_.push_back(lo_address_new(rxHost.c_str(), rxPort.c_str()));
		passed_rxAddrs = true;
	}

	while (arguments.read("--sync-port", syncPort)) {
		server->lo_syncAddr = lo_address_new(txHost.c_str(), syncPort.c_str());
	}

	
	// any option left unread are converted into errors to write out later
	arguments.reportRemainingOptionsAsUnrecognized();

	
	// *************************************************************************
	// remaining arguments are assumed to be scene elements to be loaded (.xml)
	
	std::vector<std::string>::iterator arg;
	for (int i=1; i<argc; i++) //arg=arguments.begin(); arg!=arguments.end(); ++arg)
	{
		if (arguments[i][0]!='-')
		{
			// not an option so assume string is a filename
			spinApp::Instance().sceneManager->loadXML(arguments[i]);
		} else i++;
	}
	 	
	// any option left unread are converted into errors
	//arguments.reportRemainingOptionsAsUnrecognized();
	
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}

	// *************************************************************************
	// start spin:
	server->start();

	// *************************************************************************
	// send a userRefresh message:
    SCENE_MSG("s", "userRefresh");
	
	// *************************************************************************
	// loop:
    try {	
        while (server->isRunning())
        {
            sleep(1);
            // loop until a quit message is received (TODO)
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Got exception " << e.what() << std::endl;
        return 1;
    }

    usleep(100);
    std::cout << "spinserver exited normally." << std::endl;

    return 0;
}
