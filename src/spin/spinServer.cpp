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


#include "scenemanager.h"
#include "spinapp.h"
#include "spinservercontext.h"
#include "spinlog.h"
#include "config.h"

#ifdef WITH_POCO
#include "Poco/Net/HTTPServer.h"
#include "Poco/Net/HTTPRequestHandler.h"
#include "Poco/Net/HTTPRequestHandlerFactory.h"
#include "Poco/Net/HTTPServerParams.h"
#include "Poco/Net/HTTPServerRequest.h"
#include "Poco/Net/HTTPServerResponse.h"
#include "Poco/Net/HTTPServerParams.h"
#include "Poco/Net/HTMLForm.h"
#include "Poco/Net/PartHandler.h"
#include "Poco/Net/MessageHeader.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/CountingStream.h"
#include "Poco/NullStream.h"
#include "Poco/StreamCopier.h"
#include "Poco/Exception.h"
#include "Poco/Util/ServerApplication.h"
#include "Poco/Util/Option.h"
#include "Poco/Util/OptionSet.h"
#include "Poco/Util/HelpFormatter.h"
#endif

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	using namespace spin;
	spinServerContext server;
    
	// *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinframework/args and override argc and argv with those:
    std::vector<char*> newArgs = getUserArgs();
    if ((argc==1) && (newArgs.size() > 1))
    {
        newArgs.insert(newArgs.begin(), argv[0]);
        argc = (int)newArgs.size()-1;
        argv = &newArgs[0];
    }
    
	// *************************************************************************
	// PARSE ARGUMENTS::

	osg::ArgumentParser arguments(&argc,argv);
	
    // set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a server for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] <scene-to-load.xml>");
    
    // add generic spin arguments (for address/port setup, scene-id, etc) 
    server.addCommandLineOptions(&arguments);

    // arguments specific to spinserver:
    arguments.getApplicationUsage()->addCommandLineOption("--disable-discovery", "Disable multicast discovery services.");

	// PARSE ARGS:

    // parse generic args:
    if (!server.parseCommandLineOptions(&arguments))
        return 0;

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
			spinApp::Instance().sceneManager_->loadXML(arguments[i]);
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
	server.start();


	// *************************************************************************
    // launch webserver:
    
#ifdef WITH_POCO
    //unsigned short port = 9980;
			
    // set-up a server socket
	//Poco::Net::ServerSocket svs(port);
#endif    

	// *************************************************************************
	// send a userRefresh message:
    spinApp::Instance().BroadcastSceneMessage("s", "userRefresh", SPIN_ARGS_END);
	
	// *************************************************************************
	// loop:
    std::cout << "\nspinserver is READY" << std::endl;
    try {	
        while (server.isRunning())
        {
            timespec nap;
            nap.tv_sec = 0;
            nap.tv_nsec = 1e9;
            nanosleep(&nap, NULL);
            //sleep(1);
            // loop until a quit message is received (TODO)
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Got exception " << e.what() << std::endl;
        return 1;
    }

    timespec nap;
    nap.tv_sec = 0;
    nap.tv_nsec = 1e5;
    nanosleep(&nap, NULL);
    //usleep(100);
    std::cout << "spinserver exited normally." << std::endl;

    return 0;
}
