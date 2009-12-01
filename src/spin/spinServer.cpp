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
#include <osgDB/ReadFile>
#include <osg/Timer>

#include "spinContext.h"
#include "spinLog.h"



using namespace std;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{


	spinContext *spin = new spinContext(spinContext::SERVER_MODE);


	// *************************************************************************
	// ARGUMENTS::

	osg::ArgumentParser arguments(&argc,argv);

	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a server for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] <scene-to-load.xml>");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	arguments.getApplicationUsage()->addCommandLineOption("-id <uniqueID>", "Specify a unique ID for this scene.");
	arguments.getApplicationUsage()->addCommandLineOption("-txAddr <addr>", "Set the transmission address where the server sends update (Default is broadcast: " + spin->txAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-txPort <port>", "Set the transmission port where the server sends updates (Default: " + spin->txPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxAddr <addr>", "Set the receiving address for incoming OSC messages (Default: " + spin->rxAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxPort <port>", "Set the receiving port for incoming OSC messages (Default: " + spin->rxPort + ")");

	// PARSE ARGS:

	// if user request help write it out to cout.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 1;
	}
	osg::ArgumentParser::Parameter param_spinID(spin->id);
	arguments.read("-id", param_spinID);
	osg::ArgumentParser::Parameter param_txAddr(spin->txAddr);
	arguments.read("-txAddr", param_txAddr);
	osg::ArgumentParser::Parameter param_txPort(spin->txPort);
	arguments.read("-txPort", param_txPort);
	osg::ArgumentParser::Parameter param_rxAddr(spin->rxAddr);
	arguments.read("-rxAddr", param_rxAddr);
	osg::ArgumentParser::Parameter param_rxPort(spin->rxPort);
	arguments.read("-rxPort", param_rxPort);
	
	// any option left unread are converted into errors to write out later
	arguments.reportRemainingOptionsAsUnrecognized();

	
	// *************************************************************************
	// start spin:

	spin->start();
	
	// *************************************************************************
	// remaining arguments are assumed to be scene elements to be loaded (.xml)
	
	std::vector<std::string>::iterator arg;
	for (int i=1; i<argc; i++) //arg=arguments.begin(); arg!=arguments.end(); ++arg)
	{
		if (arguments[i][0]!='-')
		{
			// not an option so assume string is a filename
			spin->sceneManager->loadXML(arguments[i]);
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
	// loop:
	
	while (spin->isRunning())
	{
		sleep(1);
		// loop until a quit message is received (TODO)
	}


	return 0;
}
