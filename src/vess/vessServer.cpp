// =============================================================================
//                    .___.__                                                 //
//   _____   __ __  __| _/|__| ____  ______ ____ _____  ______   ____         //
//   \__  \ |  |  \/ __ | |  |/  _ \/  ___// ___\\__  \ \____ \_/ __ \        //
//    / __ \|  |  / /_/ | |  (  <_> )___ \\  \___ / __ \|  |_> >  ___/        //
//   (____  /____/\____ | |__|\____/____  >\___  >____  /   __/ \___  >       //
//        \/           \/               \/     \/     \/|__|        \/  .ORG  //
//                                                                            //
// =============================================================================
// The Audioscape Project :: www.audioscape.org
// Copyright (c) 2009
//
//
// Organizations:
// - McGill University, Shared Reality Lab (SRE) :: www.cim.mcgill.ca/sre
// - La Societe des Arts Technologiques (SAT) :: www.sat.qc.ca
// - Universite de Montreal :: www.umontreal.ca
//
// Development Team:
// - Mike Wozniewski (www.mikewoz.com): Head Developer, Researcher
// - Zack Settel (www.sheefa.net/zack): Conception, Research, Artist, Programmer
// - Jeremy Cooperstock (cim.mcgill.ca/~jer): Project Coordinator
// - Sylvain Cormier: Programmer, Tester
// - Jean-Michel Dumas: Assistant, Programmer
// - Pierre-Olivier Charlebois: Programmer
//
// Funding by / Subventionne par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//
// =============================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// =============================================================================

#include <string>
#include <iostream>

#include <osgViewer/CompositeViewer>
#include <osgDB/ReadFile>
#include <osg/Timer>

#include "vessThreads.h"



using namespace std;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{


	vessMaster *vess = new vessMaster();


	// *************************************************************************
	// ARGUMENTS::

	osg::ArgumentParser arguments(&argc,argv);

	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the Virtual Environment State Server.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	arguments.getApplicationUsage()->addCommandLineOption("-id <uniqueID>", "Specify a unique ID for this vess scene.");
	arguments.getApplicationUsage()->addCommandLineOption("-txAddr <addr>", "Set the transmission address where vess sends update (Default is broadcast: " + vess->txAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-txPort <port>", "Set the transmission port where vess sends updates (Default: " + vess->txPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxAddr <addr>", "Set the receiving address for incoming OSC messages (Default: " + vess->rxAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxPort <port>", "Set the receiving port for incoming OSC messages (Default: " + vess->rxPort + ")");

	// PARSE ARGS:

	// if user request help write it out to cout.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 1;
	}
	osg::ArgumentParser::Parameter param_vessID(vess->id);
	arguments.read("-id", param_vessID);
	osg::ArgumentParser::Parameter param_txAddr(vess->txAddr);
	arguments.read("-txAddr", param_txAddr);
	osg::ArgumentParser::Parameter param_txPort(vess->txPort);
	arguments.read("-txPort", param_txPort);
	osg::ArgumentParser::Parameter param_rxAddr(vess->rxAddr);
	arguments.read("-rxAddr", param_rxAddr);
	osg::ArgumentParser::Parameter param_rxPort(vess->rxPort);
	arguments.read("-rxPort", param_rxPort);

	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);

	// any option left unread are converted into errors to write out later.
	arguments.reportRemainingOptionsAsUnrecognized();

	// report any errors if they have occured when parsing the program aguments.
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}


	// *************************************************************************
	// set up any initial scene elements:

	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		vess->sceneManager->rootNode->addChild(argScene.get());
	}


	vess->start();

	
	while (vess->isRunning())
	{
		sleep(1);
		// loop until a quit message is received (TODO)
	}
	

	vess->sceneManager->clear();

	return 0;
}
