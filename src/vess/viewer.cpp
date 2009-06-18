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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <stdlib.h>
#include <iostream>

#include <osgViewer/CompositeViewer>
#include <osgDB/ReadFile>

#include "asCameraManager.h"

using namespace std;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	int i;	
	
	// this should be automatically discovered using infoPort
	std::string port = "54322";
	std::string resolutionString = "720x480";
	
	// *************************************************************************
	
	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a OSG viewer that listens to OSC messages for camera control.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	arguments.getApplicationUsage()->addCommandLineOption("-port <port>", "Specify the port on which to listen to OSC messages (default: " + port + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-resolution <WIDTHxHEIGHT>", "Specify the resolution of the viewing window (Default: " + resolutionString + ")");


	// *************************************************************************
	// PARSE ARGS:
	
	// if user request help write it out to cout.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 1;
	}

	osg::ArgumentParser::Parameter param_port(port);
	arguments.read("-port", param_port);

	osg::ArgumentParser::Parameter param_resolution(resolutionString);
	arguments.read("-resolution", param_resolution);
	
	
	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);



	
	// *************************************************************************
	
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)
	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);

	
	// set the threading model for the viewer:
	/*
	while (arguments.read("-s")) { viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded); }
	while (arguments.read("-g")) { viewer.setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext); }
	while (arguments.read("-d")) { viewer.setThreadingModel(osgViewer::Viewer::DrawThreadPerContext); }
	while (arguments.read("-c")) { viewer.setThreadingModel(osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext); }
	*/
	
	
	// *************************************************************************
	// any option left unread are converted into errors to write out later.
	arguments.reportRemainingOptionsAsUnrecognized();
	
	// report any errors if they have occured when parsing the program aguments.
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}

	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());
	
	
	// *************************************************************************
	// set up scene and cameras:
	
	
	asCameraManager *cameraManager = new asCameraManager("default", viewer, port);
	cameraManager->setResolution(resolutionString);
	cameraManager->createCamera("default"); // must create at least one camera to start
	//cameraManager->createCamera("foo");
	
	//cameraManager->createCamera("blah");
	

	//cameraManager->getCamera("defaultCam")->enableDome(true); // turn dome on
	
	cameraManager->update(); // must run this at least once before viewer.realize()


	
	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		cameraManager->setSceneData(argScene.get());	
	} else std::cout << "No initial model defined. Waiting for OSC messages..." << std::endl;
	
	
	cameraManager->debugPrint();

	

	
	// *************************************************************************

	// start threads:
	viewer.realize();

	// program loop:
	while( !viewer.done() )
	{
		viewer.frame();

		cameraManager->update();
	}

	return 0;
}
