// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
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
// ===================================================================

#include <string>
#include <iostream>

#include <osgViewer/CompositeViewer>
#include <osgDB/ReadFile>
#include <osg/Timer>


#include "asUtil.h"
#include "vessThreads.h"
#include "asCameraManager.h"

using namespace std;

extern pthread_mutex_t pthreadLock;



// *****************************************************************************
// ******************************* GLOBALS: ************************************
// *****************************************************************************

//extern lo_server_thread oscParser; // defined in oscParser.cpp

//asSceneManager *sceneManager;
//asMediaManager *mediaManager;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	// Now make sure we can load the libAudioscape library:
	osgDB::Registry *reg = osgDB::Registry::instance();
	osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit("libAudioscape"));

	std::cout <<"\nasViewer launching..." << std::endl;

	vessListener *vess = new vessListener();


    std::string viewerID = "default";
    std::string viewerAddr = "224.0.0.1";
    //std::string viewerAddr = getMyIPAddress();
    std::string viewerPort = "54322";
    std::string resolutionString = "720x480";


	// *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);

	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a viewer for VESS.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	arguments.getApplicationUsage()->addCommandLineOption("-rxID <uniqueID>", "Specify the VESS scene ID to listen to (Default: '" + vess->id + "')");
	arguments.getApplicationUsage()->addCommandLineOption("-rxAddr <addr>", "Set the receiving address for incoming OSC messages (Default: " + vess->rxAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxPort <port>", "Set the receiving port for incoming OSC messages (Default: " + vess->rxPort + ")");

	arguments.getApplicationUsage()->addCommandLineOption("-viewerID <uniqueID>", "Specify a unique ID for the embedded asViewer (Default: '" + viewerID + "')");
	arguments.getApplicationUsage()->addCommandLineOption("-viewerAddr <addr>", "Specify the address for camera/view events (default: " + viewerAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-viewerPort <port>", "Specify the port for camera/view events (default: " + viewerPort + ")");

	arguments.getApplicationUsage()->addCommandLineOption("-resolution <WIDTHxHEIGHT>", "Specify the resolution of the viewing window, either as a ratio (eg, 0.5x0.5) or in pixel values. (Default: " + resolutionString + ")");


	// *************************************************************************
	// PARSE ARGS:

	// if user request help write it out to cout.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 1;
	}

	osg::ArgumentParser::Parameter param_rxID(vess->id);
	arguments.read("-rxID", param_rxID);
	osg::ArgumentParser::Parameter param_rxAddr(vess->rxAddr);
	arguments.read("-rxAddr", param_rxAddr);
	osg::ArgumentParser::Parameter param_rxPort(vess->rxPort);
	arguments.read("-rxPort", param_rxPort);

	osg::ArgumentParser::Parameter param_viewerID(viewerID);
	arguments.read("-viewerID", param_viewerID);
	osg::ArgumentParser::Parameter param_viewerAddr(viewerAddr);
	arguments.read("-viewerAddr", param_viewerAddr);
	osg::ArgumentParser::Parameter param_viewerPort(viewerPort);
	arguments.read("-viewerPort", param_viewerPort);
	osg::ArgumentParser::Parameter param_resolution(resolutionString);
	arguments.read("-resolution", param_resolution);


	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);


	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
	//osgViewer::CompositeViewer viewer(arguments);


	// set the threading model for the viewer:
	/*
	while (arguments.read("-s")) { viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded); }
	while (arguments.read("-g")) { viewer.setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext); }
	while (arguments.read("-d")) { viewer.setThreadingModel(osgViewer::Viewer::DrawThreadPerContext); }
	while (arguments.read("-c")) { viewer.setThreadingModel(osgViewer::Viewer::CullThreadPerCameraDrawThreadPerContext); }
	*/

	viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
	
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
	// start the vessListener thread:

	vess->start();
	vess->sceneManager->isGraphical = true;



	// *************************************************************************
	// initialize the cameraManager:

	asCameraManager *cameraManager = new asCameraManager(viewerID, viewer, viewerAddr, viewerPort);
	cameraManager->createCamera("default"); // must create at least one camera to start

	cameraManager->setSceneData(vess->sceneManager->rootNode.get());

	cameraManager->init(); // must call this before viewer.realize()

	cameraManager->setResolution(resolutionString);

	//cameraManager->debugPrint();


	// *************************************************************************
	// set up any initial scene elements:

	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		vess->sceneManager->rootNode->addChild(argScene.get());
	}


	// *************************************************************************
	// start threads:
	viewer.realize();
	cameraManager->refreshWindows();

	osg::Timer_t lastTick = osg::Timer::instance()->tick();
	osg::Timer_t frameTick = lastTick;

	// convert ports to integers for sending:
	int i_viewerPort;
	fromString<int>(i_viewerPort, viewerPort);

	// program loop:
	while( !viewer.done() && vess->isRunning() )
	{
		frameTick = osg::Timer::instance()->tick();
		if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
		{
			cameraManager->sendCameraList(vess->lo_infoAddr, vess->lo_infoServ);
			lastTick = frameTick;
		}

		// We now have to go through all the nodes, and check if we need to update the
		// graph. Note: this cannot be done as a callback in a traversal - dangerous.
		// In the callback, we have simply flagged what needs to be done (eg, set the
		// newParent symbol).
		pthread_mutex_lock(&pthreadLock);
		vess->sceneManager->updateGraph();
		pthread_mutex_unlock(&pthreadLock);
		
		
		// same goes for the cameraManager:
		pthread_mutex_lock(&pthreadLock);
		cameraManager->update();
		pthread_mutex_unlock(&pthreadLock);
		
		pthread_mutex_lock(&pthreadLock);
		viewer.frame();
		pthread_mutex_unlock(&pthreadLock);
	}


	return 0;
}
