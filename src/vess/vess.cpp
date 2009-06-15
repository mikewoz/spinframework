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

#include "asUtil.h"
#include "asCameraManager.h"
#include "asSceneManager.h"
#include "asMediaManager.h"



using namespace std;


// *****************************************************************************
// ******************************* GLOBALS: ************************************
// *****************************************************************************

asSceneManager *sceneManager;
asMediaManager *mediaManager;
asCameraManager *cameraManager;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	int i;
	
	string tmpStr = getenv("AS_INFOPORT");
	string infoPort, txAddr;
	if (infoPort.empty())
	{
		infoPort = "54320";
		txAddr = "224.0.0.1";
	} else {
		infoPort = tmpStr.substr(0,tmpStr.find(":"));
		txAddr = tmpStr.substr(tmpStr.find(":")+1);
	}
	std::cout << "\nVESS launching... info channel multicast on " << txAddr << ", port: " << infoPort << std::endl;
	
	
	// Now make sure we can load the libAudioscape library:
	osgDB::Registry *reg = osgDB::Registry::instance();
	osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit("libAudioscape"));
	

	// defaults:
	
	std::string vessID = "default";
	std::string txPort = "54323";
	//std::string rxAddr = getMyIPAddress();
	std::string rxAddr = "224.0.0.1";
	std::string rxPort = "54324";

	bool withViewer = true;
	std::string viewerID = "default";
	std::string viewerAddr = "224.0.0.1";
	//std::string viewerAddr = getMyIPAddress();
	std::string viewerPort = "54322";
	//std::string resolutionString = "720x480";
	std::string resolutionString = "800x600";
	
	std::string dataPath = "~/Documents/Audioscape-Data"; // this isn't used anymore

	
	// *************************************************************************
	
	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the Virtual Environment State Server.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
	
	arguments.getApplicationUsage()->addCommandLineOption("-id <uniqueID>", "Specify a unique ID for this vess scene.");
	arguments.getApplicationUsage()->addCommandLineOption("-txAddr <addr>", "Set the transmission address where vess sends update (Default is broadcast: " + txAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-txPort <port>", "Set the transmission port where vess sends updates (Default: " + txPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxAddr <addr>", "Set the receiving address for incoming OSC messages (Default: " + rxAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-rxPort <port>", "Set the receiving port for incoming OSC messages (Default: " + rxPort + ")");

	
	arguments.getApplicationUsage()->addCommandLineOption("-noViewer", "Disable the viewer (run as a server daemon)");
	arguments.getApplicationUsage()->addCommandLineOption("-viewerID <uniqueID>", "Specify a unique ID for the embedded asViewer");
	arguments.getApplicationUsage()->addCommandLineOption("-viewerAddr <addr>", "Specify the address for camera/view events (default: " + viewerAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-viewerPort <port>", "Specify the port for camera/view events (default: " + viewerPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-resolution <WIDTHxHEIGHT>", "Specify the resolution of the viewing window, either as a ratio (eg, 0.5x0.5) or in pixel values. (Default: " + resolutionString + ")");

	arguments.getApplicationUsage()->addCommandLineOption("-dataPath <path>", "DEPRECATED: Provides a path to find media (Default: " + dataPath + ")");
	
	
	
	
	
	// *************************************************************************
	// PARSE ARGS:
	
	// if user request help write it out to cout.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 1;
	}
	
	
	osg::ArgumentParser::Parameter param_vessID(vessID);
	arguments.read("-id", param_vessID);
	osg::ArgumentParser::Parameter param_txAddr(txAddr);
	arguments.read("-txAddr", param_txAddr);
	osg::ArgumentParser::Parameter param_txPort(txPort);
	arguments.read("-txPort", param_txPort);
	osg::ArgumentParser::Parameter param_rxAddr(rxAddr);
	arguments.read("-rxAddr", param_rxAddr);
	osg::ArgumentParser::Parameter param_rxPort(rxPort);
	arguments.read("-rxPort", param_rxPort);

	if (arguments.read("-noViewer") || arguments.read("--noViewer")) withViewer = false;
	osg::ArgumentParser::Parameter param_viewerID(viewerID);
	arguments.read("-viewerID", param_viewerID);
	osg::ArgumentParser::Parameter param_viewerAddr(viewerAddr);
	arguments.read("-viewerAddr", param_viewerAddr);
	osg::ArgumentParser::Parameter param_viewerPort(viewerPort);
	arguments.read("-viewerPort", param_viewerPort);
	osg::ArgumentParser::Parameter param_resolution(resolutionString);
	arguments.read("-resolution", param_resolution);
	
	osg::ArgumentParser::Parameter param_dataPath(dataPath);
	arguments.read("-dataPath", param_dataPath);


	
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
	// set up networking:
	
	lo_address lo_infoAddr = lo_address_new(txAddr.c_str(), infoPort.c_str());	
	

	// *************************************************************************	
	// initialize the sceneManager and mediaManager:
	sceneManager = new asSceneManager(vessID, rxAddr, rxPort);	
	sceneManager->setTXaddress(txAddr, txPort);

	mediaManager = new asMediaManager(dataPath);


	// *************************************************************************
	// initialize the cameraManager:
	if (withViewer)
	{
		asCameraManager *cameraManager = new asCameraManager(viewerID, viewer, viewerAddr, viewerPort);
		cameraManager->setResolution(resolutionString);
	
		cameraManager->createCamera("default", 0);
		//cameraManager->createCamera("foo", 1); // create multiple windows just by giving different window IDs
	
		/*
		// dome test:
		vector<float> vect;
		vect.push_back(0.45);
		vect.push_back(0.45);
		cameraManager->getCamera("default")->setDistortion("DOME", vect);
		*/
		
		cameraManager->setSceneData(sceneManager->rootNode.get());
		cameraManager->init(); // must call this before viewer.realize()
	
		//cameraManager->debugPrint();
	}
	
	// *************************************************************************
	// set up any initial scene elements:
	
	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		sceneManager->rootNode->addChild(argScene.get());	
	}

	// *************************************************************************
	
	string myIP = getMyIPaddress();
	osg::Timer_t lastTick = osg::Timer::instance()->tick();
	osg::Timer_t frameTick = lastTick;
	//osg::Timer::instance()->tick();
	
	// convert ports to integers for sending:
	int i_rxPort, i_txPort, i_viewerPort;
	fromString<int>(i_rxPort, rxPort);
	fromString<int>(i_txPort, txPort);
	fromString<int>(i_viewerPort, viewerPort);
	
	
	if (withViewer)
	{
	
		// *************************************************************************
		// start threads:
		viewer.realize();
		
		// program loop:
		while( !viewer.done() )
		{
			frameTick = osg::Timer::instance()->tick();
			if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
			{		
				//std::cout << "periodic broadcast on port " << infoPort << std::endl;
				lo_send_from(lo_infoAddr, sceneManager->txServ, LO_TT_IMMEDIATE, "/ping/vess", "ssisi", vessID.c_str(), myIP.c_str(), i_rxPort, txAddr.c_str(), i_txPort);
				lo_send_from(lo_infoAddr, sceneManager->txServ, LO_TT_IMMEDIATE, "/ping/asViewer", "ssi", viewerID.c_str(), viewerAddr.c_str(), i_viewerPort);
				cameraManager->sendCameraList(lo_infoAddr, sceneManager->txServ);
				lastTick = frameTick;
			}
	
			// We now have to go through all the nodes, and check if we need to update the
			// graph. Note: this cannot be done as a callback in a traversal - dangerous.
			// In the callback, we have simply flagged what needs to be done (eg, set the
			// newParent symbol).
			sceneManager->updateGraph();
	
			// same goes for the cameraManager:
			cameraManager->update();
			
	
			
			viewer.frame();
		}
	}
	
	else {
		
		asSceneUpdateVisitor visitor;
		while (1)
		{
			frameTick = osg::Timer::instance()->tick();
			if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
			{		
				lo_send_from(lo_infoAddr, sceneManager->txServ, LO_TT_IMMEDIATE, "/ping/vess", "ssisi", vessID.c_str(), myIP.c_str(), i_rxPort, txAddr.c_str(), i_txPort);
				lastTick = frameTick;
			}
			visitor.apply(*(sceneManager->rootNode.get()));
			sceneManager->updateGraph();
		}
		
		
	}
	

	sceneManager->clear();
	
	return 0;
}
