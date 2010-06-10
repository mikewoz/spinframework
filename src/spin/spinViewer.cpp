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
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/GraphicsContext>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>
#include <osg/Timer>

#include "ViewerManipulator.h"
#include "spinUtil.h"
#include "spinContext.h"
#include "osgUtil.h"
#include "GroupNode.h"
#include "SceneManager.h"
#include "ShapeNode.h"

extern pthread_mutex_t pthreadLock;


// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	std::cout <<"\nspinViewer launching..." << std::endl;

	//spinContext spin = new spinContext(spinContext::LISTENER_MODE);
	// Singleton instance:
	spinContext &spin = spinContext::Instance();
	spin.setMode(spinContext::LISTENER_MODE);

	std::string id = getHostname();
	
	bool picker = false;
	bool mover = true;
	
	bool fullscreen = false;
	bool hideCursor=false;

	double maxFrameRate = -1;
	
	int x=50;
	int y=50;
	int width=640;
	int height=480;
	int screen=-1;
	
	std::string redirectAddr, redirectPort;

	std::string rxHost = lo_address_get_hostname(spin.lo_rxAddr);
	std::string rxPort = lo_address_get_port(spin.lo_rxAddr);
	std::string syncPort = lo_address_get_port(spin.lo_rxAddr);


	// *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a 3D viewer for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	arguments.getApplicationUsage()->addCommandLineOption("-id <uniqueID>", "Specify an ID for this viewer (Default is hostname: '" + id + "')");

	arguments.getApplicationUsage()->addCommandLineOption("-sceneID <uniqueID>", "Specify the scene ID to listen to (Default: '" + spin.id + "')");
	arguments.getApplicationUsage()->addCommandLineOption("-serverAddr <host> <port>", "Set the receiving address for incoming OSC messages (Default: " + rxHost + " " + rxPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-serverSync <port>", "Set the receiving port for timecode sync (Default: " + syncPort + ")");

	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
    arguments.getApplicationUsage()->addCommandLineOption("--hideCursor", "Hide the mouse cursor");
	arguments.getApplicationUsage()->addCommandLineOption("--window <x y w h>", "Set the position (x,y) and size (w,h) of the viewer window (Default: 50 50 640 480)");
	arguments.getApplicationUsage()->addCommandLineOption("--screen <num>", "Screen number to display on (Default: ALLSCREENS)");
	arguments.getApplicationUsage()->addCommandLineOption("--framerate <num>", "Set the maximum framerate (Default: not limited)");

	
	arguments.getApplicationUsage()->addCommandLineOption("--disabled", "Disable camera controls for this user");
	arguments.getApplicationUsage()->addCommandLineOption("--picker", "Enable the mouse picker, and send events to the server");
	arguments.getApplicationUsage()->addCommandLineOption("--redirection <host> <port>", "Redirect events to the specified address/port instead of the SPIN server");


	// *************************************************************************
	// PARSE ARGS:

	// if user request help write it out to cout.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 1;
	}

	osg::ArgumentParser::Parameter param_id(id);
	arguments.read("-id", param_id);
	osg::ArgumentParser::Parameter param_spinID(spin.id);
	arguments.read("-sceneID", param_spinID);

	while (arguments.read("-serverAddr", rxHost, rxPort)) {
		spin.lo_rxAddr = lo_address_new(rxHost.c_str(), rxPort.c_str());
	}
	while (arguments.read("-syncPort", syncPort)) {
		spin.lo_syncAddr = lo_address_new(rxHost.c_str(), syncPort.c_str());
	}

	if (arguments.read("--fullscreen")) fullscreen=true;
    if (arguments.read("--hideCursor")) hideCursor=true;
	while (arguments.read("--window",x,y,width,height)) {}
	while (arguments.read("--screen",screen)) {}
	while (arguments.read("--framerate",maxFrameRate)) {}

	
	if (arguments.read("--disabled")) mover=false;
	if (arguments.read("--picker")) picker=true;
	while (arguments.read("--redirection",redirectAddr,redirectPort)) {}

	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);



	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
	viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);

	viewer.getUsage(*arguments.getApplicationUsage());


	// *************************************************************************
	// start the listener thread:

	if (!spin.start())
	{
        std::cout << "ERROR: could not start SPIN listener thread" << std::endl;
        exit(1);
	}

	spin.sceneManager->setGraphical(true);

	spin.registerUser(id.c_str());
		

	// *************************************************************************
	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());

    // *************************************************************************
    // set up initial view:
    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    viewer.addView(view.get());

    view->getCamera()->setClearColor(osg::Vec4(0.0,0.0,0.0,0.0));

    if (fullscreen)
    {
    	if (screen<0) view->setUpViewAcrossAllScreens();
    	else view->setUpViewOnSingleScreen(screen);
    } else {
    	if (screen<0) view->setUpViewInWindow(x,y,width,height);
    	else view->setUpViewInWindow(x,y,width,height,screen);
    }


    osgViewer::ViewerBase::Windows windows;
    osgViewer::ViewerBase::Windows::iterator wIter;
    viewer.getWindows(windows);
    for (wIter=windows.begin(); wIter!=windows.end(); wIter++)
    {
    	(*wIter)->setWindowName("spinViewer");
		if (hideCursor) (*wIter)->useCursor(false);
    }

    view->setSceneData(spin.sceneManager->rootNode.get());

	view->addEventHandler(new osgViewer::StatsHandler);
	view->addEventHandler(new osgViewer::ThreadingHandler);
	view->addEventHandler(new osgViewer::WindowSizeHandler);


	view->addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	view->addEventHandler( new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()) );

	//view->setLightingMode(osg::View::NO_LIGHT);
	view->setLightingMode(osg::View::HEADLIGHT);
	//view->setLightingMode(osg::View::SKY_LIGHT);


	// *************************************************************************
	// create a camera manipulator

	osg::ref_ptr<ViewerManipulator> manipulator = new ViewerManipulator();
	manipulator->setPicker(picker);
	manipulator->setMover(mover);
	if (!redirectAddr.empty() && !redirectPort.empty())
		manipulator->setRedirection(redirectAddr, redirectPort);

	view->setCameraManipulator(manipulator.get());
	

	// *************************************************************************
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
		spin.sceneManager->worldNode->addChild(argScene.get());
	}

	// *************************************************************************
	// start threads:
	viewer.realize();

	// ask for refresh:
	spin.SceneMessage("s", "refresh", LO_ARGS_END);

	
	double minFrameTime = 1.0 / maxFrameRate;
	
	// program loop:
	while( !viewer.done() )
	{
		//std::cout << "frame: " << view->getFrameStamp()->getSimulationTime() << std::endl;
		
		if (spin.isRunning())
		{
			osg::Timer_t startFrameTick = osg::Timer::instance()->tick();
			
			pthread_mutex_lock(&pthreadLock);
			spin.sceneManager->update();
			pthread_mutex_unlock(&pthreadLock);
	
			pthread_mutex_lock(&pthreadLock);
			viewer.frame();
			pthread_mutex_unlock(&pthreadLock);

			if (maxFrameRate>0)
			{
				// work out if we need to force a sleep to hold back the frame rate
				osg::Timer_t endFrameTick = osg::Timer::instance()->tick();
				double frameTime = osg::Timer::instance()->delta_s(startFrameTick, endFrameTick);
				if (frameTime < minFrameTime) OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0*(minFrameTime-frameTime)));
			}

		
		} else {
			
			if (manipulator.valid())
			{
				view->setCameraManipulator(NULL);
				manipulator.release();
			}
			
			viewer.setDone(true);
		}
		
	}
	
	std::cout << "spinViewer done." << std::endl;

	return 0;
}
