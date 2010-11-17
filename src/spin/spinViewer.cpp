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
#include "spinApp.h"
#include "spinClientContext.h"
#include "osgUtil.h"
#include "GroupNode.h"
#include "SceneManager.h"
#include "ShapeNode.h"
#include "config.h"

extern pthread_mutex_t sceneMutex;

struct frustum
{
	bool valid;
	double left;
	double right;
	double bottom;
	double top;
	double near;
	double far;
};

int run(int argc, char **argv)
{
	//std::cout <<"\nspinViewer launching..." << std::endl;

	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	std::string userID;
	bool picker = false;
	bool mover = true;
	
	bool fullscreen = false;
	bool hideCursor=false;

	double maxFrameRate = 60;
	
	int x=50;
	int y=50;
	int width=640;
	int height=480;
	int screen=-1;
	
	std::string sceneID = spin.getSceneID();
	std::string rxHost = lo_address_get_hostname(spinListener.lo_rxAddrs_[0]);
	std::string rxPort = lo_address_get_port(spinListener.lo_rxAddrs_[0]);
	std::string syncPort = lo_address_get_port(spinListener.lo_syncAddr);

	// *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a 3D viewer for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
	arguments.getApplicationUsage()->addCommandLineOption("--version", "Display the version number and exit.");

	arguments.getApplicationUsage()->addCommandLineOption("--user-id <uniqueID>", "Specify a user ID for this viewer (Default is the local host name)"); 
    // FIXME: see note in spinServer.cpp
    //: '" + userID + "')");

	arguments.getApplicationUsage()->addCommandLineOption("--scene-id <uniqueID>", "Specify the scene ID to listen to (Default: '" + sceneID + "')");
	arguments.getApplicationUsage()->addCommandLineOption("--server-addr <host> <port>", "Set the receiving address for incoming OSC messages (Default: " + rxHost + " " + rxPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("--sync-port <port>", "Set the receiving port for timecode sync (Default: " + syncPort + ")");

	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
    arguments.getApplicationUsage()->addCommandLineOption("--hide-cursor", "Hide the mouse cursor");
	arguments.getApplicationUsage()->addCommandLineOption("--window <x y w h>", "Set the position (x,y) and size (w,h) of the viewer window (Default: 50 50 640 480)");
	arguments.getApplicationUsage()->addCommandLineOption("--screen <num>", "Screen number to display on (Default: ALLSCREENS)");
	arguments.getApplicationUsage()->addCommandLineOption("--framerate <num>", "Set the maximum framerate (Default: not limited)");

    //TODO:2010-08-16:aalex:rename --disabled to --disable-camera-controls
	arguments.getApplicationUsage()->addCommandLineOption("--disabled", "Disable camera controls for this user");
	arguments.getApplicationUsage()->addCommandLineOption("--picker", "Enable the mouse picker, and send events to the server");


	// *************************************************************************
	// PARSE ARGS:

	// if user request help or version write it out to cout.
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
    // otherwise, start the viewer:
	osg::ArgumentParser::Parameter param_userID(userID);
	arguments.read("--user-id", param_userID);
    if (not userID.empty())
        spin.setUserID(userID);

	osg::ArgumentParser::Parameter param_spinID(sceneID);
	arguments.read("--scene-id", param_spinID);
	spin.setSceneID(sceneID);

	while (arguments.read("--server-addr", rxHost, rxPort)) {
		spinListener.lo_txAddr= lo_address_new(rxHost.c_str(), rxPort.c_str());
	}

	while (arguments.read("--sync-port", syncPort)) {
		spinListener.lo_syncAddr = lo_address_new(rxHost.c_str(), syncPort.c_str());
	}

	frustum frust;
	frust.valid = false;
	while (arguments.read("--frustum", frust.left, frust.right, frust.bottom, frust.top)) {
		frust.near = 1.0;
		frust.far = 10000.0;
		frust.valid = true;
	}

	if (arguments.read("--fullscreen")) fullscreen=true;
    if (arguments.read("--hide-cursor")) hideCursor=true;
	while (arguments.read("--window",x,y,width,height)) {}
	while (arguments.read("--screen",screen)) {}
	while (arguments.read("--framerate",maxFrameRate)) {}
	
	if (arguments.read("--disabled")) mover=false;
	if (arguments.read("--picker")) picker=true;

	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);

	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
    // Aug 19 2010:tmatth: Tried this for multithreading
	//viewer.setThreadingModel(osgViewer::CompositeViewer::AutomaticSelection);
	//viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
	viewer.setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);

	viewer.getUsage(*arguments.getApplicationUsage());


	// *************************************************************************
	// start the listener thread:

	if (!spinListener.start())
	{
        std::cout << "ERROR: could not start SPIN listener" << std::endl;
        exit(EXIT_FAILURE);
	}

	spin.sceneManager->setGraphical(true);

	// *************************************************************************
	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());

    // *************************************************************************
    // set up initial view:
    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    viewer.addView(view.get());

    view->getCamera()->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 0.0));

    if (fullscreen)
    {
    	if (screen<0) view->setUpViewAcrossAllScreens();
    	else view->setUpViewOnSingleScreen(screen);
    } else {
    	if (screen<0) view->setUpViewInWindow(x,y,width,height);
    	else view->setUpViewInWindow(x,y,width,height,screen);
    }

    if (frust.valid)
    {
    	//view->getCamera()->getProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
    	std::cout << "  Custom frustum:\t\t" << frust.left<<" "<<frust.right<<" "<<frust.bottom<<" "<<frust.top<<" "<<frust.near<<" "<<frust.far << std::endl;
    	view->getCamera()->setProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
    }

    //TODO:2010-07-28:aalex:Load an image for a window icon
    //HANDLE icon = osg::LoadImage(0, "MyIcon.ico", osg::IMAGE_ICON, 0, 0, osg::LR_LOADFROMFILE);
    osgViewer::ViewerBase::Windows windows;
    osgViewer::ViewerBase::Windows::iterator wIter;
    viewer.getWindows(windows);
    for (wIter=windows.begin(); wIter!=windows.end(); wIter++)
    {
    	(*wIter)->setWindowName("spinviewer " + spin.getUserID() + "@" + spin.getSceneID());
		if (hideCursor) (*wIter)->useCursor(false);

        //TODO:2010-07-28:aalex:Set a window icon
        //if( hIcon && hWnd )
        //{
        //    osg::SendMessage(hWnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
        //} 
    }

    /// Thu Aug 19 2010:tmatth:FIXME: this segfaults in multithreaded mode
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

	osg::Timer_t lastFrameTick = osg::Timer::instance()->tick();

	double minFrameTime = 1.0 / maxFrameRate;


	//std::cout << "Starting viewer (threading = " << viewer.getThreadingModel() << ")" << std::endl;

	// program loop:
	while(not viewer.done())
	{
		//std::cout << "frame: " << view->getFrameStamp()->getSimulationTime() << std::endl;
		
		if (spinListener.isRunning())
		{
			// ***** ORIGINAL (pollUpdates is done in clientContext thread):
			/*
			osg::Timer_t startFrameTick = osg::Timer::instance()->tick();

			spinListener.pollUpdates();
			
			pthread_mutex_lock(&sceneMutex);
			viewer.frame();
			pthread_mutex_unlock(&sceneMutex);

			if (maxFrameRate>0)
			{
				// work out if we need to force a sleep to hold back the frame rate
				osg::Timer_t endFrameTick = osg::Timer::instance()->tick();
				double frameTime = osg::Timer::instance()->delta_s(startFrameTick, endFrameTick);
				if (frameTime < minFrameTime) OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0*(minFrameTime-frameTime)));
			}
			*/

			// ***** NEW (pollUpdates are done in same thread as viewer)

			double dt = osg::Timer::instance()->delta_s(lastFrameTick, osg::Timer::instance()->tick());

			if (dt >= minFrameTime)
			{
				// we used to just call viewer.frame() within a mutex, but we
				// only really need to apply the mutex to the update traversal
				/*
				pthread_mutex_lock(&sceneMutex);
				viewer.frame();
				pthread_mutex_unlock(&sceneMutex);
				*/

				viewer.advance();
				viewer.eventTraversal();
				pthread_mutex_lock(&sceneMutex);
				viewer.updateTraversal();
				viewer.renderingTraversals();
				pthread_mutex_unlock(&sceneMutex);
				
				// save time when the last time a frame was rendered:
				lastFrameTick = osg::Timer::instance()->tick();
				dt = 0;
			}

			unsigned int sleepTime;
			if (!recv) sleepTime = static_cast<unsigned int>(1000000.0*(minFrameTime-dt));
			else sleepTime = 0;
			if (sleepTime > 100) sleepTime = 100;

			if (!recv) OpenThreads::Thread::microSleep(sleepTime);

			// ***** END
		
		} else {
			
			if (manipulator.valid())
			{
				view->setCameraManipulator(NULL);
				manipulator.release();
			}
			
			viewer.setDone(true);
		}
	}

    // make sure we're done in case we didn't quit via interrupt
	spinListener.stop();

    return 0;
}

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
    try {
        // Aug 19 2010:tmatth: Tried this to make multithreading work, didn't help
        // osg::Referenced::setThreadSafeReferenceCounting(true);
        int result = run(argc, argv);
        std::cout << "\nspinviewer exited normally." << std::endl;
        return result;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Got exception " << e.what() << std::endl;
        return 1;
    }
}
