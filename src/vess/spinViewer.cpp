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


#include "asUtil.h"
#include "vessThreads.h"
#include "asCameraManager.h"

using namespace std;

extern pthread_mutex_t pthreadLock;



// *****************************************************************************


int spinViewer_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{

    // make sure there is at least one argument (ie, a method):
	if (!argc) return 0;

	osgViewer::CompositeViewer *viewer = (osgViewer::CompositeViewer*)user_data;
    if (!viewer) return 0;


	// get the method (argv[0]):
    std::string theMethod;
	if (lo_is_string_type((lo_type)types[0]))
	{
		theMethod = string((char *)argv[0]);
	}
	else return 0;

	// parse the rest of the args:
	vector<float> floatArgs;
	vector<const char*> stringArgs;
	for (int i=1; i<argc; i++)
	{
		if (lo_is_numerical_type((lo_type)types[i]))
		{
			floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
		} else {
			stringArgs.push_back( (const char*) argv[i] );
		}
	}

	// try to find the node id:
	std::string nodeStr = string(path);
	nodeStr = nodeStr.substr(nodeStr.rfind("/")+1);
	t_symbol *s = gensym(nodeStr.c_str());
	osg::ref_ptr<asReferenced> n = s->s_thing;
	if (!n.valid())
	{
		std::cout << "spinViewer_liblo_callback: Could not find node: " << nodeStr << std::endl;
		return 0;
	}

	if ( (theMethod=="global6DOF") && (floatArgs.size()==6))
	{
//    	viewer->getCamera(0)->setViewMatrixAsLookAt(
		//std::cout << "got camera update:" << floatArgs[0] << "," << floatArgs[1] << "," << floatArgs[2] << "  " << floatArgs[3] << "," << floatArgs[4] << "," << floatArgs[5] << std::endl;

	//    osg::Vec3 = dirVector
    	//viewer->getCamera(0)->setViewMatrixAsLookAt(osg::Vec3(floatArgs[0],floatArgs[1],floatArgs[2]),
	}




	return 1;
}





// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	std::cout <<"\nspinViewer launching..." << std::endl;

	vessThread *vess = new vessThread(vessThread::LISTENER_MODE);

	std::string id = getHostname();


	// *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);

	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a 3D viewer for VESS.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");

	arguments.getApplicationUsage()->addCommandLineOption("-id <uniqueID>", "Specify an ID for this viewer (Default is hostname: '" + id + "')");

	arguments.getApplicationUsage()->addCommandLineOption("-vessID <uniqueID>", "Specify the VESS scene ID to listen to (Default: '" + vess->id + "')");
	arguments.getApplicationUsage()->addCommandLineOption("-vessAddr <addr>", "Set the receiving address for incoming OSC messages (Default: " + vess->rxAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-vessPort <port>", "Set the receiving port for incoming OSC messages (Default: " + vess->rxPort + ")");


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
	osg::ArgumentParser::Parameter param_vessID(vess->id);
	arguments.read("-vessID", param_vessID);
	osg::ArgumentParser::Parameter param_vessAddr(vess->rxAddr);
	arguments.read("-vessAddr", param_vessAddr);
	osg::ArgumentParser::Parameter param_vessPort(vess->rxPort);
	arguments.read("-vessPort", param_vessPort);



	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);



	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
	viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);



	// *************************************************************************
	// start the vessListener thread:

	if (!vess->start())
	{
        std::cout << "ERROR: could not start vessThread" << std::endl;
        exit(1);
	}

	vess->sceneManager->isGraphical = true;


	std::cout << "  Registering userNode '" << id << "' with VESS" << std::endl;

	// Add a userNode to the local scene and use it to feed a NodeTracker for
	// the viewer's camera. We expect that this node will be created in VESS and
	// that updates will be generated. Also note that we store it in a ref_ptr
	// so that it can't be deleted by vess threads.
	osg::ref_ptr<asReferenced> userNode = vess->sceneManager->createNode(id, "userNode");

    std::string OSCpath;

    OSCpath = "/vess/" + vess->id + "/" + std::string(id);
    lo_server_thread_add_method(vess->sceneManager->rxServ, OSCpath.c_str(), NULL, spinViewer_liblo_callback, (void*)&viewer);


    // for now, we try to send a message to vess that creates this node (assumes
    // that the server is running). Eventually, we'll need a better method to
    // synchronize user state with vess server... how? Maybe if VESS receives a
    // ping for a user that doesn't exist, it can request the creation messages?
    vess->sendSceneMessage("sss", "createNode", (char*) id.c_str(), "userNode", LO_ARGS_END);

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
	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());


	// *************************************************************************
    // window and graphicsContext stuff:
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        exit(0);
    }


    unsigned int width, height;
    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);


    // create a GraphicsContext::Traits for this window and initialize with some defaults:
	osg::ref_ptr<osg::GraphicsContext::Traits> gfxTraits = new osg::GraphicsContext::Traits;

    gfxTraits->windowName = "spinViewer";
	gfxTraits->x = 50;
	gfxTraits->y = 50;
	gfxTraits->width = 320;
	gfxTraits->height = 240;
	gfxTraits->windowDecoration = true;
	gfxTraits->doubleBuffer = true;
	gfxTraits->useCursor = true;
	gfxTraits->supportsResize = true;
	gfxTraits->sharedContext = 0;

	gfxTraits->displayNum = 0;
	gfxTraits->screenNum = 0;



	osg::ref_ptr<osg::GraphicsContext> gfxContext = osg::GraphicsContext::createGraphicsContext(gfxTraits.get());
	if (gfxContext.valid())
	{
		gfxContext->setClearColor(osg::Vec4f(0.0f,0.0f,0.0f,1.0f));
		gfxContext->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	} else {
		std::cout << "ERROR: Could not create GraphicsContext." << std::endl;
	}


    // *************************************************************************
    // set up initial view:
    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    viewer.addView(view.get());


    //
    //view->setUpViewOnSingleScreen(0);

    view->getCamera()->setGraphicsContext(gfxContext.get());
    view->getCamera()->setViewport(new osg::Viewport(0,0, gfxTraits->width, gfxTraits->height));

    view->setSceneData(vess->sceneManager->rootNode.get());


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

/*
	osgGA::TrackballManipulator *manipulator = new osgGA::TrackballManipulator();
	manipulator->setMinimumDistance ( 0.0001 );
	manipulator->setHomePosition( osg::Vec3(0,-1,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
*/


	osgGA::NodeTrackerManipulator *manipulator = new osgGA::NodeTrackerManipulator();
	manipulator->setTrackerMode(  osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
	manipulator->setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
	manipulator->setMinimumDistance( 0.0001 );
//	manipulator->setHomePosition( osg::Vec3(0,-1,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
	manipulator->setHomePosition( osg::Vec3(0,-0.0001,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
//	manipulator->setHomePosition( osg::Vec3(0,1,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
	manipulator->setTrackNode(userNode->getAttachmentNode());


	view->setCameraManipulator(manipulator);


	// *************************************************************************
	// set up any initial scene elements:

	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		vess->sceneManager->worldNode->addChild(argScene.get());
	}


	// *************************************************************************
	// start threads:
	viewer.realize();

	osg::Timer_t lastTick = osg::Timer::instance()->tick();
	osg::Timer_t frameTick = lastTick;

	// program loop:
	while( !viewer.done() && vess->isRunning() )
	{
		frameTick = osg::Timer::instance()->tick();
		if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
		{
			// TODO: send ping message here ...
			lastTick = frameTick;
		}

		// We now have to go through all the nodes, and check if we need to update the
		// graph. Note: this cannot be done as a callback in a traversal - dangerous.
		// In the callback, we have simply flagged what needs to be done (eg, set the
		// newParent symbol).
		pthread_mutex_lock(&pthreadLock);
		vess->sceneManager->updateGraph();
		pthread_mutex_unlock(&pthreadLock);

		pthread_mutex_lock(&pthreadLock);
		viewer.frame();
		pthread_mutex_unlock(&pthreadLock);
	}

	return 0;
}
