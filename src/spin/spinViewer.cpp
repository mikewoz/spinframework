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
#include "ShapeNode.h"

using namespace std;

extern pthread_mutex_t pthreadLock;



// global:
// we store userNode in a global ref_ptr so that it can't be deleted
//static osg::ref_ptr<UserNode> userNode;
static UserNode *userNode;


//void registerUser(spinContext *spin)
void registerUser()
{
	spinContext &spin = spinContext::Instance();
	
	//if (!userNode.valid())
	if (!userNode)
	{
        std::cout << "ERROR: could not register user" << std::endl;
        exit(1);
	}
	
	
	// Send a message to the server to create this node (assumes that the server
	// is running). If not, it will send a 'userRefresh' method upon startup
	// that will request that this function is called again
	spin.sendSceneMessage("sss", "createNode", userNode->id->s_name, "UserNode", LO_ARGS_END);

	std::cout << "  Registered user '" << userNode->id->s_name << "' with SPIN" << std::endl;

}

// *****************************************************************************


static int spinViewer_liblo_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{

	
	
    // make sure there is at least one argument (ie, a method):
	if (!argc) return 0;

	//spinContext *spin = (spinContext*)user_data;
    //if (!spin) return 0;
    spinContext &spin = spinContext::Instance();
    
    
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

	if (theMethod=="userRefresh")
	{
		//registerUser(spin);
		registerUser();
	}
		
		
		/*
	if ( (theMethod=="global6DOF") && (floatArgs.size()==6))
	{
//    	viewer->getCamera(0)->setViewMatrixAsLookAt(
		//std::cout << "got camera update:" << floatArgs[0] << "," << floatArgs[1] << "," << floatArgs[2] << "  " << floatArgs[3] << "," << floatArgs[4] << "," << floatArgs[5] << std::endl;

	//    osg::Vec3 = dirVector
    	//viewer->getCamera(0)->setViewMatrixAsLookAt(osg::Vec3(floatArgs[0],floatArgs[1],floatArgs[2]),
	}
*/



	return 1;
}




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
	
	int x=50;
	int y=50;
	int width=640;
	int height=480;
	int screen=-1;
	
	std::string redirectAddr, redirectPort;


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
	arguments.getApplicationUsage()->addCommandLineOption("-serverAddr <addr>", "Set the receiving address for incoming OSC messages (Default: " + spin.rxAddr + ")");
	arguments.getApplicationUsage()->addCommandLineOption("-serverPort <port>", "Set the receiving port for incoming OSC messages (Default: " + spin.rxPort + ")");

	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
	arguments.getApplicationUsage()->addCommandLineOption("--window <x y w h>", "Set the position (x,y) and size (w,h) of the viewer window (Default: 50 50 640 480)");
	arguments.getApplicationUsage()->addCommandLineOption("--screen <num>", "Screen number to display on (Default: ALLSCREENS)");

	
	arguments.getApplicationUsage()->addCommandLineOption("--disabled", "Disable camera controls for this user");
	arguments.getApplicationUsage()->addCommandLineOption("--picker", "Enable the mouse picker, and send events to the server");
	arguments.getApplicationUsage()->addCommandLineOption("--redirection <addr port>", "Redirect events to the specified address/port instead of the SPIN server");


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
	osg::ArgumentParser::Parameter param_spinAddr(spin.rxAddr);
	arguments.read("-serverAddr", param_spinAddr);
	osg::ArgumentParser::Parameter param_spinPort(spin.rxPort);
	arguments.read("-serverPort", param_spinPort);

	if (arguments.read("--fullscreen")) fullscreen=true;
	while (arguments.read("--window",x,y,width,height)) {}
	while (arguments.read("--screen",screen)) {}
	
	
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

	
	// register an extra OSC callback so that we can spy on OSC messages:
	std::string OSCpath = "/SPIN/" + spin.id;
	//lo_server_thread_add_method(spin.sceneManager->rxServ, OSCpath.c_str(), NULL, spinViewer_liblo_callback, (void*)spin);
	lo_server_thread_add_method(spin.sceneManager->rxServ, OSCpath.c_str(), NULL, spinViewer_liblo_callback, NULL);

	
	
	// Add a UserNode to the local scene and use it to feed a NodeTracker for
	// the viewer's camera. We expect that this node will be created in the
	// sceneManager and that updates will be generated. 
	userNode = dynamic_cast<UserNode*>(spin.sceneManager->getOrCreateNode(id.c_str(), "UserNode"));
	
	
	// send userNode info to spin
	registerUser();
	//registerUser(spin);
	
		


	// *************************************************************************
	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());

    // *************************************************************************
    // set up initial view:
    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    viewer.addView(view.get());

    if (fullscreen)
    {
    	if (screen<0) view->setUpViewAcrossAllScreens();
    	else view->setUpViewOnSingleScreen(screen);
    } else {
    	if (screen<0) view->setUpViewInWindow(x,y,width,height);
    	else view->setUpViewInWindow(x,y,width,height,screen);
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

/*
	osgGA::TrackballManipulator *manipulator = new osgGA::TrackballManipulator();
	manipulator->setMinimumDistance ( 0.0001 );
	//manipulator->setHomePosition( osg::Vec3(0,0,0), osg::Vec3(0,1,0), osg::Vec3(0,0,1), false );
	manipulator->setHomePosition( osg::Vec3(0,-0.0001,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
*/

	

	//osgGA::NodeTrackerManipulator *manipulator = new osgGA::NodeTrackerManipulator();
	//custom_NodeTrackerManipulator *manipulator = new custom_NodeTrackerManipulator();
	//ViewerManipulator *manipulator = new ViewerManipulator(spin, userNode.get());
	//ViewerManipulator *manipulator = new ViewerManipulator(userNode.get());
	osg::ref_ptr<ViewerManipulator> manipulator = new ViewerManipulator(userNode);
	
	manipulator->setPicker(picker);
	manipulator->setMover(mover);
	if (!redirectAddr.empty() && !redirectPort.empty())
		manipulator->setRedirection(redirectAddr, redirectPort);
	
	manipulator->setTrackerMode(  osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
	manipulator->setRotationMode( osgGA::NodeTrackerManipulator::ELEVATION_AZIM );
	manipulator->setMinimumDistance( 0.0001 );
//	manipulator->setHomePosition( osg::Vec3(0,-1,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
	manipulator->setHomePosition( osg::Vec3(0,-0.0001,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
//	manipulator->setHomePosition( osg::Vec3(0,1,0), osg::Vec3(0,0,0), osg::Vec3(0,0,1), false );
	manipulator->setTrackNode(userNode->getAttachmentNode());


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

	if (0)
	{
		osg::ref_ptr<ShapeNode> shp = dynamic_cast<ShapeNode*>(spin.sceneManager->createNode("shp", "ShapeNode"));
		shp->setShape(ShapeNode::SPHERE);
		shp->setTranslation(0,5,0);
		shp->setInteractionMode(GroupNode::PUSH);
	}

	// *************************************************************************
	// start threads:
	viewer.realize();

	// ask for refresh:
	spin.sendSceneMessage("s", "refresh", LO_ARGS_END);
	
	osg::Timer_t lastTick = osg::Timer::instance()->tick();
	osg::Timer_t frameTick = lastTick;

	// program loop:
	//while( !viewer.done() && spin.isRunning() )
	while( !viewer.done() )
	{
		
		if (spin.isRunning())
		{
			frameTick = osg::Timer::instance()->tick();
			if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
			{
				spin.sendInfoMessage("/ping/user", "s", (char*) id.c_str(), LO_ARGS_END);
				lastTick = frameTick;
			}
	
			// TODO: move this into the callback, and do it only when userNode sends
			// a global6DOF message:
			/*
			osg::Matrix m = osg::computeLocalToWorld(userNode->currentNodePath);
			osg::Vec3 rot = QuatToEuler(m.getRotate());
			manipulator->setCenter(m.getTrans());
			manipulator->setRotation(osg::Quat( rot.x()+osg::PI_2,osg::X_AXIS, rot.y(),osg::Y_AXIS, rot.z(),osg::Z_AXIS) );
			*/
	
			// We now have to go through all the nodes, and check if we need to update the
			// graph. Note: this cannot be done as a callback in a traversal - dangerous.
			// In the callback, we have simply flagged what needs to be done (eg, set the
			// newParent symbol).
			pthread_mutex_lock(&pthreadLock);
			spin.sceneManager->updateGraph();
			pthread_mutex_unlock(&pthreadLock);
	
			pthread_mutex_lock(&pthreadLock);
			viewer.frame();
			pthread_mutex_unlock(&pthreadLock);
		
		} else {
			
			view->setCameraManipulator(NULL);
			manipulator.release();
			viewer.setDone(true);
		}
		
	}
	
	std::cout << "spinViewer done." << std::endl;

	return 0;
}
