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
#include <boost/filesystem.hpp>

#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/GraphicsContext>
#include <osg/TextureCubeMap>
#include <osg/ClampColor>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>
#include <osg/Timer>

#include <boost/algorithm/string.hpp>

#include "config.h"
#include "ViewerManipulator.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "spinClientContext.h"
#include "osgUtil.h"
#include "GroupNode.h"
#include "SceneManager.h"
#include "ShapeNode.h"

#include "CompositeViewer.h"

#ifdef HAVE_SPNAV_H
#include "spnav.h"
#endif

//#include "dofppu.h"

//#define VELOCITY_SCALAR 0.005
//#define SPIN_SCALAR 0.01
#define VELOCITY_SCALAR 0.004
#define SPIN_SCALAR 0.007

extern pthread_mutex_t sceneMutex;

int run(int argc, char **argv)
{
	//std::cout <<"\nspinViewer launching..." << std::endl;
    using namespace spin;
	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	std::string userID;
	bool picker = false;
	bool mover = true;
    
    bool dof = false;
	float speedScaleValue = 1.0;
	float moving = false;
	
    int multisamples = 4;
	bool fullscreen = false;
	bool hideCursor=false;
	bool grid=false;

	double maxFrameRate = 60;
	
	int x=50;
	int y=50;
	int width=640;
	int height=480;
	int screen=-1;

    double nearClipping = -1;
    double farClipping = -1;
	
	std::string camConfig;
	std::string sceneID = spin.getSceneID();
	
    //osg::setNotifyLevel(osg::INFO);
    
    // *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a 3D viewer for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	
    // add generic spin arguments (for address/port setup, scene-id, etc)
    spinListener.addCommandLineOptions(&arguments);

    // arguments specific to spinviewer:	
    arguments.getApplicationUsage()->addCommandLineOption("--user-id <uniqueID>", "Specify a user ID for this viewer (Default: <this computer's name>)"); 
	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
    arguments.getApplicationUsage()->addCommandLineOption("--hide-cursor", "Hide the mouse cursor");
	arguments.getApplicationUsage()->addCommandLineOption("--config <filename>", "Provide a configuration file to customize the setup of multiple windows/cameras.");
	arguments.getApplicationUsage()->addCommandLineOption("--window <x y w h>", "Set the position (x,y) and size (w,h) of the viewer window (Default: 50 50 640 480)");
	arguments.getApplicationUsage()->addCommandLineOption("--clipping <near far>", "Manually specify fixed clipping planes (Default: clipping planes will be recomputed for every frame)");
    arguments.getApplicationUsage()->addCommandLineOption("--screen <num>", "Screen number to display on (Default: ALLSCREENS)");
	arguments.getApplicationUsage()->addCommandLineOption("--framerate <num>", "Set the maximum framerate (Default: not limited)");
	arguments.getApplicationUsage()->addCommandLineOption("--multisamples <num>", "Set the level of multisampling for antialiasing (Default: 4)");
	arguments.getApplicationUsage()->addCommandLineOption("--disable-camera-controls", "Disable mouse-baed camera controls for this user. This is helpful when using a mouse picker.");
	arguments.getApplicationUsage()->addCommandLineOption("--enable-mouse-picker", "Enable the mouse picker, and send events to the server");

	// *************************************************************************
	// PARSE ARGS:

    if (!spinListener.parseCommandLineOptions(&arguments))
        return 0;
    
	osg::ArgumentParser::Parameter param_userID(userID);
	arguments.read("--user-id", param_userID);
    if (not userID.empty())
        spin.setUserID(userID);

	osg::ArgumentParser::Parameter param_camConfig(camConfig);
	arguments.read("--config", param_camConfig);
   
	frustum frust;
	frust.valid = false;
	while (arguments.read("--frustum", frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far))
    {
		frust.valid = true;
	}
	while (arguments.read("--clipping",nearClipping,farClipping)) {}
    if (arguments.read("--fullscreen")) fullscreen=true;
    if (arguments.read("--hide-cursor")) hideCursor=true;
    if (arguments.read("--dof")) dof=true;
	while (arguments.read("--window",x,y,width,height)) {}
	while (arguments.read("--screen",screen)) {}
	while (arguments.read("--framerate",maxFrameRate)) {}
	while (arguments.read("--multisamples",multisamples)) {}
	if (arguments.read("--disable-camera-controls")) mover=false;
	if (arguments.read("--enable-mouse-picker")) picker=true;
	if (arguments.read("--grid")) grid=true;


	// *************************************************************************
	// For testing purposes, we allow loading a scene with a commandline arg:
    osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);
    
	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	CompositeViewer viewer = CompositeViewer(arguments);
	//osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
    //viewer.setThreadingModel(osgViewer::CompositeViewer::AutomaticSelection);
	//viewer.setThreadingModel(osgViewer::CompositeViewer::SingleThreaded);
	viewer.setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);

	viewer.getUsage(*arguments.getApplicationUsage());

	// *************************************************************************
    // multisampling / antialiasing:
    
    osg::DisplaySettings::instance()->setNumMultiSamples( multisamples );

	// *************************************************************************
	// start the listener thread:

	if (!spinListener.start())
	{
        std::cout << "ERROR: could not start SPIN listener" << std::endl;
        exit(EXIT_FAILURE);
	}

	spin.sceneManager_->setGraphical(true);

	// *************************************************************************
	// get details on keyboard and mouse bindings used by the viewer.
	viewer.getUsage(*arguments.getApplicationUsage());

	// *************************************************************************
	// open a connection to the SpaceNavigator
	#ifdef HAVE_SPNAV_H
	spnav_event spnavevent;
	if (spnav_open()==-1)
	{
         	std::cout << "Failed to connect to the space navigator" << std::endl;
        } else
	{
         	std::cout << "Listening to space navigator... " << std::endl;
	}
	#endif

    // *************************************************************************
    // set up initial view:

    osg::ref_ptr<ViewerManipulator> manipulator;


	// ***************************************************************************

	if (camConfig.size())
	{
		std::cout << "Loading config file (" << camConfig << ")..." << std::endl;
		
		TiXmlDocument doc( camConfig.c_str() );

		TiXmlNode *root = 0;
		TiXmlElement *child = 0;

		// Load the XML file and verify:
		if ( !doc.LoadFile() ) {
			std::cout << "WARNING: failed to load " << camConfig << "." << std::endl;
            return 1;
        }

		// get the <camConfig> tag and verify:
		if (!(root = doc.FirstChild( "camConfig" )))
		{
			std::cout << "WARNING: failed to load " << camConfig << ". XML file has no <camConfig> tag." << std::endl;
			return false;
            return 1;
		}
		
		// okay.. we have a valid xml file.

		// look for cameras:
		for ( child = root->FirstChildElement("window"); child; child = child->NextSiblingElement("window") )
		{
			spin::loadXMLwindow(child, viewer);
		}
	
	}

    else
    {
	    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
        viewer.addView(view.get());

        view->getCamera()->setClearColor(osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));

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
            double l,r,b,t,n,f; 
        	view->getCamera()->getProjectionMatrixAsFrustum(l,r,b,t,n,f);
            std::cout << "  Origin frustum:\t\t" <<l<<" "<<r<<" "<<b<<" "<<t<<" "<<n<<" "<<f<< std::endl;
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

        view->setSceneData(spin.sceneManager_->rootNode.get());

	    view->addEventHandler(new osgViewer::StatsHandler);
	    view->addEventHandler(new osgViewer::ThreadingHandler);
	    view->addEventHandler(new osgViewer::WindowSizeHandler);
        
        if (dof)
        {
            view->addEventHandler(new CustomResizeHandler(&viewer));
        }

	    view->addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	    view->addEventHandler( new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()) );

	    //view->setLightingMode(osg::View::NO_LIGHT);
	    view->setLightingMode(osg::View::HEADLIGHT);
	    //view->setLightingMode(osg::View::SKY_LIGHT);

        view->setCameraManipulator(new ViewerManipulator());
    }

    // *************************************************************************
    // disable OSG's auto computation of near/far clipping planes, and replace
    // the with our own:
    if ((nearClipping>=0) && (farClipping>nearClipping))
    {
        osgViewer::Viewer::Cameras cameras;
        viewer.getCameras(cameras);
        for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
        {
            (*iter)->setNearFarRatio(0.0001f);
            double fovy, aspectRatio, zNear, zFar;
            (*iter)->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
            (*iter)->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
            (*iter)->setProjectionMatrixAsPerspective(fovy, aspectRatio, nearClipping, farClipping);
            
            // for point clouds, let's disable small feature culling:
            /*
            (*iter)->setCullingMode((*iter)->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
            osg::View *v = (*iter)->getView();
            for (unsigned int slaveNum=0; slaveNum<v->getNumSlaves(); slaveNum++)
            {
                osg::Camera *slaveCam = v->getSlave(slaveNum)._camera.get();
                slaveCam->setCullingMode(slaveCam->getCullingMode() & ~osg::CullSettings::SMALL_FEATURE_CULLING);
            }
            */
        }
    }


    // *************************************************************************
    // create a camera manipulator
/*
    manipulator = new ViewerManipulator();
    manipulator->setPicker(picker);
    manipulator->setMover(mover);
*/
    
    for (unsigned int i=0; i<viewer.getNumViews(); i++)
    {
        ViewerManipulator *vm = dynamic_cast<ViewerManipulator*>(viewer.getView(i)->getCameraManipulator());
        if (vm)
        {
            vm->setPicker(picker);
            vm->setMover(mover);
        }
        //viewer.getView(i)->setCameraManipulator(manipulator.get());
    }
        
	// ***************************************************************************
	// debug print camera info
	
	if (0)
	{
        std::cout << std::endl << "CAMERA DEBUG PRINT:" << std::endl;
        
		osgViewer::Viewer::Cameras cameras;
		viewer.getCameras(cameras);
		
		osg::Vec3d eye, center, up;
		double left, right, bottom, top, zNear, zFar;

		if (manipulator.valid())
        {
            manipulator->getHomePosition (eye, center, up);
		    std::cout << "   manipulator:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;
        }
		for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
		{
            std::cout << "Camera '" << (*iter)->getName() << "':" << std::endl;
            
			osg::Vec4 v4 = (*iter)->getClearColor();
			std::cout << "   clear color:      (" << v4.x() << "," << v4.y() << "," << v4.z() << "," << v4.w() << ")" << std::endl;
			
			(*iter)->getProjectionMatrixAsFrustum (left, right, bottom, top, zNear, zFar);
			std::cout << "   Frustum:          " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;

			(*iter)->getViewMatrixAsLookAt (eye, center, up);
			std::cout << "   Camera LookAt:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;

            const osg::Viewport *viewport = (*iter)->getViewport();
            if (viewport)
                std::cout << "   Camera viewport:    " << viewport->x() << "," << viewport->y() << " " << viewport->width() << "x" << viewport->height() << std::endl;
            else
                std::cout << "   Camera viewport:    INVALID" << std::endl;

			osg::View *v = (*iter)->getView();
			std::cout << "   view numSlaves:   " << v->getNumSlaves() << std::endl;

            for (unsigned int slaveNum=0; slaveNum<v->getNumSlaves(); slaveNum++)
            {
                osg::Camera *slaveCam = v->getSlave(slaveNum)._camera.get();
                std::cout << "   Slave '" << slaveCam->getName() << "':" << std::endl;
            
			    osg::Vec4 v4 = slaveCam->getClearColor();
			    std::cout << "     clear color:      (" << v4.x() << "," << v4.y() << "," << v4.z() << "," << v4.w() << ")" << std::endl;
			
			    slaveCam->getProjectionMatrixAsFrustum (left, right, bottom, top, zNear, zFar);
			    std::cout << "     Frustum:          " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;

			    slaveCam->getViewMatrixAsLookAt (eye, center, up);
			    std::cout << "     Camera LookAt:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;

			    const osg::Viewport *viewport = slaveCam->getViewport();
			    if (viewport)
			        std::cout << "     Camera viewport:  pos=(" << viewport->x() << "," << viewport->y() << ") size=" << viewport->width() << "x" << viewport->height() << std::endl;
			    else
			        std::cout << "     Camera viewport:  INVALID" << std::endl;
            }
		}
	}

	
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
    // add the callback so that the viewer can be controlled by OSC messages:

    std::string viewerOSCpath = "/SPIN/"+spin.getSceneID()+"/"+spin.getUserID();

    std::cout << "registering viewer callback: " << viewerOSCpath << std::endl;

    std::vector<lo_server>::iterator servIter;
    for (servIter = spin.getContext()->lo_rxServs_.begin(); servIter != spin.getContext()->lo_rxServs_.end(); ++servIter)
    {
        lo_server_add_method((*servIter), viewerOSCpath.c_str(), NULL, viewerCallback, &viewer);
    }




	// *************************************************************************
	// set up any initial scene elements:

	if (argScene.valid()) {
		std::cout << "Loading sample model" << std::endl;
		spin.sceneManager_->worldNode->addChild(argScene.get());
	}

	if (grid)
	{
		spin.sceneManager_->createNode("grid", "GridNode");
	}

	// *************************************************************************
	// start threads:
	viewer.realize();

    // Try to subscribe with the current (default or manually specified) TCP
    // subscription information. If this fails, it's likely because the server
    // is not online, but when it comes online, it will send a userRefresh
    // message which will invoke another subscription attempt:
    spinListener.subscribe();

	// ask for refresh:
	spin.SceneMessage("s", "refresh", SPIN_ARGS_END);

	osg::Timer_t lastFrameTick = osg::Timer::instance()->tick();
	osg::Timer_t lastNavTick = osg::Timer::instance()->tick();

	double minFrameTime = 1.0 / maxFrameRate;


	//std::cout << "Starting viewer (threading = " << viewer.getThreadingModel() << ")" << std::endl;
    std::cout << "\nspinviewer is READY" << std::endl;


    // depth-of-field effect:
    if (dof)
    {
        viewer.frame();
        viewer.initializePPU();
        
        // disable color clamping, because we want to work on real hdr values
        osg::ClampColor* clamp = new osg::ClampColor();
        clamp->setClampVertexColor(GL_FALSE);
        clamp->setClampFragmentColor(GL_FALSE);
        clamp->setClampReadColor(GL_FALSE);

        // make it protected and override, so that it is done for the whole rendering pipeline
        spin.sceneManager_->worldNode->getOrCreateStateSet()->setAttribute(clamp, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
    }

	// program loop:
	while(not viewer.done())
	{
		
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
				// poll the space navigator:
				viewer.updateSpaceNavigator();
				#ifdef HAVE_SPNAV_Hi_
				int speventCount = 0;
				osg::Vec3 spVel, spSpin;
			
				//std::cout << "delta = " << osg::Timer::instance()->delta_s(lastNavTick, osg::Timer::instance()->tick()) << std::endl;
	
				// if the last significant event was more than a second ago,
				// assume the user has let go of the puck and reset speedScale
				if ((float)(osg::Timer::instance()->delta_s(lastNavTick, osg::Timer::instance()->tick()) > 1.0) && moving)
				{
					//std::cout << "reset spacenavigator" << std::endl;
					spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setVelocity", 0.0, 0.0, 0.0, SPIN_ARGS_END);
					spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setSpin", 0.0, 0.0, 0.0, SPIN_ARGS_END);
					speedScaleValue = 1.0;
					moving = false;
				}

				while (spnav_poll_event(&spnavevent))
				{
					if (spnavevent.type == SPNAV_EVENT_MOTION)
                        		{
						// just make note of the count; the latest motion data will be stored in spnavevent
						
						speventCount++;
						//spPos = osg::Vec3(spnavevent.motion.x, spnavevent.motion.z, spnavevent.motion.y);
						//spRot = osg::Vec3(spnavevent.motion.rx, spnavevent.motion.rz, spnavevent.motion.ry);
                			}
					else
					{
					        // SPNAV_EVENT_BUTTON
						static bool button1, button2;
						spin.NodeMessage(spin.getUserID().c_str(), "ssii", "event", "button", spnavevent.button.bnum, (int)spnavevent.button.press, SPIN_ARGS_END);
						if (spnavevent.button.bnum==0) button1 = (bool)spnavevent.button.press;	
						if (spnavevent.button.bnum==1) button2 = (bool)spnavevent.button.press;
						if (button1 && button2)
						{
							spin.NodeMessage(spin.getUserID().c_str(), "s", "goHome", SPIN_ARGS_END);
						}
                			}
				}

				// if at least one update was received this frame
				if (speventCount)
				{
					moving = true;

					// note: y and z axis flipped:
					float x = spnavevent.motion.x;
					float y = spnavevent.motion.z;
					float z = spnavevent.motion.y;
					float rx = -spnavevent.motion.rx;
					float ry = -spnavevent.motion.rz;
					float rz = -spnavevent.motion.ry;

					// if user is pushing beyond a threshold, inrement
					// the speed over time
					// NOTE: pulling up on the puch is harder to get ti to the max than pushing down, so we handle the up/down separately
					float xyMagnitude = sqrt((x*x)+(y*y));
					if ((xyMagnitude>190)||(z<-200)||(z>150))
						speedScaleValue += 0.02;
					else
						speedScaleValue -= 0.04;
						
					// max out at 10x of speed scaling
					if (speedScaleValue < 1) speedScaleValue = 1.0;
					else if (speedScaleValue > 10) speedScaleValue = 10.0;
					
					// compute velocity vector:
					speedScaleValue= 1.0;
					spVel = osg::Vec3( pow(x * VELOCITY_SCALAR, 5) * speedScaleValue * 2,
							   pow(y * VELOCITY_SCALAR, 5) * speedScaleValue * 2,
							   pow(z * VELOCITY_SCALAR, 5) * speedScaleValue);
					// rotate around the z-axis faster than the other two:
					spSpin = osg::Vec3( pow(rx*SPIN_SCALAR,5),
							    pow(ry*SPIN_SCALAR,5),
							    pow(rz*SPIN_SCALAR,5));

						
					//std::cout << "spacenav x,y,z="<<x<<","<<y<<","<<z<<" rx,ry,rz="<<rx<<","<<ry<<","<<rz<<" computed speedScale="<<speedScaleValue<< ", spVel= " << stringify(spVel) << ", spSpin= " << stringify(spSpin) << std::endl;
					spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setVelocity", spVel.x(), spVel.y(), spVel.z(), SPIN_ARGS_END);
					spin.NodeMessage(spin.getUserID().c_str(), "sfff", "setSpin", spSpin.x(), spSpin.y(), spSpin.z(), SPIN_ARGS_END);
				
					lastNavTick = osg::Timer::instance()->tick();
				}
				#endif

			
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
				spin.sceneManager_->update();
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
			
            for (int i=0; i<viewer.getNumViews(); i++)
            {
                // this should automatically release the manipulator (right??)
                viewer.getView(i)->setCameraManipulator(NULL);
            }

            // just in case, we'll check if the manipulator is still around:
			if (manipulator.valid())
			{
                manipulator.release();
			}
			
			viewer.setDone(true);
		}
	}

	
	#ifdef HAVE_SPNAV_H
	spnav_close();
	#endif

    // make sure we're done in case we didn't quit via interrupt
	spinListener.stop();

    return 0;
}

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
    /*
    // *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinFramework/args and override argc and argv with those:
    std::vector<char*> newArgs;
    if (argc == 1)
    {
        try
        {
            using namespace boost::filesystem;
            
            if (exists(SPIN_DIRECTORY+"/args"))
            {
                std::stringstream ss;
                ss << std::ifstream( (SPIN_DIRECTORY+"/args").c_str() ).rdbuf();
                
                // executable path is always the first argument:
                newArgs.push_back(argv[0]);
                
                std::string token;
                while (ss >> token)
                {
                    char *arg = new char[token.size() + 1];
                    copy(token.begin(), token.end(), arg);
                    arg[token.size()] = '\0';
                    newArgs.push_back(arg);
                }
                newArgs.push_back(0); // needs to end with a null item
                
                argc = (int)newArgs.size()-1;
                argv = &newArgs[0];
            }
        }
        catch ( const boost::filesystem::filesystem_error& e )
        {
            std::cout << "Warning: cannot read arguments from " << SPIN_DIRECTORY+"/args. Reason: " << e.what() << std::endl;
        }
    }
     */
    
    
    // *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinFramework/args and override argc and argv with those:
    std::vector<char*> newArgs = spin::getUserArgs();
    if ((argc==1) && (newArgs.size() > 1))
    {
        // need first arg (command name):
        newArgs.insert(newArgs.begin(), argv[0]);
        argc = (int)newArgs.size()-1;
        argv = &newArgs[0];
    }
    
    
    try
    {
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
