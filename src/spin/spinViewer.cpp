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

#include <boost/algorithm/string.hpp>

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
	float left;
	float right;
	float bottom;
	float top;
	float near;
	float far;
};

static void loadXMLcamera(TiXmlElement *XMLnode, osgViewer::Viewer::View *view, osg::Camera *cam, int screenWidth, int screenHeight)
{
	TiXmlElement *child = 0;
	std::string tag="", val="";
	float v[4];

    osg::Vec3 eye = osg::Vec3(0,0,0);
	osg::Vec3 lookat = osg::Y_AXIS;
	osg::Vec3 up = osg::Z_AXIS;

    if (XMLnode->Attribute("id"))
    {
        std::cout << "    Loading camera: " << XMLnode->Attribute("id") << std::endl;
        cam->setName(XMLnode->Attribute("id"));
    }
    else std::cout << "    Loading camera" << std::endl;

	// get all properties of the camera:
	for ( child = XMLnode->FirstChildElement(); child; child = child->NextSiblingElement() )
	{
		// get tag and value:
		if (child->FirstChild())
		{
			tag = child->Value();
			val = child->FirstChild()->Value();
		} else continue;
        

		// now update camera parameters:
		if (tag=="clearColor")
		{
			if (sscanf (val.c_str(),"%f %f %f %f",&v[0],&v[1],&v[2],&v[3]))
				cam->setClearColor( osg::Vec4(v[0],v[1],v[2],v[3]) );
		}
		else if (tag=="eye")
		{
			if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
				eye = osg::Vec3(v[0],v[1],v[2]);
		}
		else if (tag=="lookat")
		{
			if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
				lookat = osg::Vec3(v[0],v[1],v[2]);
		}
		else if (tag=="up")
		{
			if (sscanf (val.c_str(),"%f %f %f",&v[0],&v[1],&v[2]))
				up = osg::Vec3(v[0],v[1],v[2]);
		}
        
		else if (tag=="viewport")
		{
			if (sscanf(val.c_str(),"%f%% %f%% %f%% %f%%",&v[0],&v[1],&v[2],&v[3])==4)
			{
			    cam->setViewport( (int) (v[0]/100*screenWidth/100), (int) (v[1]/100*screenHeight), (int) (v[2]/100*screenWidth), (int) (v[3]/100*screenHeight) );
			}
			else if (sscanf(val.c_str(),"%f %f %f %f",&v[0],&v[1],&v[2],&v[3])==4)
			{
			    view->getCamera()->setViewport( v[0],v[1],v[2],v[3] );
			}
			else {
			    std::cout << "Bad viewport values: " << val << ". Need four values <x y width height>, either as pixel values of percentages of the window size" << std::endl;
			}

		}
	    else if (tag == "perspective")
	    {
	        float fovy, aspectRatio, zNear, zFar;
	        if (sscanf(val.c_str(), "%f %f %f %f", &fovy, &aspectRatio, &zNear, &zFar))
	        {
	            //std::cout << "setting perspective of " << fovy << "deg, aspect: " << aspectRatio << std::endl;
	            cam->setProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
	        }
	    }
		else if (tag=="frustum")
		{
            frustum frust;
            if (sscanf (val.c_str(),"%f %f %f %f %f %f",&frust.left,&frust.right,&frust.bottom,&frust.top,&frust.near,&frust.far))
            {
                cam->setProjectionMatrixAsFrustum(frust.left, frust.right, frust.bottom, frust.top, frust.near, frust.far);
		    }
        }

		else
		{
			std::cout << "Unknown parameter in configuration file: " << tag << std::endl;
		}
	}

	//cam->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    //cam->setViewMatrixAsLookAt(eye, lookat, up);


    spin::ViewerManipulator *manipulator = new spin::ViewerManipulator();
    manipulator->setHomePosition( eye, lookat, up, false );
    view->setCameraManipulator(manipulator);


/*
	// note: the first matrix scales and offsets the axes (perspective) while the second matrix offsets the view:
	//viewer.addSlave(cam->camera.get(), cam->pMatrix*cam->tMatrix, cam->rMatrix);
	//cam->camera->setViewMatrixAsLookAt( cam->_eye, cam->_lookat, cam->_up );
	osg::Matrixd viewMatrix;
	viewMatrix.makeLookAt( cam->_eye, cam->_lookat, cam->_up );
	viewMatrix *= osg::Matrixd::rotate(osg::PI/2, X_AXIS);
	viewer.addSlave(cam->camera.get(), cam->pMatrix*cam->tMatrix, cam->rMatrix*viewMatrix);
*/
        
}




static void loadXMLwindow(TiXmlElement *XMLnode, osgViewer::CompositeViewer &viewer)
{
	TiXmlElement *n;

    // first check if the wsi is valid:
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        return;
    }


    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    if (XMLnode->Attribute("id"))
    {
        std::cout << "  Loading window: " << XMLnode->Attribute("id") << std::endl;
        view->setName("SPIN Viewer: "+std::string(XMLnode->Attribute("id")));
    }
    else std::cout << "  Loading window" << std::endl;

    osg::DisplaySettings* ds;
    if (view->getDisplaySettings())
    {
        ds = view->getDisplaySettings();
     }
    else
    {
        //std::cout << "Display settings not valid" << std::endl;
        ds = osg::DisplaySettings::instance();
    }


    osg::GraphicsContext::ScreenIdentifier si;
    si.readDISPLAY();

    // displayNum has not been set so reset it to 0.
    if (si.displayNum<0) si.displayNum = 0;

    // get screenNum from config file, or default to 0:
    if ((n = XMLnode->FirstChildElement("screen")))
        si.screenNum = atoi(n->FirstChild()->Value());
    else
        si.screenNum = 0;
    
    
    // Now that we have the screen, let's get the resolution. This is important
    // because size and position in the xml file can be specified as a
    // percentage
	unsigned int screenWidth, screenHeight;
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(si.screenNum), screenWidth, screenHeight);
	//std::cout << "Resolution for screen " << si.screenNum << " is: " << screenWidth << "x" << screenHeight << std::endl;

    /*
	if (n = XMLnode->FirstChildElement("fullscreen"))
	{
		if (n->FirstChild()->Value() == "true")
        {
            view->setUpViewOnSingleScreen(screenNum);
        }
        else
        {
            int x=50;
            int y=50;
            int w=800;
            int h=600;
	        if (n = XMLnode->FirstChildElement("windowPosition"))
	            sscanf( n->FirstChild()->Value(), "%d %d", &x, &y);
		    if (n = XMLnode->FirstChildElement("windowSize"))
	            sscanf( n->FirstChild()->Value(), "%d %d", &w, &h);
		    view->setUpViewInWindow(x,y,w,h,screenNum);
            maxWidth = w;
            maxHeight = h;
        }
    }
    */


    // create a GraphicsContext::Traits for this window and initialize with
    // some defaults:
	//osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(view->getDisplaySettings());
	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds);

	//traits->hostName = si.hostName;
    traits->displayNum = 0;//si.displayNum;
    traits->screenNum = si.screenNum;
    traits->x = 0;
    traits->y = 0;
    traits->width = 320;
    traits->height = 240;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->useCursor = true;
    traits->supportsResize = true;
    traits->sharedContext = 0;
    traits->windowName = view->getName();

    // update window position based on config file:
    if ((n = XMLnode->FirstChildElement("windowPosition")))
    {
        float percentWidth, percentHeight;
        if (sscanf( n->FirstChild()->Value(), "%f%% %f%%", &percentWidth, &percentHeight)==2)
        {
            traits->x = (int) screenWidth * (percentWidth/100);
            traits->y = (int) screenHeight * (percentHeight/100);
        }
        else
        {
            sscanf( n->FirstChild()->Value(), "%d %d", &traits->x, &traits->y);
        }
    }

    if ((n = XMLnode->FirstChildElement("windowSize")))
    {
        float percentWidth, percentHeight;
        if (sscanf( n->FirstChild()->Value(), "%f%% %f%%", &percentWidth, &percentHeight)==2)
        {
            traits->width = (int) screenWidth * (percentWidth/100);
            traits->height = (int) screenHeight * (percentHeight/100);
        }
        else
        {
            sscanf( n->FirstChild()->Value(), "%d %d", &traits->width, &traits->height);
        }
    }

    if ((n = XMLnode->FirstChildElement("supportsResize")))
    {
        if (boost::iequals(n->FirstChild()->Value(), "false"))
        {
            traits->supportsResize = false;
        }
        else
        {
            traits->supportsResize = true;
            view->addEventHandler(new osgViewer::WindowSizeHandler);
        }
    }

    if ((n = XMLnode->FirstChildElement("useCursor")))
    {
        if (boost::iequals(n->FirstChild()->Value(), "false"))
            traits->useCursor = false;
        else
            traits->useCursor = true;
    }

    if ((n = XMLnode->FirstChildElement("windowDecoration")))
    {
        if (boost::iequals(n->FirstChild()->Value(), "false"))
            traits->windowDecoration = false;
        else
            traits->windowDecoration = true;
    }

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());


	// now search for cameras:
    bool firstCamera = true;
	for ( n = XMLnode->FirstChildElement("camera"); n; n = n->NextSiblingElement("camera") )
	{
	    osg::Camera *cam;
        if (firstCamera)
        {
            cam = view->getCamera();
            firstCamera = false;
        }
        else
        {
            cam = new osg::Camera();
            view->addSlave(cam, view->getCamera()->getProjectionMatrix(), view->getCamera()->getViewMatrix());
        }

        if (gc.valid()) cam->setGraphicsContext(gc.get());
        else std::cout << "ERROR: GraphicsContext not valid. Bad configuration file?" << std::endl;

        // Projection matrix aspect fix (can be overridden using either the
        // frustum or perspective configuration values in config file)
        if (0) {
            double fovy, aspectRatio, zNear, zFar;
            cam->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);

            double newAspectRatio = double(traits->width) / double(traits->height);
            double aspectRatioChange = newAspectRatio / aspectRatio;
            if (aspectRatioChange != 1.0)
            {
                cam->getProjectionMatrix() *= osg::Matrix::scale(1.0/aspectRatioChange,1.0,1.0);
            }
        }

        loadXMLcamera( n, view, cam, traits->width, traits->height );

        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        cam->setDrawBuffer(buffer);
        cam->setReadBuffer(buffer);
	}

	view->setLightingMode(osg::View::SKY_LIGHT);
	view->addEventHandler(new osgViewer::StatsHandler);
    view->setSceneData(spin::spinApp::Instance().sceneManager->rootNode.get());
    viewer.addView(view.get());

    osgViewer::GraphicsWindow* gw = dynamic_cast<osgViewer::GraphicsWindow*>(gc.get());
    if (gw)
    {
        gw->getEventQueue()->getCurrentEventState()->setWindowRectangle(0, 0, traits->width, traits->height );
    }

}





int run(int argc, char **argv)
{
	//std::cout <<"\nspinViewer launching..." << std::endl;
    using namespace spin;
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
	
	std::string camConfig;
	
	std::string sceneID = spin.getSceneID();
	std::string rxHost = lo_address_get_hostname(spinListener.lo_rxAddrs_[0]);
	std::string rxPort = lo_address_get_port(spinListener.lo_rxAddrs_[0]);
	std::string txHost = lo_address_get_hostname(spinListener.lo_txAddrs_[0]);
	std::string txPort = lo_address_get_port(spinListener.lo_txAddrs_[0]);
	std::string syncPort = lo_address_get_port(spinListener.lo_syncAddr);
    int ttl=1;

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
	arguments.getApplicationUsage()->addCommandLineOption("--server-addr <host> <port>", "Tell the viewer where the spinserver is located, and which port it receives UDP messages (Default: " + rxHost + " " + rxPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("--recv-addr <host> <port>", "Set the receiving address for incoming UDP messages (Default: " + txHost + " " + txPort + ")");
	arguments.getApplicationUsage()->addCommandLineOption("--tcp-port <port>", "Specify an incoming TCP port when subscribing to the server's TCP channel (Default: " + spinListener.tcpPort_ + ")");
	arguments.getApplicationUsage()->addCommandLineOption("--sync-port <port>", "Set the receiving port for timecode sync (Default: " + syncPort + ")");
    arguments.getApplicationUsage()->addCommandLineOption("--ttl <number>", "Set the TTL (time to live) for multicast packets in order to hop across routers (Default: 1)");

	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
    arguments.getApplicationUsage()->addCommandLineOption("--hide-cursor", "Hide the mouse cursor");
	arguments.getApplicationUsage()->addCommandLineOption("--config <filename>", "Provide a configuration file to set up a view for multiple windows/cameras.");
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

	osg::ArgumentParser::Parameter param_camConfig(camConfig);
	arguments.read("--config", param_camConfig);
    
	while (arguments.read("--server-addr", rxHost, rxPort)) {
        spinListener.lo_txAddrs_.clear();
		spinListener.lo_txAddrs_.push_back(lo_address_new(rxHost.c_str(), rxPort.c_str()));
	}
	while (arguments.read("--recv-addr", txHost, txPort)) {
        spinListener.lo_rxAddrs_.clear();
		spinListener.lo_rxAddrs_.push_back(lo_address_new(txHost.c_str(), txPort.c_str()));
	}

    arguments.read("--tcp-port", spinListener.tcpPort_);	

	while (arguments.read("--sync-port", syncPort)) {
		spinListener.lo_syncAddr = lo_address_new(rxHost.c_str(), syncPort.c_str());
	}
    while (arguments.read("--ttl", ttl)) {
        spinListener.setTTL(ttl);
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

    osg::ref_ptr<ViewerManipulator> manipulator;


	// ***************************************************************************

	if (camConfig.size())
	{
		std::cout << "Found config file (" << camConfig << "). Loading..." << std::endl;
		
		TiXmlDocument doc( camConfig.c_str() );

		TiXmlNode *root = 0;
		TiXmlElement *child = 0;

		// Load the XML file and verify:
		if ( !doc.LoadFile() ) {
			std::cout << "WARNING: failed to load " << camConfig << ". Invalid XML format." << std::endl;
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
			loadXMLwindow(child, viewer);
		}
	
	}

    else
    {
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

        view->setCameraManipulator(new ViewerManipulator());
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
				spin.sceneManager->update();
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
