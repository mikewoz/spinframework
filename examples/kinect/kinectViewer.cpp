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
#include <fstream>
#include <sstream>

#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osgSim/LightPointNode>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>
#include <osg/Timer>

//#include <boost/algorithm/string.hpp>

#include "ViewerManipulator.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "spinClientContext.h"
#include "osgUtil.h"
#include "GroupNode.h"
#include "SceneManager.h"
#include "ShapeNode.h"
#include "config.h"

#include "libfreenect.hpp"

extern pthread_mutex_t sceneMutex;

volatile int die = 0;

pthread_t freenect_thread;

freenect_context *f_ctx;
freenect_device *f_dev;

double depth_data[240][320];

int frame = 0;

pthread_mutex_t backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;


bool export_state=false;

void addToLightPointNode(osgSim::LightPointNode& lpn, unsigned int noSteps, unsigned int j, bool bexport, std::ostringstream& sstream)
{
    //using namespace osgSim;

    lpn.getLightPointList().reserve(noSteps);

    for ( unsigned int i=0;i<noSteps;++i)
    {
        osgSim::LightPoint lp;
        lp._color.set(1.0f, 1.0f, 1.0f, 1.0f);
        lp._position.z() = depth_data[j][i] * 14.0;
        lp._position.y() = (j-120.0) * (lp._position.z() + -10.0) * 0.005;
        lp._position.x() = (i-160.0) * (lp._position.z() + -10.0) * 0.005;

        if (depth_data[j][i] <= ( 100.0/3.33 ))
            continue;

        lpn.addLightPoint(lp);

        if (bexport)
        {
            sstream<<(int)lp._position.x()<<" "<<(int)lp._position.y()<<" "<<(int)lp._position.z()<<std::endl;
        }
    }
}

osg::Node* createLightPointsDatabase(bool export_state)
{
    osg::MatrixTransform* transform = new osg::MatrixTransform;

    transform->setDataVariance(osg::Object::STATIC);
    transform->setMatrix(osg::Matrix::scale(0.002, 0.002, 0.002));

    int noStepsX = 320;
    int noStepsY = 240;

    std::ofstream file;
    if (export_state)
    {
        file.open("kinect.ply");
        file<<"ply"<<std::endl;
        file<<"format ascii 1.0"<<std::endl;
        file<<"comment bla bla"<<std::endl;
        file<<"element vertex ";
    }

    pthread_mutex_lock(&backbuf_mutex);

    std::ostringstream ss;
    size_t totall_points = 0;
    for (int i=0;i<noStepsY;++i)
    {
        osgSim::LightPointNode* lpn = new osgSim::LightPointNode;
        addToLightPointNode(*lpn, noStepsX, i, export_state, ss);
        totall_points += lpn->getNumLightPoints();
        transform->addChild(lpn);
    }
    pthread_mutex_unlock(&backbuf_mutex);

    osg::Group* group = new osg::Group;
    group->addChild(transform);

    if (export_state)
    {
        file<<totall_points<<std::endl;
        file<<"property float x"<<std::endl;
        file<<"property float y"<<std::endl;
        file<<"property float z"<<std::endl;
        file<<"end_header"<<std::endl;
        file<<ss.str();
        export_state = false;
        file.close();
    }

    return group;
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
    uint16_t *depth = ( uint16_t * ) v_depth;
    pthread_mutex_lock(&backbuf_mutex);
    for ( int i=0; i<240; i++ )
    {
        for ( int j=0; j<320; j++ )
        {
            double metric = 100.0 / ( -0.00307 * depth[ ( ( i*2 ) * 640) + ( j*2 ) ] + 3.33 );
            depth_data[i][j] = metric;
        }
    }
    pthread_mutex_unlock(&backbuf_mutex);
    frame++;
}

void *freenect_threadfunc(void *arg)
{
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

    int res = freenect_start_depth(f_dev);
    if (res !=0)
    {
        std::cout<<"freenect_start_depth failed!"<<std::endl;
    }

    while ( !die && freenect_process_events(f_ctx) >= 0 )
    {
        freenect_raw_tilt_state* state;
        state = freenect_get_tilt_state(f_dev);
        double dx,dy,dz;
        freenect_get_mks_accel(state, &dx, &dy, &dz);
        std::cout<<"\r frame: "<<frame<<" - mks acc: "<<dx<<" "<<dy<<" "<<dz<<"\r";
    }

    std::cout<<"\nshutting down streams...\n";

    freenect_stop_depth(f_dev);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    std::cout<<"-- done!\n";
    return NULL;
}




int run(int argc, char **argv)
{
	//std::cout <<"\nspinViewer launching..." << std::endl;
    using namespace spin;
	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	std::string userID;
	
	bool fullscreen = false;
	bool hideCursor=false;

	double maxFrameRate = 60;
	
	int x=50;
	int y=50;
	int width=640;
	int height=480;
	int screen=-1;

	
	std::string sceneID = spin.getSceneID();
	
	// *************************************************************************
    // kinect init
    for (int i=0; i<240; i++)
        for (int j=0; j<320; j++)
            depth_data[i][j] = 0;

    if (freenect_init(&f_ctx, NULL) < 0)
    {
        std::cout<<"freenect_init() failed\n";
        return 1;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
    {
            std::cout<<"Could not open device\n";
            return 1;
    }
        
    int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    if (res)
    {
        printf("pthread_create failed\n");
        return 1;
    }


	
    // *************************************************************************

	// get arguments:
	osg::ArgumentParser arguments(&argc,argv);
	
	// set up the usage document, which a user can acess with -h or --help
    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is an example viewer for SPIN that adds Kinect point cloud viewing via the freenect library.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options]");
	
    // add generic spin arguments (for address/port setup, scene-id, etc)
    spinListener.addCommandLineOptions(&arguments);

    // arguments specific to spinviewer:	
    arguments.getApplicationUsage()->addCommandLineOption("--user-id <uniqueID>", "Specify a user ID for this viewer (Default: <this computer's name>)"); 
	arguments.getApplicationUsage()->addCommandLineOption("--fullscreen", "Expand viewer to fullscreen");
    arguments.getApplicationUsage()->addCommandLineOption("--hide-cursor", "Hide the mouse cursor");
	arguments.getApplicationUsage()->addCommandLineOption("--window <x y w h>", "Set the position (x,y) and size (w,h) of the viewer window (Default: 50 50 640 480)");
    arguments.getApplicationUsage()->addCommandLineOption("--screen <num>", "Screen number to display on (Default: ALLSCREENS)");
	arguments.getApplicationUsage()->addCommandLineOption("--framerate <num>", "Set the maximum framerate (Default: not limited)");
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

   
    if (arguments.read("--fullscreen")) fullscreen=true;
    if (arguments.read("--hide-cursor")) hideCursor=true;
	while (arguments.read("--window",x,y,width,height)) {}
	while (arguments.read("--screen",screen)) {}
	while (arguments.read("--framerate",maxFrameRate)) {}


	// For testing purposes, we allow loading a scene with a commandline arg:
	osg::ref_ptr<osg::Node> argScene = osgDB::readNodeFiles(arguments);

	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	osgViewer::CompositeViewer viewer = osgViewer::CompositeViewer(arguments);
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


    osgViewer::ViewerBase::Windows windows;
    osgViewer::ViewerBase::Windows::iterator wIter;
    viewer.getWindows(windows);
    for (wIter=windows.begin(); wIter!=windows.end(); wIter++)
    {
      	(*wIter)->setWindowName("spinviewer " + spin.getUserID() + "@" + spin.getSceneID());
	    if (hideCursor) (*wIter)->useCursor(false);

    }

    view->setSceneData(spin.sceneManager->rootNode.get());

	view->addEventHandler(new osgViewer::StatsHandler);
	//view->addEventHandler(new osgViewer::ThreadingHandler);
	view->addEventHandler(new osgViewer::WindowSizeHandler);

	view->addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	view->addEventHandler( new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()) );

	//view->setLightingMode(osg::View::NO_LIGHT);
	view->setLightingMode(osg::View::HEADLIGHT);
	//view->setLightingMode(osg::View::SKY_LIGHT);

    view->setCameraManipulator(new ViewerManipulator());
    
	
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

    // Try to subscribe with the current (default or manually specified) TCP
    // subscription information. If this fails, it's likely because the server
    // is not online, but when it comes online, it will send a userRefresh
    // message which will invoke another subscription attempt:
    spinListener.subscribe();

	// ask for refresh:
	spin.SceneMessage("s", "refresh", LO_ARGS_END);
	osg::Timer_t lastFrameTick = osg::Timer::instance()->tick();
	double minFrameTime = 1.0 / maxFrameRate;

    std::cout << "\nviewer is READY" << std::endl;

    spin.SceneMessage("sss", "createNode", "kinectGroup", "GroupNode", LO_ARGS_END);
    spin::GroupNode *kinectGroup = dynamic_cast<spin::GroupNode*>(spin.sceneManager->getOrCreateNode("kinectGroup", "GroupNode"));

    osg::PositionAttitudeTransform *kinectPAT = new osg::PositionAttitudeTransform();
    osg::Node* lps = createLightPointsDatabase(false);
    kinectGroup->addChild(lps);
    kinectPAT->addChild(lps);
    spin.sceneManager->worldNode->addChild(kinectPAT);
    
	// program loop:
	while(not viewer.done())
	{
		if (spinListener.isRunning())
		{
			double dt = osg::Timer::instance()->delta_s(lastFrameTick, osg::Timer::instance()->tick());

			if (dt >= minFrameTime)
			{
			    kinectGroup->getAttachmentNode()->removeChild(lps);
                lps = createLightPointsDatabase(false);
                kinectGroup->getAttachmentNode()->addChild(lps);
			
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
