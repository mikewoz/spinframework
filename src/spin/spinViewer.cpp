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
#include <osg/TextureCubeMap>
#include <osg/ClampColor>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>
#include <osg/Timer>

#include "config.h"
#include "viewermanipulator.h"
#include "spinutil.h"
#include "spinapp.h"
#include "spinclientcontext.h"
#include "osgutil.h"
#include "groupnode.h"
#include "scenemanager.h"
#include "shapenode.h"
#include "compositeviewer.h"

#ifdef HAVE_SPNAV_H
#include "spnav.h"
#endif

//#include "dofppu.h"

//#define VELOCITY_SCALAR 0.005
//#define SPIN_SCALAR 0.01
#define VELOCITY_SCALAR 0.004
#define SPIN_SCALAR 0.007

#define DISPLAY_FPS_TERMINAL 1
#define DISPLAY_FPS_WINDOW 2
#define DISPLAY_FPS_LOG 3

extern pthread_mutex_t sceneMutex;

int run(int argc, char **argv)
{
    //std::cout <<"\nspinViewer launching..." << std::endl;
    using namespace spin;
    spinClientContext spinListener;
    spinApp &spin = spinApp::Instance();

    std::string userID;
    bool mover = true;

    bool ssao = false;
    bool dof = false;
    bool mblur = false;
    bool outline = false;
    bool mask = false;
    bool shader = false;
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

    int displayFps = 0;
    double displayFpsFreq = 1.0;
    std::string displayFpsWhere = "";
    int displayFpsNb = 0;
    osgViewer::Viewer* fpsViewer = 0;
    osgText::Text* fpsWindowText = 0;

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
    arguments.getApplicationUsage()->addCommandLineOption("--framerate <num>", "Set the maximum framerate (Default: 60fps). Set to -1 for no limit (except VSync)");
    arguments.getApplicationUsage()->addCommandLineOption("--multisamples <num>", "Set the level of multisampling for antialiasing (Default: 4)");
    arguments.getApplicationUsage()->addCommandLineOption("--disable-camera-controls", "Disable mouse-based camera controls for this user");
    arguments.getApplicationUsage()->addCommandLineOption("--cache", "Enable caching for all models and textures. ie, disable automatic unloading of culled media");
    arguments.getApplicationUsage()->addCommandLineOption("--dof", "Enables depth of field effect");
    arguments.getApplicationUsage()->addCommandLineOption("--ssao", "Enables screen space ambient occlusion effect");
    arguments.getApplicationUsage()->addCommandLineOption("--mblur", "Enables motion blur effect");
    arguments.getApplicationUsage()->addCommandLineOption("--outline <filename>", "Enables the outline effect");
    arguments.getApplicationUsage()->addCommandLineOption("--shader", "Activates an additionnal PPU shader, to specify through OSC messaging");
    arguments.getApplicationUsage()->addCommandLineOption("--mask", "Enables the masking effect (from a secondary camera render)");
    arguments.getApplicationUsage()->addCommandLineOption("--display-fps < [terminal|window|log] t >", "Display the framerate on the terminal, in a window or in the server's logs.  The fps is refreshed every t seconds.");
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
    if (arguments.read("--ssao")) ssao=true;
    if (arguments.read("--mblur")) mblur=true;
    if (arguments.read("--outline")) outline=true;
    if (arguments.read("--mask")) mask=true;
    if (arguments.read("--shader")) shader=true;
    while (arguments.read("--window",x,y,width,height)) {}
    while (arguments.read("--screen",screen)) {}
    while (arguments.read("--framerate",maxFrameRate)) {}
    while (arguments.read("--multisamples",multisamples)) {}
    if (arguments.read("--disable-camera-controls")) mover=false;
    if (arguments.read("--grid")) grid=true;

    while( arguments.read("--display-fps", displayFpsWhere , displayFpsFreq ) ) {}

    //displayFps = DISPLAY_FPS_TERMINAL; }
    //while( arguments.read("--display-fps" , displayFpsFreq ) ) { displayFps = DISPLAY_FPS_WINDOW; }

    if (arguments.read("--cache"))
    {
        osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options();
        options->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_ALL);
        osgDB::Registry::instance()->setOptions(options);
    }

 


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
    // display fps

    if ( displayFpsWhere == "terminal" ) {
        displayFps = DISPLAY_FPS_TERMINAL;
        //printf("terminal!, %f\n", displayFpsFreq);
        
    } else if ( displayFpsWhere == "window" ) {
        int fpsw = 150;
        int fpsh = 50;
        displayFps = DISPLAY_FPS_WINDOW;
        //printf("window!, %f\n", displayFpsFreq);

        fpsViewer = new osgViewer::Viewer();
        fpsViewer->setUpViewInWindow(0,0,fpsw,fpsh);

        osg::Camera* camera = fpsViewer->getCamera();
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        camera->setProjectionMatrixAsOrtho2D(0,fpsw,0,fpsh);
        camera->setViewMatrix(osg::Matrix::identity());
        //camera->setClearMask(GL_DEPTH_BUFFER_BIT);
        //camera->addChild(createHUDText());
        camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);


        osgText::Font* font = osgText::readFontFile("arial.ttf");
        osg::Vec4 layoutColor(1.0f,1.0f,1.0f,1.0f);
        float layoutCharacterSize = 30.0f;    
                     
        fpsWindowText = new osgText::Text;
        fpsWindowText->setFont(font);
        fpsWindowText->setColor(layoutColor);
        fpsWindowText->setCharacterSize(layoutCharacterSize);
        //fpsWindowText->setPosition(osg::Vec3(50, 15, 0));
        fpsWindowText->setPosition(osg::Vec3(fpsw/2, fpsh/2, 0));
        fpsWindowText->setLayout(osgText::Text::LEFT_TO_RIGHT);
        fpsWindowText->setAlignment(osgText::Text::CENTER_CENTER);
        //fpsWindowText->setAlignment(osgText::Text::LEFT_TOP);
        fpsWindowText->setFontResolution(30,30);
        //fpsWindowText->setDrawMode(osgText::Text::TEXT|osgText::Text::ALIGNMENT|osgText::Text::BOUNDINGBOX);
        fpsWindowText->setText("FPS");
        osg::Geode* g = new osg::Geode();
        g->addDrawable( fpsWindowText );
        
        fpsViewer->setSceneData( g );
        fpsViewer->realize();
        fpsViewer->frame();
    } else if ( displayFpsWhere == "log" ) {
        displayFps = DISPLAY_FPS_LOG;
    } else {
        displayFps = 0;
    }

    if ( displayFps ) viewer.getViewerStats()->collectStats( "frame_rate", true );




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
        std::cout << "Failed to connect to the Space Navigator" << std::endl;
    } else
    {
        viewer.setSpaceNavigatorNode(spin.getUserID());
        std::cout << "Space Navigator is enabled and sending messages to: " << spin.getUserID() << std::endl;
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

        if (dof || ssao || mblur || outline || mask || shader)
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
    for (unsigned int i=0; i<viewer.getNumViews(); i++)
    {
        ViewerManipulator *vm = dynamic_cast<ViewerManipulator*>(viewer.getView(i)->getCameraManipulator());
        if (vm)
        {
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
        double left, right, bottom, top, zNear, zFar, fovy, aspectRatio;

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
            std::cout << "   Proj as Frustum:  " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;

            (*iter)->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
            std::cout << "   Proj as Perspect: fovy=" << fovy << ", aspectRatio=" << aspectRatio << ", clip: " << zNear << "-" << zFar << std::endl;

            (*iter)->getProjectionMatrixAsOrtho(left, right, bottom, top, zNear, zFar);
            std::cout << "   Proj as Ortho:    " << left << "," << right << "," << top << "," << bottom << "  clip: " << zNear << "-" << zFar << std::endl;



            (*iter)->getViewMatrixAsLookAt (eye, center, up);
            std::cout << "   Camera LookAt:    eye=(" << eye.x() << "," << eye.y() << "," << eye.z() << ") center=(" << center.x() << "," << center.y() << "," << center.z() << ") up=(" << up.x() << "," << up.y() << "," << up.z() << ")" << std::endl;

            const osg::Viewport *viewport = (*iter)->getViewport();
            if (viewport)
                std::cout << "   Camera viewport:  pos=(" << viewport->x() << "," << viewport->y() << ") size=" << viewport->width() << "x" << viewport->height() << std::endl;
            else
                std::cout << "   Camera viewport:  INVALID" << std::endl;

            //osg::View *v = (*iter)->getView();
            //std::cout << "   view numSlaves:   " << v->getNumSlaves() << std::endl;
        }
        for (unsigned int i=0; i<viewer.getNumViews(); i++)
        {
            osg::View *v = viewer.getView(i);
            std::cout << "View " << v->getName() << " has " << v->getNumSlaves() << " slaves" << std::endl;

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

    if (dof || ssao || mblur || outline || mask || shader)
    {
        unsigned int lEffects = 0x0000;
        if(dof)
            lEffects |= PPU_DOF;
        if(ssao)
            lEffects |= PPU_SSAO;
        if(mblur)
            lEffects |= PPU_MOTIONBLUR;
        if(outline)
            lEffects |= PPU_OUTLINE;
        if(mask)
            lEffects |= PPU_MASK;
        if(shader)
            lEffects |= PPU_SHADER;

        //viewer.frame();
        viewer.viewerInit(); // TODO: move this in a better place
        viewer.initializePPU(lEffects);

        osg::ClampColor* clamp = new osg::ClampColor();
        clamp->setClampVertexColor(GL_FALSE);
        clamp->setClampFragmentColor(GL_FALSE);
        clamp->setClampReadColor(GL_FALSE);

        spin.sceneManager_->worldNode->getOrCreateStateSet()->setAttribute(clamp, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
    }

    // depth-of-field effect:
    /*if (dof)
    {
        viewer.frame();
        viewer.initializePPU(CompositeViewer::dofEffect);

        // disable color clamping, because we want to work on real hdr values
        osg::ClampColor* clamp = new osg::ClampColor();
        clamp->setClampVertexColor(GL_FALSE);
        clamp->setClampFragmentColor(GL_FALSE);
        clamp->setClampReadColor(GL_FALSE);

        // make it protected and override, so that it is done for the whole rendering pipeline
        spin.sceneManager_->worldNode->getOrCreateStateSet()->setAttribute(clamp, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
    }
    else if(ssao)
    {
        viewer.frame();
        viewer.initializePPU(CompositeViewer::ssaoEffect);

        // disable color clamping, because we want to work on real hdr values
        osg::ClampColor* clamp = new osg::ClampColor();
        clamp->setClampVertexColor(GL_FALSE);
        clamp->setClampFragmentColor(GL_FALSE);
        clamp->setClampReadColor(GL_FALSE);

        // make it protected and override, so that it is done for the whole rendering pipeline
        spin.sceneManager_->worldNode->getOrCreateStateSet()->setAttribute(clamp, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);
    }*/



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

            if(maxFrameRate = -1)
                viewer.frame();
            else
            {
                if (dt >= minFrameTime)
                    {
                        viewer.frame();
/*
                    // poll the space navigator:
                    viewer.updateSpaceNavigator();

                    viewer.advance();
                    viewer.eventTraversal();
                    pthread_mutex_lock(&sceneMutex);
                    spin.sceneManager_->update();
                    viewer.updateTraversal();
                    viewer.renderingTraversals();
                    pthread_mutex_unlock(&sceneMutex);
*/

                    // save time when the last time a frame was rendered:
                    lastFrameTick = osg::Timer::instance()->tick();
                    dt = 0;
                }

                unsigned int sleepTime;
                if (!recv) sleepTime = static_cast<unsigned int>(1000000.0*(minFrameTime-dt));
                else sleepTime = 0;
                if (sleepTime > 100) sleepTime = 100;

                if (!recv) OpenThreads::Thread::microSleep(sleepTime);
            }

            if ( displayFps && viewer.getViewerFrameStamp()->getReferenceTime() > displayFpsFreq * displayFpsNb ) {
                double d;
                viewer.getViewerStats()->getAveragedAttribute( "Frame rate", d );
                if ( displayFps == DISPLAY_FPS_TERMINAL ) {
                    printf( "Frame rate: %f               \r", d );
                    fflush( stdout );
                } else if ( displayFps == DISPLAY_FPS_WINDOW ) {                    
                    std::ostringstream oss;
                    oss << d;
                    fpsWindowText->setText( oss.str() );
                    fpsViewer->frame();
                } else { // displayFps == DISPLAY_FPS_LOG
                    spin.NodeMessage( spin.getUserID().c_str(), "si", "FPS", (int)d, SPIN_ARGS_END );
                }
                displayFpsNb++;
            }

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
    // *************************************************************************
    // If no command line arguments were passed, check if there is an args file
    // at ~/.spinframework/args and override argc and argv with those:
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
