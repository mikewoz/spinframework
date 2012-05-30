
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>
#include <osg/GraphicsContext>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osg/Timer>
#include <osg/ClampColor>


#include <boost/algorithm/string.hpp>

#include <spin/config.h>
#include <spin/ViewerManipulator.h>
#include <spin/spinUtil.h>
#include <spin/spinApp.h>
#include <spin/spinClientContext.h>
#include <spin/osgUtil.h>
#include <spin/GroupNode.h>
#include <spin/SceneManager.h>
#include <spin/ShapeNode.h>
#include <spin/TextNode.h>


#include "dofppu.h"

extern pthread_mutex_t sceneMutex;


class SpinViewer : public osgViewer::CompositeViewer
//class SpinViewer : public osgViewer::Viewer
{
    private:
        osg::ref_ptr<osgPPU::Processor> mProcessor;

        float mOldTime;
        DoFRendering mDoFSetup;
        bool mbInitialized;

    public:
        //! Default construcotr
        SpinViewer(osg::ArgumentParser& args) : osgViewer::CompositeViewer(args)
        //SpinViewer(osg::ArgumentParser& args) : osgViewer::Viewer(args)
        {
            mbInitialized = false;
            mOldTime = 0.0f;
        }

        //! Get the ppu processor
        osgPPU::Processor* getProcessor() { return mProcessor.get(); }

        //! Create camera resulting texture
        static osg::Texture* createRenderTexture(int tex_width, int tex_height, bool depth)
        {
            // create simple 2D texture
            osg::Texture2D* texture2D = new osg::Texture2D;
            texture2D->setTextureSize(tex_width, tex_height);
            texture2D->setResizeNonPowerOfTwoHint(false);
            texture2D->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
            texture2D->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
            texture2D->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_BORDER);
            texture2D->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_BORDER);
            texture2D->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

            // setup float format
            if (!depth)
            {
                
                texture2D->setInternalFormat(GL_RGBA16F_ARB);
                texture2D->setSourceFormat(GL_RGBA);
                texture2D->setSourceType(GL_FLOAT);
                
                /*
                texture2D->setInternalFormat(GL_RGBA32I_EXT);
                texture2D->setSourceFormat(GL_RGBA_INTEGER_EXT);
                texture2D->setSourceType(GL_BYTE);
                */
            }
            else{
                texture2D->setInternalFormat(GL_DEPTH_COMPONENT);
            }

            return texture2D;
        }

        //! Setup the camera to do the render to texture
        void setupCamera(osg::Viewport* vp)
        {
            // setup viewer's default camera
            //osg::Camera* camera = getCamera();
            osg::Camera* camera = this->getView(0)->getCamera();
            
            //camera->setViewport(0,0,(int)vp->width(), (int)vp->height()); 
            
            // TODO: setup for all cameras:
            /*
            osgViewer::Viewer::Cameras cameras;
            viewer.getCameras(cameras);
            for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
            {
                //(*iter)->setNearFarRatio(0.0001f);
                
            }
            */
            
            std::cout << std::endl;
            std::cout << "---------------------------------------" << std::endl;
            std::cout << "setting up camera with wxh=" << (int)vp->width()<<"x"<<(int)vp->height() << std::endl;
            

            // create texture to render to
            osg::Texture* texture = createRenderTexture((int)vp->width(), (int)vp->height(), false);
            osg::Texture* depthTexture = createRenderTexture((int)vp->width(), (int)vp->height(), true);

            // set up the background color and clear mask.
            camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
            camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // set viewport
            camera->setViewport(vp);
            camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
            camera->setProjectionMatrixAsPerspective(20.0, vp->width()/vp->height(), 0.1, 100.0);

            // tell the camera to use OpenGL frame buffer object where supported.
            camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);

            // attach the texture and use it as the color buffer.
            camera->attach(osg::Camera::COLOR_BUFFER, texture);
            camera->attach(osg::Camera::DEPTH_BUFFER, depthTexture);
        }

        //! Just setup some stuff
        void viewerInit()
        {
            // propagate the method
            osgViewer::CompositeViewer::viewerInit();
            //osgViewer::Viewer::viewerInit();

            // setup data
            //setupCamera(getCamera()->getViewport());
            setupCamera(this->getView(0)->getCamera()->getViewport());
            
            // add ppu processor into the scene graph
            osg::Group* group = new osg::Group();
            //group->addChild(getSceneData());
            group->addChild(this->getView(0)->getSceneData());
            
            //setSceneData(group);
            this->getView(0)->setSceneData(group);

        }

        //! Setup osgppu for rendering
        void initializePPU()
        {
           // if already initialized then just do nothing
            if (mbInitialized == false)
                mbInitialized = true;
            else
                return;

            mProcessor = new osgPPU::Processor();
            //dynamic_cast<osg::Group*>(getSceneData())->addChild(mProcessor.get());
            dynamic_cast<osg::Group*>(getView(0)->getSceneData())->addChild(mProcessor.get());

            // initialize the post process
            //mProcessor->setCamera(getCamera());
            mProcessor->setCamera(this->getView(0)->getCamera());
            mProcessor->setName("Processor");
            mProcessor->dirtyUnitSubgraph();

            // we want to simulate hdr rendering, hence setup the pipeline
            // for the hdr rendering
            osgPPU::Unit* lastUnit = NULL;

            //osg::setNotifyLevel(osg::DEBUG_FP);
            mDoFSetup.createDoFPipeline(mProcessor.get(), lastUnit, 0.1, 100.0);
            //osg::setNotifyLevel(osg::FATAL);
    

            // add a text ppu after the pipeline is setted up
            if (0){
                osgPPU::UnitText* fpstext = new osgPPU::UnitText();
                fpstext->setName("FPSTextPPU");
                fpstext->setSize(44);
                fpstext->setText("Example DoF-pipeline from a .ppu file");
                fpstext->setPosition(0.01, 0.95);
                lastUnit->addChild(fpstext);
            }

            // As a last step we setup a ppu which do render the content of the result
            // on the screenbuffer. This ppu MUST be as one of the last, otherwise you
            // will not be able to get results from the ppu pipeline
            osgPPU::UnitOut* ppuout = new osgPPU::UnitOut();
            ppuout->setName("PipelineResult");
            ppuout->setInputTextureIndexForViewportReference(-1); // need this here to get viewport from camera
            lastUnit->addChild(ppuout);

            // write pipeline to a file
            //osgDB::writeObjectFile(*mProcessor, "dof.ppu");
        }

        //! Update the frames
        void frame(double f = USE_REFERENCE_TIME)
        {
            // update default viewer
            // this should also update the post processing graph
            // since it is attached to the camera
            //osgViewer::Viewer::frame(f);
            osgViewer::CompositeViewer::frame(f);
            
            // initilize PPU if it was not done before
            initializePPU();

            // compute frame time
            float frameTime = elapsedTime() - mOldTime;
            mOldTime = elapsedTime();

            // print also some info about the fps number
            if (0)
            {
                osgPPU::UnitText* ppu = dynamic_cast<osgPPU::UnitText*>(mProcessor->findUnit("FPSTextPPU"));
                if (ppu)
                {
                    char txt[64];
                    sprintf(txt, "FPS: %4.2f", 1.0 / frameTime);
                    ppu->setText(txt);
                }
            }
        }
        
        //int run()


};

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
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
    
    using namespace spin;
	spinClientContext spinListener;
	spinApp &spin = spinApp::Instance();

	std::string userID;

	double maxFrameRate = 60;
	
	std::string sceneID = spin.getSceneID();
	
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

	// *************************************************************************
	// PARSE ARGS:

    if (!spinListener.parseCommandLineOptions(&arguments))
        return 0;
    
	osg::ArgumentParser::Parameter param_userID(userID);
	arguments.read("--user-id", param_userID);
    if (not userID.empty())
        spin.setUserID(userID);

    double fov = 1.0;
    while (arguments.read("--fov", fov)) {}

   
	// *************************************************************************
	// construct the viewer:
	// (note, this constructor gets rid of some additional args)

	SpinViewer viewer = SpinViewer(arguments);
	//viewer.setThreadingModel(osgViewer::CompositeViewer::CullDrawThreadPerContext);
	//viewer.setThreadingModel(osgViewer::Viewer::CullDrawThreadPerContext);
	viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

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

    /*
    viewer.setSceneData(spin.sceneManager->rootNode.get());
    viewer.setUpViewInWindow(50,50,800,600,0);
    viewer.getCamera()->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 0.0));
    viewer.setLightingMode(osg::View::HEADLIGHT);
    osg::ref_ptr<ViewerManipulator> manipulator = new ViewerManipulator();
	viewer.setCameraManipulator(manipulator.get());
    */

	
	osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    view->setSceneData(spin.sceneManager->rootNode.get());
    view->setUpViewInWindow(50,50,800,600,0);
	viewer.addView(view.get());

	view->getCamera()->setClearColor(osg::Vec4(0.0, 0.0, 0.0, 0.0));
	view->addEventHandler(new osgViewer::StatsHandler);
	view->addEventHandler(new osgViewer::ThreadingHandler);
	view->addEventHandler(new osgViewer::WindowSizeHandler);
	view->addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));
	view->addEventHandler( new osgGA::StateSetManipulator(view->getCamera()->getOrCreateStateSet()) );

	//view->setLightingMode(osg::View::NO_LIGHT);
	view->setLightingMode(osg::View::HEADLIGHT);
	//view->setLightingMode(osg::View::SKY_LIGHT);

	osg::ref_ptr<ViewerManipulator> manipulator = new ViewerManipulator();
	view->setCameraManipulator(manipulator.get());
    



	// *************************************************************************
    // register an extra OSC callback so that we can spy on OSC messages:
    
    /*
    std::vector<lo_server>::iterator servIter;
    for (servIter = spin.getContext()->lo_rxServs_.begin(); servIter != spin.getContext()->lo_rxServs_.end(); ++servIter)
    {
    	lo_server_add_method((*servIter), NULL, NULL, message_callback, NULL);
    }
    */




	// *************************************************************************
	// For testing purposes, we allow loading a scene with a commandline arg:
    //osg::setNotifyLevel(osg::INFO);

    spin.sceneManager->createNode("grid", "GridNode");
    for (unsigned int i=0; i<50; i++)
    {
        osg::ref_ptr<ShapeNode> shp = dynamic_cast<spin::ShapeNode*>(spin.sceneManager->createNode("shp"+stringify((int)i), "ShapeNode"));
        
        shp->setTranslation(random(-10.0,10.0), random(1.0,10.0), random(-5.0,5.0));
        shp->setOrientation(random(0,360), random(0,360), random(0,360));
    
        shp->setColor(random(0.0,1.0), random(0.0,1.0), random(0.0,1.0), 1.0);
    
    }
    

    //spin.sceneManager->worldNode->addChild(scene.get());




    // disable color clamping, because we want to work on real hdr values
    osg::ClampColor* clamp = new osg::ClampColor();
    clamp->setClampVertexColor(GL_FALSE);
    clamp->setClampFragmentColor(GL_FALSE);
    clamp->setClampReadColor(GL_FALSE);

    // make it protected and override, so that it is done for the whole rendering pipeline
    spin.sceneManager->worldNode->getOrCreateStateSet()->setAttribute(clamp, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE | osg::StateAttribute::PROTECTED);


    /*
    osgViewer::Viewer::Cameras cameras;
    viewer.getCameras(cameras);
    for (osgViewer::Viewer::Cameras::iterator iter = cameras.begin(); iter != cameras.end(); ++iter)
    {
        (*iter)->setNearFarRatio(0.0001f);
        double fovy, aspectRatio, zNear, zFar;
        //(*iter)->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
        (*iter)->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
        std::cout << "fovy= " << fovy << " ... setting to " << fov << std::endl;
        (*iter)->setProjectionMatrixAsPerspective(fov, aspectRatio, zNear, zFar);
        
    }
    */

/*
    
    osg::ref_ptr<osgPPU::Processor> mProcessor;


    int tex_width = 800;
    int tex_height = 600;

    // create simple 2D texture
    osg::Texture2D* texture2D = new osg::Texture2D;
    texture2D->setTextureSize(tex_width, tex_height);
    texture2D->setResizeNonPowerOfTwoHint(false);
    texture2D->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture2D->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    texture2D->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_BORDER);
    texture2D->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_BORDER);
    texture2D->setBorderColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    // setup float format
    if (!depth)
    {
        texture2D->setInternalFormat(GL_RGBA16F_ARB);
        texture2D->setSourceFormat(GL_RGBA);
        texture2D->setSourceType(GL_FLOAT);
    }else{
        texture2D->setInternalFormat(GL_DEPTH_COMPONENT);
    }
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


	// *************************************************************************
	// start threads:
    osg::setNotifyLevel(osg::DEBUG_FP);
    
    //return viewer.run();
    
    //viewer.viewerInit();
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


    std::cout << "\nspinviewer is READY" << std::endl;

    // run viewer
    //return viewer.run();

    // force the initialize
    viewer.frame();
    viewer.initializePPU();


	// program loop:
	while(not viewer.done())
	{
		osg::setNotifyLevel(osg::FATAL);
		if (spinListener.isRunning())
		{
			double dt = osg::Timer::instance()->delta_s(lastFrameTick, osg::Timer::instance()->tick());

			if (dt >= minFrameTime)
			{
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

            //viewer.setCameraManipulator(NULL);
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
