
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

#include <boost/algorithm/string.hpp>

#include <spin/ViewerManipulator.h>
#include <spin/spinUtil.h>
#include <spin/spinApp.h>
#include <spin/spinClientContext.h>
#include <spin/osgUtil.h>
#include <spin/GroupNode.h>
#include <spin/SceneManager.h>
#include <spin/ShapeNode.h>
#include <spin/config.h>


#include <btBulletDynamicsCommon.h>

extern pthread_mutex_t sceneMutex;

static btTransform trans;

float phys2world_scale = 100.0;
extern ContactAddedCallback gContactAddedCallback;

class BallUpdateCallback: public osg::NodeCallback
{
    private:
        btRigidBody *_body;
        osg::Matrixf previousMatrix_;

    public:
        BallUpdateCallback(btRigidBody *body) :
            _body(body)
        {
            previousMatrix_ = osg::Matrix::identity();
        }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            btScalar m[16];

            btDefaultMotionState* myMotionState = (btDefaultMotionState*) _body->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

            osg::Matrixf mat(m);

            
            // to move our ShapeNode according to gravity, we do something like this?:
            
            osg::Matrixf diff = osg::Matrix::inverse(previousMatrix_) * mat;
            spin::ShapeNode *shp = dynamic_cast<spin::ShapeNode*>(node);
            /*
            if (shp)
            {
                //osg::Vec3 t = diff.getTrans() - osg::Vec3(2,0,0);
                //shp->translate(t.x(),t.y(),t.z());
                shp->setTranslation(mat.getTrans().x(),mat.getTrans().y(),mat.getTrans().z());
            }
            else
            {
                osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform*>(node);
                if (pat)
                {
                    pat->setPosition(mat.getTrans());
                    pat->setAttitude(mat.getRotate());

                    //std::cout << mat.getTrans().x() << ", " << mat.getTrans().y() << ", " << mat.getTrans().z() << std::endl;
                }
            }
            */
            
            //_body->getMotionState()->setWorldTransform(trans);
            _body->setWorldTransform(trans);
            
            previousMatrix_ = mat;
            
            traverse(node, nv);
        }
};

bool materialCombinerCallback(btManifoldPoint& cp,
                              const btCollisionObject* colObj0,
                              int partId0,
                              int index0,
                              const btCollisionObject* colObj1,
                              int partId1,
                              int index1)
{
    btVector3 impact_point = cp.getPositionWorldOnA();

    if (!colObj0->isStaticObject() && colObj0->isActive())
    {
        osg::Node *n0 = static_cast<osg::Node*>(colObj0->getUserPointer());
        osg::Node *n1 = static_cast<osg::Node*>(colObj1->getUserPointer());
    
        std::cout << "Hit at ["<< impact_point.getX() << ", " << impact_point.getY() << ", " << impact_point.getZ() << "] "<< std::endl;
        //if (n0 && !n0->getName().empty()) std::cout << " obj0: " << n0->getName() << std::endl;
        if (n1 && !n1->getName().empty()) std::cout << " obj1: " << n1->getName() << std::endl;
        
        //std::cout << "Hit between " << static_cast<osg::Node *> (colObj0->getUserPointer())->getName() << " and " << static_cast<osg::Node *> (colObj1->getUserPointer())->getName() << " at ["<< impact_point.getX() << ", " << impact_point.getY() << ", " << impact_point.getZ() << "] "<< std::endl;
    }

    return false;
}

btRigidBody* createRigidBody(btDynamicsWorld *world,
                             float mass,
                             const btTransform& startTransform,
                             btCollisionShape* shape)
{
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody* body = new btRigidBody(mass, myMotionState, shape, localInertia);

    world->addRigidBody(body);

    return body;
}





int shpCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    spin::spinApp &spin = spin::spinApp::Instance();

    // make sure there is at least one argument (ie, a method to call):
    if (!argc) return 1;

    // get the method (argv[0]):
    std::string theMethod;
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else return 1;

    // parse the rest of the args:
    std::vector<float> floatArgs;
    std::vector<const char*> stringArgs;
    for (int i=1; i<argc; i++)
    {
        if (lo_is_numerical_type((lo_type)types[i]))
        {
            floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
		} else {
            stringArgs.push_back( (const char*) argv[i] );
        }
    }
    
    // reset the transform
    trans.setIdentity();
    
    // update from spin state:
    spin::ShapeNode *shp = dynamic_cast<spin::ShapeNode*>(spin.sceneManager->getNode("shp"));
    if (shp)
    {
        btVector3 pos(shp->getTranslation().x(), shp->getTranslation().y(), shp->getTranslation().z());
        btQuaternion quat(shp->getOrientationQuat().x(), shp->getOrientationQuat().y(), shp->getOrientationQuat().z(), shp->getOrientationQuat().w());
    
        trans.setOrigin(pos);
        trans.setRotation(quat);
    }
    
    // update state of physics object:
    
    btRigidBody* shp_body = (btRigidBody*)user_data;  
    if (shp_body && shp)
    {
        if ((theMethod=="setTranslation") && (floatArgs.size()==3))
        {
           // update physics object:
            
            std::cout << "got /shp setTranslation " <<floatArgs[0]<<","<<floatArgs[1]<<","<<floatArgs[2]<< std::endl;
            
            btVector3 newpos(floatArgs[0],floatArgs[1],floatArgs[2]);
            trans.setOrigin(newpos);
            
            // CRASH:
            // maybe we need to just save this somewhere and apply it during the update callback?
            //shp_body->proceedToTransform(trans);
            //shp_body->setWorldTransform(trans);
            //shp_body->proceedToTransform(worldxform);
            
        } else if ((theMethod=="setOrientation") && (floatArgs.size()==3))
        {
            osg::Quat q = osg::Quat( osg::DegreesToRadians(floatArgs[0]), osg::Vec3d(1,0,0),
                                     osg::DegreesToRadians(floatArgs[1]), osg::Vec3d(0,1,0),
                                     osg::DegreesToRadians(floatArgs[2]), osg::Vec3d(0,0,1));
            btQuaternion quat(q.x(),q.y(),q.z(),q.w());
            trans.setRotation(quat);
        }
    }
		 
    return 1;
}




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




    btVector3 worldAabbMin(-1000, -1000, -1000);
    btVector3 worldAabbMax(1000, 1000, 1000);
    const int maxProxies = 32766;

    // register global callback
    gContactAddedCallback = materialCombinerCallback;

    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;
    btAxisSweep3 *broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btDynamicsWorld *m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    
    //m_dynamicsWorld->setGravity(btVector3(0, 0, -9.81));
    m_dynamicsWorld->setGravity(btVector3(0, 0, -10.0));

    // ShapeNode

    osg::ref_ptr<ShapeNode> shp = dynamic_cast<spin::ShapeNode*>(spin.sceneManager->createNode("shp", "ShapeNode"));
    shp->setTranslation(-2.0, 0.0, 5.0);
    shp->setOrientation(0.0, 45.0, 0.0);
    shp->setInteractionMode(spin::GroupNode::DRAG);
    std::cout << "created shp with osg name: " << shp->getName() << std::endl;
    if (1)
    {
        btRigidBody* shp_body;
        btCollisionShape *shp_collisionObj;       
        shp->setTranslation(-2.0, 0.0, 2.0);
        shp->setOrientation(0.0, 45.0, 0.0);
        shp->setInteractionMode(spin::GroupNode::DRAG);
    
        shp_collisionObj = new btBoxShape(btVector3(1, 1, 1));

        // TODO: use global position: shp->getGlobalMatrix
        btVector3 pos(shp->getTranslation().x(), shp->getTranslation().y(), shp->getTranslation().z());
        btQuaternion quat(shp->getOrientationQuat().x(), shp->getOrientationQuat().y(), shp->getOrientationQuat().z(), shp->getOrientationQuat().w());
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(pos);
        //trans.setRotation(quat);
        btScalar mass = 1.f;

        shp_body = createRigidBody(m_dynamicsWorld, mass, trans, shp_collisionObj);
        shp_body->setUserPointer(shp.get());
        shp_body->setCollisionFlags(shp_body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        //shp_body->setCcdMotionThreshold(0.5);
        //shp_body->setCcdSweptSphereRadius(0.2 * 0.5);
        
        shp->setUpdateCallback(new BallUpdateCallback(shp_body));
        
        // register an extra OSC callback so that we can spy on OSC messages:
        std::vector<lo_server>::iterator servIter;
        for (servIter = spin.getContext()->lo_rxServs_.begin(); servIter != spin.getContext()->lo_rxServs_.end(); ++servIter)
        {
            lo_server_add_method((*servIter), std::string("/SPIN/"+spin.getSceneID()+"/shp").c_str(), NULL, shpCallback, &shp_body);
        }

    }


    // ball
    osg::ref_ptr<osg::Node> ball;
    osg::ref_ptr<osg::PositionAttitudeTransform> ball_pat;
    if (0) {
        btRigidBody* lnd;
        btCollisionShape *lnd_shape;
        ball = osgDB::readNodeFile("ball.osg");
        ball->setName("ball");
        ball_pat = new osg::PositionAttitudeTransform;
        ball_pat->setScale(osg::Vec3(5, 1, 1));
        ball_pat->setAttitude(osg::Quat(osg::DegreesToRadians(45.0),osg::Y_AXIS));
        ball_pat->addChild(ball.get());
        spin.sceneManager->rootNode->addChild(ball_pat.get());
    
        lnd_shape = new btBoxShape(btVector3(ball_pat->getScale().x(), ball_pat->getScale().y(), ball_pat->getScale().z()));
        //lnd_shape = new btBoxShape(btVector3(1, 1, 1));

        btVector3 pos(0, 0, 20.0);
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(pos);
        btQuaternion quat(ball_pat->getAttitude().x(), ball_pat->getAttitude().y(), ball_pat->getAttitude().z(), ball_pat->getAttitude().w());
        trans.setRotation(quat);
        btScalar mass = 1.f;

        lnd = createRigidBody(m_dynamicsWorld, mass, trans, lnd_shape);
        lnd->setUserPointer(ball.get());
        lnd->setCollisionFlags(lnd->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        lnd->setCcdMotionThreshold(0.5);
        lnd->setCcdSweptSphereRadius(0.2 * 0.5);
        ball_pat->setUpdateCallback(new BallUpdateCallback(lnd));
    }


    // ground plane
    osg::ref_ptr<osg::Node> groundPlane;
    osg::ref_ptr<osg::PositionAttitudeTransform> ground_pat;
    {
        groundPlane = osgDB::readNodeFile("ground-plane.osg");
        //groundPlane->setName("groundPlane");
        ground_pat = new osg::PositionAttitudeTransform;
        ground_pat->addChild(groundPlane.get());
        ground_pat->setScale(osg::Vec3(100.0, 100.0, 0.1));
        spin.sceneManager->rootNode->addChild(ground_pat.get());

        btCollisionShape *gnd_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0.5);

        btVector3 pos(0, 0, -0.5);
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(pos);
        btScalar mass = 0.f; // 0 makes this a static (unmovable) object
        
        btRigidBody* gnd = createRigidBody(m_dynamicsWorld, mass, trans, gnd_shape);

        gnd->setUserPointer(groundPlane.get());
        gnd->setCollisionFlags(gnd->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT
                        | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        gnd->setActivationState(DISABLE_DEACTIVATION);
    }
    
    
    
    // move view back:
    spin.userNode->setTranslation(0,-50,3);
    spin.sceneManager->createNode("grid", "GridNode");
    
	
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

	// program loop:
	while(not viewer.done())
	{
		
		if (spinListener.isRunning())
		{
			double dt = osg::Timer::instance()->delta_s(lastFrameTick, osg::Timer::instance()->tick());

			if (dt >= minFrameTime)
			{
				viewer.advance();
				viewer.eventTraversal();
				pthread_mutex_lock(&sceneMutex);
				spin.sceneManager->update();

                //physics update:
                m_dynamicsWorld->stepSimulation(dt); //, 10, 0.01);
                m_dynamicsWorld->updateAabbs();

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
