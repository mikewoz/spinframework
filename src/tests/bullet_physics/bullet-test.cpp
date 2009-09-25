#include <iostream>

#include <osgDB/ReadFile>
#include <osg/CoordinateSystemNode>
#include <osg/PositionAttitudeTransform>
#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <btBulletDynamicsCommon.h>

float phys2world_scale = 100.0;
extern ContactAddedCallback gContactAddedCallback;

class BallUpdateCallback: public osg::NodeCallback
{
    private:
        btRigidBody *_body;

    public:
        BallUpdateCallback(btRigidBody *body) :
            _body(body)
        {
        }

        virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
        {
            btScalar m[16];

            btDefaultMotionState* myMotionState = (btDefaultMotionState*) _body->getMotionState();
            myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);

            osg::Matrixf mat(m);

            osg::PositionAttitudeTransform *pat = dynamic_cast<osg::PositionAttitudeTransform *> (node);
            pat->setPosition(mat.getTrans());
            pat->setAttitude(mat.getRotate());

            //std::cout << mat.getTrans().x() << ", " << mat.getTrans().y() << ", " << mat.getTrans().z() << std::endl;

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
        std::cout << "Hit between " << static_cast<osg::Node *> (colObj0->getUserPointer())->getName() << " and "
                        << static_cast<osg::Node *> (colObj1->getUserPointer())->getName() << " at ["
                        << impact_point.getX() << ", " << impact_point.getY() << ", " << impact_point.getZ() << "] "
                        << std::endl;
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

int main(int argc, char **argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc, argv);

    // construct the viewer.
    osgViewer::Viewer viewer(arguments);

    // scenegraph root
    osg::ref_ptr<osg::Group> root = new osg::Group;

    // read the scene from the list of file specified command line args.
    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFile("data/ball.osg");
    osg::ref_ptr<osg::PositionAttitudeTransform> ball_pat = new osg::PositionAttitudeTransform;
    //ball_pat->setScale(osg::Vec3(100, 100, 100));
    ball_pat->addChild(loadedModel.get());
    root->addChild(ball_pat.get());

    osg::ref_ptr<osg::Node> loadedModel2 = osgDB::readNodeFile("data/ground-plane.osg");
    osg::ref_ptr<osg::PositionAttitudeTransform> ground_pat = new osg::PositionAttitudeTransform;
    ground_pat->addChild(loadedModel2.get());
    ground_pat->setScale(osg::Vec3(100.0, 100.0, 0.1));
    root->addChild(ground_pat.get());

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // optimize the scene graph, remove redundant nodes and state etc.
    osgUtil::Optimizer optimizer;
    optimizer.optimize(root.get());

    // pass the loaded scene graph to the viewer.
    viewer.setSceneData(root.get());

    /*
     * Physics
     */
    btVector3 worldAabbMin(-1000, -1000, -1000);
    btVector3 worldAabbMax(1000, 1000, 1000);
    const int maxProxies = 32766;

    // register global callback
    gContactAddedCallback = materialCombinerCallback;

    btSequentialImpulseConstraintSolver *solver = new btSequentialImpulseConstraintSolver;
    btAxisSweep3 *broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);

    btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btDynamicsWorld *m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,
                                                                   broadphase,
                                                                   solver,
                                                                   collisionConfiguration);
    m_dynamicsWorld->setGravity(btVector3(0, 0, -10.0));

    // ground plane
    {
        btCollisionShape *gnd_shape = new btStaticPlaneShape(btVector3(0, 0, 1), 0.5);

        btVector4 pos;
        pos.setValue(0, 0, -0.5, 0);
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(pos);
        btScalar mass = 0.f;

        btRigidBody* gnd = createRigidBody(m_dynamicsWorld, mass, trans, gnd_shape);

        gnd->setUserPointer(loadedModel2.get());
        gnd->setCollisionFlags(gnd->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT
                        | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        gnd->setActivationState(DISABLE_DEACTIVATION);
    }

    // lander
    btRigidBody* lnd;
    btCollisionShape *lnd_shape;
    {
        lnd_shape = new btBoxShape(btVector3(1, 1, 1));

        btVector4 pos;
        pos.setValue(0, 0, 20.0, 0);
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(pos);
        btScalar mass = 1.f;

        lnd = createRigidBody(m_dynamicsWorld, mass, trans, lnd_shape);
        lnd->setUserPointer(loadedModel.get());
        lnd->setCollisionFlags(lnd->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        //lnd->setCcdSquareMotionThreshold(0.5);
        lnd->setCcdSweptSphereRadius(0.2 * 0.5);
    }
    ball_pat->setUpdateCallback(new BallUpdateCallback(lnd));

    // set up the camera manipulators.
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator('1', "Trackball", new osgGA::TrackballManipulator());
        keyswitchManipulator->addMatrixManipulator('2', "Flight", new osgGA::FlightManipulator());
        keyswitchManipulator->addMatrixManipulator('3', "Drive", new osgGA::DriveManipulator());
        keyswitchManipulator->addMatrixManipulator('4', "Terrain", new osgGA::TerrainManipulator());

        std::string pathfile;
        char keyForAnimationPath = '5';
        while (arguments.read("-p", pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid())
            {
                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator(keyForAnimationPath, "Path", apm);
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator(keyswitchManipulator.get());
    }

    // add the state manipulator
    viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));

    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);

    // add the screen capture handler
    viewer.addEventHandler(new osgViewer::ScreenCaptureHandler);

    // create the windows and run the threads.
    viewer.realize();

    osg::Timer_t frame_tick = osg::Timer::instance()->tick();
    while (!viewer.done())
    {
        // Physics update
        osg::Timer_t now_tick = osg::Timer::instance()->tick();
        float dt = osg::Timer::instance()->delta_s(frame_tick, now_tick);
        frame_tick = now_tick;
        /* int numSimSteps = */
        m_dynamicsWorld->stepSimulation(dt); //, 10, 0.01);
        m_dynamicsWorld->updateAabbs();

        // render
        viewer.frame();
    }

    return 0;
}

