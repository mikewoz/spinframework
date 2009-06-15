#ifndef ASCAMERAMANAGER_H_
#define ASCAMERAMANAGER_H_

#include<osgViewer/CompositeViewer>
#include<osg/PositionAttitudeTransform>
#include <osg/GraphicsContext>

#include "lo/lo.h"
#include "tinyxml.h"
#include "asCameraView.h"
#include "asWindow.h"

/**
 * \brief The master class that manages: asCameraView and asWindow for rendering
 * 
 * This class should be instantiate in any process that wishes to render the
 * scene. It uses osgViewer::CompositeViewer to allow for many simultaneous but
 * unique views of the scene.
 */
class asCameraManager
{
	
	public:
		
		asCameraManager(std::string viewerID, osgViewer::CompositeViewer &viewer, std::string listenAddr, std::string listenPort);
		~asCameraManager();
		
		void debugPrint();
		
		void refreshWindows();
		
		asWindow *getWindow(int id);		
		asCameraView *getCamera(const char *id);
		
		void createCamera(const char *id);
		void createCamera(const char *id, int windowID);
		void destroyCamera(const char *id);

		void sendCameraList(lo_address addr, lo_server serv);
		void setResolution(std::string resolutionString);

		void setSceneData(osg::Node *node);
		
		void init();
		void update();

		
		//void loadXML(std::string configFile);


		// For each camera defined, ..
		std::vector< osg::ref_ptr<asCameraView> > cameraList;

		std::vector< osg::ref_ptr<asWindow> > windowList;

		
	private:
		
		std::string id;
		
		bool initialized;
		
		// We maintain a reference to the CompositeViewer for conveience:
		osgViewer::CompositeViewer *viewer;

		// All of the above asCameraViews are attached to the cameraGroup node:
		osg::ref_ptr<osg::Group> cameraGroup;
		
		osg::ref_ptr<osg::Node> sceneRoot;
		
		// An OSC listener (liblo) listens for messages that move the above 
		// camera transforms:
		lo_server_thread oscListener;
		

};

// OSC stuff:
static void oscListener_error(int num, const char *msg, const char *path);
static int oscListener_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int oscListener_admin_callback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);



#endif
