#ifndef ASCAMERAVIEW_H_
#define ASCAMERAVIEW_H_

#include <osg/Referenced>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/GraphicsContext>
#include <osgViewer/View>
#include <osg/Geometry>
#include <osg/Geode>

#include "asWindow.h"

//enum distortionEnum { NONE, DOME };

/**
 * \brief Provides a mechanism to create several independent camera views of the
 *        scene.
 * 
 * Allows for several unique views, with control of which window the camera is
 * rendered upon, viewport, and even distortion (eg, for spherical projection).
 */
class asCameraView : public osg::PositionAttitudeTransform
{
	
public:
	asCameraView(const char *id, osgViewer::CompositeViewer &vr, osg::ref_ptr<asWindow> win);
	~asCameraView();
	
	const char* getID() { return id.c_str(); }
	osgViewer::View* getView() { return view.get(); }
	asWindow *getWindow() { return win.get(); }
	std::string getDistortionType() { return distortionType; }
	
	void setFrustum (float left, float right, float bottom, float top, float zNear, float zFar);
	void setViewport (float x, float y, float w, float h);
	void updateViewport();
	void setClearColor (float r, float g, float b, float a);
	void enableDome (bool b);
	void setTranslation (float x, float y, float z);
	void setOrientation (float x, float y, float z);
	void move (float x, float y, float z);
	void rotate (float p, float r, float y);
	
	void setDistortion(std::string type, std::vector<float> args);
	
	void setResolution(std::string resolutionString);
		
	void makePlanar();
	void makeDome();
	
	
	// We use flags to specify whether the view should be added to or removed
	// from the viewer. We cannot do it within this class, becase we never know
	// the status of the viewer threads; we instead rely on asCameraManager's
	// update() method to be called at appropriate times for viewer manipulation
	bool additionFlag;
	bool removalFlag;
	bool distortionUpdate;
	
	
	
private:
	osg::Vec3 _orientation; // in degrees, and in the order it comes in
	osg::ref_ptr<osgViewer::View> view;
	std::string id;
	
	osg::Vec4 _viewport; // need to keep viewport for refreshes
	
	osg::ref_ptr<asWindow> win;
	
	std::string					distortionType;
	std::vector<float>			distortionArgs;
	osg::ref_ptr<osg::Geometry>	distortionGeometry;
	osg::ref_ptr<osg::Geode>	distortionGeode;

};


static osg::Geometry* create3DSphericalDisplayDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector, double sphere_radius, double collar_radius,osg::Image* intensityMap, const osg::Matrix& projectorMatrix);
static osg::Geometry* createDomeDistortionMesh(const osg::Vec3& origin, const osg::Vec3& widthVector, const osg::Vec3& heightVector, double sphere_radius, double collar_radius);


#endif
