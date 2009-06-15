#ifndef ASCONTOUR_H_
#define ASCONTOUR_H_

#include <osg/Geode>
#include <osg/PositionAttitudeTransform>

#include "asReferenced.h"

enum asContourTypeEnum { THIN, CUBIC, CYLINDRICAL };
enum trackingModeEnum { POSITION, FULL6DOF };

/**
 * \brief Represents a sequence of connected points in 3D space
 * 
 * The contour holds a number of vertices up the limit specified by _maxVertices.
 * New values are added to the front of the vectorArray, and once it is filled,
 * we pop the last element off the end.
 * 
 * An index controls the point on which a child node is attached to the contour.
 */
class asContour : public asReferenced
{

public:

	asContour(asSceneManager *sceneManager, char *initID);
	virtual ~asContour();
		
	/**
	 * IMPORTANT:
	 * subclasses of asReferenced are allowed to contain complicated subgraphs,
	 * and can also change their attachmentNode so that children are attached
	 * anywhere in this subgraph. If that is the case, the updateNodePath()
	 * function MUST be overridden, and extra nodes must be manually pushed onto
	 * the currentNodePath.
	 */
	virtual void updateNodePath();

	virtual void callbackUpdate();
	
	void updateTransforms();
	osg::Quat getOrientation(int index);
	osg::Vec3 getTranslation(float index);
	
	void setCurrentIndex (float newValue);
	void prev();
	void next();

	void reset();
	
	void add (float x, float y, float z);
	void setMaxVertices (int newValue);
	
	void setTrackingMode(int newValue);
	
	void setVisible (int newValue);
	void setThickness (float newValue);
	void setLineType (int newValue);
	void setColor (float newR, float newG, float newB, float newA);
		
	
	float getCurrentIndex() { return _currentIndex; }
	int getMaxVertices() { return _maxVertices; }
	
	int getTrackingMode() { return (int) _trackingMode; }
	
	int getVisible() { return (int) _visible; }
	float getThickness() { return _thickness; }
	float getLineType() { return (int) _lineType; }
	osg::Vec4 getColor() { return _color;  }
	
	
	
	void draw();
	

	/**
	 * For each subclass of asReferenced, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();
	
	/**
	 * We must include a stateDump() method that simply invokes the base class
	 * method. Simple C++ inheritance is not enough, because osg::Introspection
	 * won't see it.
	 */
	virtual void stateDump() { asReferenced::stateDump(); };
		
	// ============

private:
	
	bool _visible;
	
	bool _redrawFlag;
	
	osg::ref_ptr<osg::Vec3Array> _vArray;
	
	float _currentIndex; // allow float indices for interpolation between values
	int _maxVertices;

	trackingModeEnum _trackingMode;

	
	osg::Vec4 _color;
	
	asContourTypeEnum _lineType;
	float _thickness;

	osg::ref_ptr<osg::PositionAttitudeTransform> mainTransform;
	osg::ref_ptr<osg::Geode> vArrayGeode;
	//osg::ref_ptr<osg::Geometry> vArrayGeometry;

};


#endif
