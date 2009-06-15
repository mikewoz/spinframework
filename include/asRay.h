#ifndef ASRAY_H_
#define ASRAY_H_

#include <osg/Geode>


#include "asGlobals.h"
#include "asReferenced.h"

/**
 * \brief Describes a ray (actually a line segment) in space.
 * 
 * When attached to an asBasicNode, the asRay essentially points in the
 * direction of specified by it's parent, and is visible up to a certain length.
 * 
 * asRay is used by asPointer to report which nodes are being pointed at.
 */
class asRay : public asReferenced
{

public:

	asRay(asSceneManager *sceneManager, char *initID);
	virtual ~asRay();

	void setVisible		(int b);
	void setLength		(float len);
	void setThickness	(float thk);
	void setColor		(float r, float g, float b, float a);
	
	int getVisible() { return (int) this->visible; }
	float getLength() { return this->length; };
	float getThickness() { return this->thickness; };
	osg::Vec4 getColor() { return this->color;  };
	
	
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
	
	
private:
	
	void drawRay();	
	
	osg::ref_ptr<osg::Geode> rayGeode;	
	
	bool visible;
	float length;
	float thickness;
	osg::Vec4 color;
	
};



#endif
