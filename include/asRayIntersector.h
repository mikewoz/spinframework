#ifndef ASRAYINTERSECTOR_H_
#define ASRAYINTERSECTOR_H_

#include "asGlobals.h"
#include "asReferenced.h"

#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

/**
 * \brief DEPRECATED (replaced by asPointer)
 */
class asRayIntersector : public asReferenced
{

public:
	
	asRayIntersector(asSceneManager *sceneManager, char *initID);
	virtual ~asRayIntersector();
	
	virtual void callbackUpdate();

	
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
	
	std::vector<t_symbol*> intersectList;
	//osgUtil::IntersectionVisitor rayIntersectVisitor;
	
};

	
	
	
	
#endif /*ASRAYINTERSECTOR_H_*/
