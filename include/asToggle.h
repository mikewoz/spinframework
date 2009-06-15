#ifndef ASTOGGLE_H_
#define ASTOGGLE_H_

#include <osg/Geode>


#include "asGlobals.h"
#include "asReferenced.h"

/**
 * \brief DEPRECATED
 */
class asToggle : public asReferenced
{

public:

	asToggle(asSceneManager *sceneManager, char *initID);
	virtual ~asToggle();

	void setToggle (int b);
	int getToggle() { return (int) this->toggle; }

	
	
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
	
	bool toggle;
	
};



#endif
