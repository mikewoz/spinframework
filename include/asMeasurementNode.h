#ifndef ASMEASUREMENTNODE_H_
#define ASMEASUREMENTNODE_H_


#include "asGlobals.h"
#include "asReferenced.h"

/**
 * \brief A node that can be used to measure relations to another node.
 * 
 * 
 * A targetNode must be specified, and then measurements (such as distance,
 * incidence of orientation, etc.) are computed in the callback and can be
 * reported with different levels of detail.
 */
class asMeasurementNode : public asReferenced
{

public:

	asMeasurementNode(asSceneManager *sceneManager, char *initID);
	virtual ~asMeasurementNode();
	
	virtual void callbackUpdate();

	void setTarget (char *id);
	void setReportingLevel (int level);
	
	const char* getTarget();
	int getReportingLevel() { return this->reportingLevel; }
	
	
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
	
	osg::ref_ptr<asReferenced> targetNode;
	int reportingLevel;
	
	
	osg::Matrix thisMatrix, targetMatrix;
	
};



#endif
