#ifndef EVENTDISPATCHER_H_
#define EVENTDISPATCHER_H_

#include "asGlobals.h"
#include "asReferenced.h"
#include "asBasicNode.h"
#include "asSoundNode.h"
#include "asSoundSpace.h"
#include "asShape.h"
#include "netmail.h"


#define CALL_FUNCTION(objPtr,ptrToMember) (*(objPtr).*(ptrToMember))


// The standard callback type.  The callback data would be a class or a struct that has information that the function called would be able to handle.  If you don't need to pass data, just remove it.
typedef void ( * FunctionCallbackType )( void *callbackData, void *userData );

typedef void (asReferenced::*FnPointer_asBasicNode)	(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[]);
typedef void (asReferenced::*FnPointer_asReferenced)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[]);




class EventDispatcher
{

public:
	EventDispatcher();
	~EventDispatcher();
	void dispatch(asReferenced *n, const char* types, ...);
    void event(osg::ref_ptr<asReferenced> n, mail *msg);


	lo_address broadcastAddr;
	
private:

	  
	std::map<std::string, void(asReferenced::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_asReferenced;
	std::map<std::string, void(asBasicNode::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_asBasicNode;
	std::map<std::string, void(asSoundNode::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_asSoundNode;
	std::map<std::string, void(asSoundSpace::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_asSoundSpace;
	std::map<std::string, void(asShape::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_asShape;

};



#endif
