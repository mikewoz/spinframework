#include "asToggle.h"
#include "asSceneManager.h"

using namespace std;

//extern asSceneManager *sceneManager;


// *****************************************************************************
// constructor:
asToggle::asToggle (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asToggle");
	nodeType = "asToggle";

	this->setNodeMask(STATSDATA_NODE_MASK); // nodemask info in asGlobals.h
	
	toggle = false;
}

// *****************************************************************************
// destructor
asToggle::~asToggle()
{

}


// *****************************************************************************
void asToggle::setToggle (int b)
{
	if (this->toggle != (bool)b)
	{
		this->toggle = (bool) b;
		BROADCAST(this, "si", "setToggle", (int) this->toggle);	
	}
}

// *****************************************************************************

std::vector<lo_message> asToggle::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setToggle", (int)toggle);
	ret.push_back(msg);
	
	return ret;
}