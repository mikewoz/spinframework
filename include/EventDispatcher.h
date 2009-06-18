// -----------------------------------------------------------------------------
// |    ___  ___  _  _ _     ___                                        _      |
// |   / __>| . \| || \ |   | __>_ _  ___ ._ _ _  ___  _ _ _  ___  _ _ | |__   |
// |   \__ \|  _/| ||   |   | _>| '_><_> || ' ' |/ ._>| | | |/ . \| '_>| / /   |
// |   <___/|_|  |_||_\_|   |_| |_|  <___||_|_|_|\___.|__/_/ \___/|_|  |_\_\   |
// |                                                                           |
// |---------------------------------------------------------------------------|
//
// http://spinframework.sourceforge.net
// Copyright (C) 2009 Mike Wozniewski, Zack Settel
//
// Developed/Maintained by:
//    Mike Wozniewski (http://www.mikewoz.com)
//    Zack Settel (http://www.sheefa.net/zack)
// 
// Principle Partners:
//    Shared Reality Lab, McGill University (http://www.cim.mcgill.ca/sre)
//    La Societe des Arts Technologiques (http://www.sat.qc.ca)
//
// Funding by:
//    NSERC/Canada Council for the Arts - New Media Initiative
//    Heritage Canada
//    Ministere du Developpement economique, de l'Innovation et de l'Exportation
//
// -----------------------------------------------------------------------------
//  This file is part of the SPIN Framework.
//
//  SPIN Framework is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  SPIN Framework is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

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
