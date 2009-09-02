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
//  You should have received a copy of the GNU Lesser General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#ifndef EVENTDISPATCHER_H_
#define EVENTDISPATCHER_H_

#include "ReferencedNode.h"
#include "GroupNode.h"
#include "SoundNode.h"
#include "SoundSpace.h"
#include "ShapeNode.h"
#include "netmail.h"


#define CALL_FUNCTION(objPtr,ptrToMember) (*(objPtr).*(ptrToMember))


// The standard callback type.  The callback data would be a class or a struct that has information that the function called would be able to handle.  If you don't need to pass data, just remove it.
typedef void ( * FunctionCallbackType )( void *callbackData, void *userData );

typedef void (ReferencedNode::*FnPointer_GroupNode)	(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[]);
typedef void (ReferencedNode::*FnPointer_ReferencedNode)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[]);




class EventDispatcher
{

public:
	EventDispatcher();
	~EventDispatcher();
	void dispatch(ReferencedNode *n, const char* types, ...);
    void event(osg::ref_ptr<ReferencedNode> n, mail *msg);


	lo_address broadcastAddr;
	
private:

	  
	std::map<std::string, void(ReferencedNode::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_ReferencedNode;
	std::map<std::string, void(GroupNode::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_GroupNode;
	std::map<std::string, void(SoundNode::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_SoundNode;
	std::map<std::string, void(SoundSpace::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_SoundSpace;
	std::map<std::string, void(ShapeNode::*)(short narg, char argv[][AS_NAMESIZE], short nfl, float fl[])> functionTable_ShapeNode;

};



#endif
