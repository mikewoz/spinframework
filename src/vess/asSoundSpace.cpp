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
//    La SociŽtŽ des Arts Technologiques (http://www.sat.qc.ca)
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


#include "asGlobals.h"
#include "asSoundSpace.h"
#include "asSceneManager.h"

using namespace std;

//extern asSceneManager *sceneManager;


// ===================================================================
// constructor:
asSoundSpace::asSoundSpace (asSceneManager *sceneManager, char *initID) : asDSPnode(sceneManager, initID)
{
	nodeType = "asSoundSpace";
	
	absorption = 0.6;
	filterCoef = 0.0;
	transition = 0.0;
	
}

// ===================================================================
// destructor
asSoundSpace::~asSoundSpace()
{

}



// ===================================================================
// ======================= DRAW METHODS: =============================
// ===================================================================



void asSoundSpace::setAbsorption (t_floatarg newval)
{
	absorption = newval;
	BROADCAST(this, "sf", "setAbsorption", absorption);
}

void asSoundSpace::setFilterCoef (t_floatarg newval)
{
	filterCoef = newval;
	BROADCAST(this, "sf", "setFilterCoef", filterCoef);
}

void asSoundSpace::setTransition (t_floatarg newval)
{
	transition = newval;
	BROADCAST(this, "sf", "setTransition", transition);
}

std::vector<lo_message> asSoundSpace::getState ()
{
	std::vector<lo_message> ret;
	return ret;
}


