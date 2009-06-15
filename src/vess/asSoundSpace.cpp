// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================


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


