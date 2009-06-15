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


#ifndef __ASSOUNDSPACE_H
#define __ASSOUNDSPACE_H

#include "asGlobals.h"
#include "asDSPnode.h"

#include <osgUtil/IntersectVisitor>

class asDSPnode;

/**
 * \brief Represents an acoustic enclosure or volume in space.
 * 
 * The asSoundSpace class allows for the creation of an acoustic space inhabited
 * by audio processing (or playback).
 */
class asSoundSpace : public asDSPnode
{
	
  public:
		
		asSoundSpace(asSceneManager *sceneManager, char *initID);
		virtual ~asSoundSpace();


		void setAbsorption (t_floatarg newval);
		void setFilterCoef (t_floatarg newval);
		void setTransition (t_floatarg newval);
		t_float absorption;
		t_float filterCoef;
		t_float transition;
		
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

#ifdef AS_GRAPHICAL
		osgUtil::IntersectVisitor soundSpaceIntersectVisitor;
#endif
	
};

#endif
