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

#ifndef __ASSOUNDNODE_H
#define __ASSOUNDNODE_H

#include "asGlobals.h"
#include "asDSPnode.h"

#define AS_UNIT_SCALE 1.0f // 1m
#define AS_DEBUG_SCALE 4.0f // size of debug views (radiation/sensitivity/etc)
#define AS_LASER_RADIUS 0.0025
#define DEFAULT_DIRECTIVITY_COLOR osg::Vec3(0.0,0.0,1.0) //blue

class asDSPnode;

/**
 * \brief Represents a point sound source (or sink).
 * 
 * The asSoundNode class allows for the positioning of a sound node in 3D space,
 * and controlling various aspects such as directivity and visual rendering for
 * debugging purposes.
 */
class asSoundNode : public asDSPnode
{
	
	public:
		
		asSoundNode(asSceneManager *sceneManager, char *initID);
		virtual ~asSoundNode();
			
		// SET methods:
		void setRolloff (char *newvalue);
		void setSpread (float newvalue);
		void setLength (float newvalue);
		
		void setVUmeterFlag (float newFlag);
		void setDirectivityFlag (float newFlag);
		void setLaserFlag (float newFlag);
		
		// 
		void setIntensity(float newvalue);
		
		// GET methods:
		const char* getRolloff() { return _rolloff.c_str(); }
		float getSpread() { return _spread; }
		float getLength() { return _length; }
		
		float getVUmeterFlag() { return VUmeterFlag; }
		float getDirectivityFlag() { return directivityFlag; }
		float getLaserFlag() { return laserFlag; }
		
		
		// DRAW methods:
		void drawVUmeter();
		void drawDirectivity();
		void drawLaser();
		
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
		
		/**
		 * Same goes shared functions from asDSPnode. We must redefine them here
		 * so that osg::Introspection will see them:
		 */
		virtual void connect(char *snk) { asDSPnode::connect(snk); };
		virtual void disconnect(char *snk) { asDSPnode::disconnect(snk); };
		virtual void setActive(int i) { asDSPnode::setActive(i); };
		virtual void setDSP(char *newDSP) {asDSPnode::setDSP(newDSP); };
		//virtual void connectionMsg (char *snkName, char *method, float value) { asDSPnode::connectionMsg(snkName,method,value); };
		
	private:
		
		float currentSoundIntensity;
		osg::Vec3 currentSoundColor;
		
		
		std::string _rolloff; // we keep a reference name for the rolloff (directivity) table
		float _spread; // propagation cone for source
		float _length; // the length of the laser and cone

		// TODO: add toggle for PRE/POST

		// flags with continuous values (can be used for alpha, etc):
		float VUmeterFlag;
		float directivityFlag;
		float laserFlag;


		// The following methods and parameters are for drawing aspects of the
		// soundNode using OSG (eg, directivity pattern, laser, etc)

		// directivity patterns:
		osg::ref_ptr<osg::Geode> directivityGeode;

		// laser beams:
		osg::ref_ptr<osg::Geode> laserGeode;

		// VU meter:
		osg::ref_ptr<osg::PositionAttitudeTransform> VUmeterTransform;


};


#endif
