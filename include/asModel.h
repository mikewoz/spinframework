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


#ifndef __ASMODEL_H
#define __ASMODEL_H

#include <string>
#include <osg/Vec3>
#include <osg/PositionAttitudeTransform>

#include <osgUtil/Optimizer>

#include "asGlobals.h"
#include "asReferenced.h"


#define ASMODEL_NUM_ANIM_CONTROLS 10 // identify how many animation controls there are

enum animationModeType { OFF, SWITCH, SEQUENCE };


/**
 * \brief Node for including 3D models of popular formats in the scene.
 * 
 * This class allows us to attach an external 3D model from a file. Popular
 * formats (.3ds, .obj, .osg, etc) are supported as long as an OSG plugin exists
 * to read it. The model can be offset and scaled. Animations are also supported
 * as long as the model has an osg::Switch or osg::Sequence node inside.
 */

class asModel : public asReferenced
{

public:

	asModel (asSceneManager *sceneManager, char *initID);
	virtual ~asModel();
	
	virtual void updateNodePath();

	void setTranslation		(float x, float y, float z);
	void setOrientation		(float p, float r, float y);
	void setScale			(float x, float y, float z);
	
	void setModel			(int newModel);
	void setModelFromFile	(char *f);
	
	osg::Vec3 getTranslation() { return modelTransform->getPosition(); };
	osg::Vec3 getOrientation() { return _orientation; };
	osg::Vec3 getScale() { return modelTransform->getScale(); };
	
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
	
	void drawModel();	
	
	// asModel supports simple loading of 3D models (eg, those that
	// have been designed in 3DSMax, Maya, Blender, etc.)
	// 
	// The model is attached to a modelTransform, allowing it to be
	// offset from it's parent.

	osg::ref_ptr<osg::PositionAttitudeTransform> modelTransform;

	osg::Vec3 _orientation; // store the orientation as it comes in (in degrees)
	
	// the model id from the database:
	int modelID;
	std::string modelName;
	std::string modelPath;
	
	osg::ref_ptr<osg::Group> model;
	
	// animation stuff for gfx:
	t_float state[ASMODEL_NUM_ANIM_CONTROLS]; // keyframe index (value from 0-1)
	int animationNumFrames[ASMODEL_NUM_ANIM_CONTROLS]; // number of keyframes
	animationModeType animationMode[ASMODEL_NUM_ANIM_CONTROLS]; // type of animation (switch vs. sequence vs. ??)
	osg::ref_ptr<osg::Switch> switcher[ASMODEL_NUM_ANIM_CONTROLS];
	osg::ref_ptr<osg::Sequence> sequencer[ASMODEL_NUM_ANIM_CONTROLS];
	
	osgUtil::Optimizer optimizer;
	
	
	
};



#endif
