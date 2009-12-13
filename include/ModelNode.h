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

#ifndef __ModelNode_H
#define __ModelNode_H

#include <string>
#include <osgUtil/Optimizer>
#include <osg/ImageStream>
#include <osg/Image>


#include "GroupNode.h"

#ifdef WITH_SHARED_VIDEO
#include "SharedVideoTexture.h"
#endif

#define MODELNODE_NUM_ANIM_CONTROLS 10 // identify how many animation controls there are

enum animationModeType { OFF, SWITCH, SEQUENCE };


/**
 * \brief Node for including 3D models of popular formats in the scene.
 *
 * This class allows us to attach an external 3D model from a file. Popular
 * formats (.3ds, .obj, .osg, etc) are supported as long as an OSG plugin exists
 * to read it. The model can be offset and scaled. Animations are also supported
 * as long as the model has an osg::Switch or osg::Sequence node inside.
 */

class ModelNode : public GroupNode
{

public:

	ModelNode (SceneManager *sceneManager, char *initID);
	virtual ~ModelNode();

	virtual void setHost (const char *newvalue);	
	
	void setModelFromFile	(const char *filename);
	const char* getModelFromFile() { return modelPath.c_str(); }

	void setPlay(int i);
	int getPlay() { return (int) _play; }

	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();

private:

	void drawModel();

	// ModelNode supports simple loading of 3D models (eg, those that
	// have been designed in 3DSMax, Maya, Blender, etc.)

	// the model:
	//std::string modelName;
	std::string modelPath;
	
#ifdef WITH_SHARED_VIDEO
	//std::vector< osg::ref_ptr<SharedVideoTexture> > sharedVideoTextures;
	//std::vector<SharedVideoTexture*> sharedVideoTextures;
	//osg::ref_ptr<SharedVideoTexture> shTex;
#endif

	osg::ref_ptr<osg::Group> model;

	osg::ref_ptr<osg::Image> image;
	osg::ref_ptr<osg::ImageStream> imagestream;
	bool _play;

	// animation stuff for gfx:
	t_float state[MODELNODE_NUM_ANIM_CONTROLS]; // keyframe index (value from 0-1)
	int animationNumFrames[MODELNODE_NUM_ANIM_CONTROLS]; // number of keyframes
	animationModeType animationMode[MODELNODE_NUM_ANIM_CONTROLS]; // type of animation (switch vs. sequence vs. ??)
	osg::ref_ptr<osg::Switch> switcher[MODELNODE_NUM_ANIM_CONTROLS];
	osg::ref_ptr<osg::Sequence> sequencer[MODELNODE_NUM_ANIM_CONTROLS];

	osgUtil::Optimizer optimizer;



};



#endif
