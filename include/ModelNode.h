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
#include "SceneManager.h"
#include "ReferencedStateSet.h"

#define MODELNODE_NUM_ANIM_CONTROLS 10 // identify how many animation controls there are

/**
 * \brief Node for including 3D models of popular formats in the scene.
 *
 * This class allows us to attach an external 3D model from a file. Popular
 * formats (.3ds, .obj, .osg, etc) are supported as long as an OSG plugin exists
 * to read it. The model can be offset and scaled. Animations are also supported
 * as long as the model has an osg::Switch or osg::Sequence node inside. Texture
 * and shader information can be parsed out (using the StateRegistraion flag),
 * and automatically create referenced statesets controllable by SPIN.
 */

class ModelNode : public GroupNode
{

public:

	ModelNode (SceneManager *sceneManager, char *initID);
	virtual ~ModelNode();

	enum animationModeType { OFF, SWITCH, SEQUENCE };


	/**
	 * The context is an arbitrary keyword that associates this node with a
	 * particular behaviour. Currently, it is used to *prevent* display if the
	 * context matches the name of a machine. ie, allowing it to be seen on all
	 * machines except for the one that is named by setContext.
	 */
	virtual void setContext 	(const char *newvalue);

	/**
	 * Load a 3D model from a file (eg, .osg, .3ds, .obj, .dae, etc).
	 * Make sure that StateRegistration flag is set if you want to have control
	 * over any textures or shaders withing the model
	 */
	void setModelFromFile		(const char *filename);
	const char* getModelFromFile() { return modelPath.c_str(); }
	
	/**
	 * The StateRegistration flag should be set if you want any textures or
	 * shaders to be parsed out when loading a model. Any statesets found in
	 * the file will generate corresponding ReferencedStateSets for use within
	 * SPIN. This way, you'll be able to swap textures, control videos, adjust
	 * shader parameters, etc.
	 */
	void setStateRegistration	(int i);
	int getStateRegistration() { return (int)_registerStates; }

	/**
	 * Render bins allow you to control drawing order, and manage Z-fighting.
	 * The higher the number, the later it gets processed (ie, appears on top).
	 * Default renderBin = 11
	 */
	void setRenderBin			(int i);
	int getRenderBin() { return _renderBin; }

	/**
	 * Render bins allow you to control drawing order, and manage Z-fighting.
	 * The higher the number, the later it gets processed (ie, appears on top).
	 * Default renderBin = 11
	 */
	void setKeyframe (int index, float keyframe);
	float getKeyframe(int index) { return _keyframe[index]; }

	/**
	 * For statesets embedded in the model, it is possible to swap with some
	 * other (already existing) stateset.
	 *
	 * Note: for this to work, stateRegistration must be enabled.
	 */
	void setStateSet (int index, const char *replacement);


	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();

private:

	void drawModel();

	// the model:
	//std::string modelName;
	std::string modelPath;
	
	std::vector<t_symbol*> _statesetList;

	std::vector<osg::Drawable*> _ssDrawableList;
	std::vector<osg::Node*> _ssNodeList;

	osg::ref_ptr<osg::Group> model;

	// animation stuff for gfx:
	float _keyframe[MODELNODE_NUM_ANIM_CONTROLS]; // keyframe index (value from 0-1)
	animationModeType animationMode[MODELNODE_NUM_ANIM_CONTROLS]; // type of animation (switch vs. sequence vs. ??)
	osg::ref_ptr<osg::Switch> switcher[MODELNODE_NUM_ANIM_CONTROLS];
	osg::ref_ptr<osg::Sequence> sequencer[MODELNODE_NUM_ANIM_CONTROLS];

	osgUtil::Optimizer optimizer;

	bool _registerStates;
	int _renderBin;
};



#endif
