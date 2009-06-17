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



#include <osgDB/ReadFile>
#include <osg/Group>

//#include <osgFX/Scribe>
//#include <osgFX/Cartoon>


#include <osgUtil/SmoothingVisitor>


#include "asGlobals.h"
#include "asModel.h"
#include "osgUtil.h"
#include "asSceneManager.h"
#include "asMediaManager.h"
#include "findNodeVisitor.h"



using namespace std;

//extern asSceneManager *sceneManager;
//extern asMediaManager *mediaManager;


// ===================================================================
// constructor:
asModel::asModel (asSceneManager *sceneManager, char *initID) : asReferenced(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".asModel");
	nodeType = "asModel";
	
	modelTransform = new osg::PositionAttitudeTransform();	
	modelTransform->setName(string(id->s_name) + ".modelTransform");
	this->addChild(modelTransform.get());
	
	modelID = 0;
	modelName = "NULL";

	// When children are attached to this, they get added to the attachNode:
	// NOTE: by changing this, we MUST override the updateNodePath() method!
	attachmentNode = modelTransform.get();

}

// ===================================================================
// destructor
asModel::~asModel()
{

}

// *****************************************************************************
// IMPORTANT:
// subclasses of asReferenced are allowed to contain complicated subgraphs, and
// can also change their attachmentNode so that children are attached anywhere
// in this subgraph. If that is the case, the updateNodePath() function MUST be
// overridden, and extra nodes must be manually pushed onto the currentNodePath.

void asModel::updateNodePath()
{
	currentNodePath.clear();
	if ((parent!=WORLD_SYMBOL) && (parent!=NULL_SYMBOL))
	{
		osg::ref_ptr<asReferenced> parentNode = parent->s_thing;
		if (parentNode.valid())
		{
			currentNodePath = parentNode->currentNodePath;
		}
	}
	
	// here, the nodePath includes the base osg::group, PLUS the modelTransform
	currentNodePath.push_back(this);
	currentNodePath.push_back(modelTransform.get());
	
	// now update NodePaths for all children:
	updateChildNodePaths();
	
}


// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void asModel::setTranslation (float x, float y, float z)
{
	modelTransform->setPosition(osg::Vec3(x,y,z));
	BROADCAST(this, "sfff", "setTranslation", x, y, z);
}

void asModel::setOrientation (float p, float r, float y)
{
	_orientation = osg::Vec3(p, r, y);
	
	osg::Quat q = osg::Quat( osg::DegreesToRadians(p), osg::Vec3d(1,0,0),
							 osg::DegreesToRadians(r), osg::Vec3d(0,1,0),
							 osg::DegreesToRadians(y), osg::Vec3d(0,0,1));
	
	modelTransform->setAttitude(q);
	BROADCAST(this, "sfff", "setOrientation", p, r, y);
}

void asModel::setScale (float x, float y, float z)
{
	modelTransform->setScale(osg::Vec3(x,y,z));
	BROADCAST(this, "sfff", "setScale", x, y, z);	
}

void asModel::setModel (int newModel)
{	
	// don't do anything if the current model is already loaded:
	if (modelID == newModel) return;
	
	modelID = newModel;
	modelName = mediaManager->getModelName(modelID);
	modelPath = mediaManager->getModelPath(modelID);

	drawModel();
	
	BROADCAST(this, "si", "setModel", modelID);

}

void asModel::setModelFromFile (char* s)
{
	// don't do anything if the current model is already loaded:
	if (string(s)==modelName) return;
	
	modelID = 0;
	modelName = string(s);
	modelPath = mediaManager->getModelPath(s);

	drawModel();
	
	BROADCAST(this, "ss", "setModelFromFile", modelName.c_str());
	
}


// ===================================================================
// ===================================================================
// ===================================================================
void asModel::drawModel()
{
	int i,j; 

	if (model.valid())
	{
			
		modelTransform->removeChild(model.get());
		model = NULL;
		for (i=0; i<ASMODEL_NUM_ANIM_CONTROLS; i++)
		{
			// re-initialize:
			switcher[i] = NULL;
			sequencer[i] = NULL;
			animationMode[i] = OFF;
			animationNumFrames[i] = 0;
			state[i] = 0;
		}
	}
		
	if (!modelPath.empty())
	{

		//model = dynamic_cast<osg::Group*>(osgDB::readNodeFile(modelPath));
		model = (osg::Group*)(osgDB::readNodeFile(modelPath));

		if (model.valid())
		{
			findNodeVisitor getAnimInterface;
			char buf[16];
					
			for (i=0; i<ASMODEL_NUM_ANIM_CONTROLS; i++)
			{
						
				sprintf( buf, "%02d", i );
						
				// Check if there are multiple states available from a osg::Switch
				// note: from 3DS exporter, switch nodes are called: OSG_Switch01, etc.
				getAnimInterface.searchNode(model.get(), "OSG_Switch"+string(buf));
				switcher[i] = getAnimInterface.getSwitchNode();
				if (switcher[i].valid())
				{
					std::cout << "found OSG_Switch0" << i << " with " << switcher[i]->getNumChildren() << " frames" << std::endl;
					animationMode[i] = SWITCH;
					animationNumFrames[i] = switcher[i]->getNumChildren();
					// initialize so only first frame is visible:
					switcher[i]->setValue(0, true);
					for (j=1; j<animationNumFrames[i]; j++) switcher[i]->setValue(j, false);
							
				}

				// Check if there is an osg::Sequence node.
				getAnimInterface.searchNode(model.get(), "OSG_Sequence"+string(buf));
				sequencer[i] = getAnimInterface.getSequenceNode();
				if (sequencer[i].valid())
				{
					std::cout << "found OSG_Sequence0" << i << " with " << sequencer[i]->getNumChildren() << " frames" << std::endl;
					animationMode[i] = SEQUENCE;
					animationNumFrames[i] = sequencer[i]->getNumChildren();
					sequencer[i]->setValue(0);
					sequencer[i]->setMode(osg::Sequence::PAUSE);
				}
						
			}
			optimizer.optimize(model.get());
			modelTransform->addChild(model.get());
			model->setName(string(id->s_name) + ".model['" + modelName + "']");

			//osg::StateSet *modelStateSet = new osg::StateSet();
			//modelStateSet->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
			//model->setStateSet(modelStateSet);
			
			std::cout << "Created model " << modelName << std::endl;
			osg::BoundingSphere bound = model->computeBound();
			osg::Vec3 c = bound.center();
			std::cout << "  center=" <<c.x()<<","<<c.y()<< ","<<c.z()<< "  radius=" << bound.radius() << std::endl;
					
		} else {
			std::cout << "ERROR [asModel::drawModel]: Could not find \"" << modelPath << "\". Make sure file exists, and that it is a valid 3D model." << std::endl;
		}
	}
}

std::vector<lo_message> asModel::getState ()
{

	// inherit state from base class
	std::vector<lo_message> ret = asReferenced::getState();
	
	lo_message msg;
	osg::Vec3 v;
	
	
	msg = lo_message_new();
	v = this->getTranslation();
	lo_message_add(msg, "sfff", "setTranslation", v.x(), v.y(), v.z());
	ret.push_back(msg);
	
	msg = lo_message_new();
	v = this->getOrientation();
	lo_message_add(msg, "sfff", "setOrientation", v.x(), v.y(), v.z());
	ret.push_back(msg);
	
	msg = lo_message_new();
	v = this->getScale();
	lo_message_add(msg, "sfff", "setScale", v.x(), v.y(), v.z());
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setModel", modelID);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setModelFromFile", modelName.c_str());
	ret.push_back(msg);

	return ret;
}
