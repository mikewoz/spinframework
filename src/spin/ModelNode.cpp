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



#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osg/Group>
#include <osg/Billboard>
#include <osg/AutoTransform>
#include <osg/MatrixTransform>
#include <osg/Switch>
#include <osg/Sequence>

//#include <osgFX/Scribe>
//#include <osgFX/Cartoon>


#include <osgUtil/SmoothingVisitor>
#include <osg/BoundingBox>
#include <osg/ImageStream>

#include "ModelNode.h"
#include "osgUtil.h"
#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"
#include "nodeVisitors.h"

#include "ImageTexture.h"
#include "VideoTexture.h"
#include "SharedVideoTexture.h"

extern pthread_mutex_t sceneMutex;

namespace spin
{

// ===================================================================
// constructor:
ModelNode::ModelNode (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{
	this->setName(this->getID() + ".ModelNode");
	this->setNodeType("ModelNode");

    // Save a pointer to the current attachmentNode (we will attach the loaded
    // 3D mesh there:
    _modelAttachmentNode = this->getAttachmentNode();
    
    // Now, add a new 'centroid' node, and make that the new attachmentNode.
    // Children of this node will be attached there, and an offset will be added
    // depending whether _attachCentroid is enabled or not:
    _centroid = new osg::PositionAttitudeTransform();
    _centroid->setName(getID()+".CentroidOffset");
    this->getAttachmentNode()->addChild(_centroid.get());
    this->setAttachmentNode(_centroid.get());

    _attachCentroid = false;
    _registerStates = false;
	_statesetList.clear();
    _statesetList.push_back(stateset_);
    _renderBin = 10;
    _lightingOverride = 0;

	modelPath = "NULL";


}

// ===================================================================
// destructor
ModelNode::~ModelNode()
{
	//std::cout << "Destroying ModelNode: " << this->getID() << std::endl;
}


void ModelNode::updateNodePath(bool updateChildren)
{
	// call GroupNode's method, which will update all the way from the root, and
	// we just need to add the centroid node:
	GroupNode::updateNodePath(false);
	currentNodePath_.push_back(_centroid.get());

    /*
    osg::NodePath::iterator iter;
    std::cout << "nodepath for " << getID() << ":" << std::endl;
    for (iter = currentNodePath_.begin(); iter!=currentNodePath_.end(); iter++)
    {
        std::cout << " > " << (*iter)->getName() << std::endl;
    }
    */
    
	// now update NodePaths for all children:
	updateChildNodePaths();
}


// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void ModelNode::setContext (const char *newvalue)
{
	if (getContextString()==std::string(newvalue)) return;

	// need to redraw after setContext() is called:
	ReferencedNode::setContext(newvalue);
	drawModel();
}

void ModelNode::setModelFromFile (const char* filename)
{
	std::string path = getRelativePath(std::string(filename));
	
	// don't do anything if the current model is already loaded:
	if (path==modelPath) return;

	modelPath = path;

	drawModel();

	BROADCAST(this, "ss", "setModelFromFile", getModelFromFile());
}

void ModelNode::setAttachCentroid (int i)
{
    _attachCentroid = (bool)i;

    // TODO: bound may change dynamically (eg, animations, switch nodes, etc)
    // so this should be done in the update callback whenever bound is made 
    // dirty(). Can we check for that?
    
    if (model.valid() && _attachCentroid)
    {	
        osg::BoundingSphere bound = model->computeBound();
        _centroid->setPosition(bound.center());
        osg::Vec3 c = bound.center();
        std::cout << "setting centroid for model: " <<c.x()<<","<<c.y()<< ","<<c.z() << std::endl;
    }
    else
    {
        std::cout << "setting centroid for model: 0,0,0" << std::endl;
        _centroid->setPosition(osg::Vec3(0.0,0.0,0.0));
    }
    		
    BROADCAST(this, "si", "setAttachCentroid", getAttachCentroid());
}

void ModelNode::makeCentered()
{
    if (model.valid())
    {
        osg::BoundingSphere bound = _modelAttachmentNode->computeBound();

        std::cout << "centering the model to centroid: ("<<bound.center().x()<<","<<bound.center().y()<<","<<bound.center().z()<<") length="<< bound.center().length() << std::endl;

        if (bound.center().length() > 0.00001)
        {
            _modelAttachmentNode->removeChild(model.get());

            osg::PositionAttitudeTransform *mpat = new osg::PositionAttitudeTransform();
            mpat->setName(getID()+".ModelAttachment");
            mpat->setPosition(-bound.center());

            mpat->addChild(model.get());
            _modelAttachmentNode->addChild(mpat);

            //mpat->addChild(model.get());
            //_modelAttachmentNode->replaceChild(model.get(), mpat);
        }

        _centroid->setPosition(osg::Vec3(0.0,0.0,0.0));

        bound = _modelAttachmentNode->computeBound();
        std::cout << "new centroid: ("<<bound.center().x()<<","<<bound.center().y()<<","<<bound.center().z()<<") length="<< bound.center().length() << std::endl;

    }


}

void ModelNode::setStateRegistration (int i)
{
	_registerStates = (bool)i;

	BROADCAST(this, "si", "setStateRegistration", getStateRegistration());
}

void ModelNode::setRenderBin (int i)
{
	_renderBin = i;

	if (model.valid())
	{
		osg::StateSet *ss = model->getOrCreateStateSet();
		ss->setRenderBinDetails( (int)_renderBin, "RenderBin");
	}

	BROADCAST(this, "si", "setRenderBin", _renderBin);
}

void ModelNode::setPlaying (int index, int playstate)
{
    _playState[index] = playstate;
    
    if (sequencer[index].valid())
    {
        osg::Sequence::SequenceMode mode = sequencer[index]->getMode();
        std::cout << "about to set mode to "<<_playState[index] << ", old="<<mode<<std::endl;
        
        sequencer[index]->setMode((osg::Sequence::SequenceMode)_playState[index]);
        
    
        std::cout << "osgSequence mode: " << sequencer[index]->getMode() << ", duration: " << sequencer[index]->getNumFrames()<< ", speed: " << sequencer[index]->getSpeed() << ", lastFrameTime: " << sequencer[index]->getLastFrameTime() <<std::endl;
    }
    else std::cout << "Warning: Model '" << this->getID() << "' has no Sequence for index " << index << std::endl;
    
    BROADCAST(this, "sii", "setPlaying", index, _playState[index]);
}
    
void ModelNode::setKeyframe (int index, float keyframe)
{
	_keyframe[index] = keyframe;

	if (switcher[index].valid())
	{
		for (unsigned j = 1; j < switcher[index]->getNumChildren(); j++) 
            switcher[index]->setValue(j, false);
		switcher[index]->setValue((int)(switcher[index]->getNumChildren()*_keyframe[index]), true);
	}

	else if (sequencer[index].valid())
	{
		sequencer[index]->setValue((int)(sequencer[index]->getNumChildren()*_keyframe[index]));
	}

	BROADCAST(this, "sif", "setKeyframe", index, _keyframe[index]);
}

void ModelNode::setStateSet (int i, const char *replacement)
{
    osg::ref_ptr<ReferencedStateSet> ssReplacement = sceneManager_->getStateSet(replacement);

    if (i==0)
    {
        osg::ref_ptr<ReferencedStateSet> ss = dynamic_cast<ReferencedStateSet*>(stateset_->s_thing);
        if (mainTransform_.valid() && ss.valid()) mainTransform_->setStateSet( ss.get() );
    }
    else if ((i>0) && (i<_statesetList.size()))
    {
        osg::ref_ptr<ReferencedStateSet> ssOrig = dynamic_cast<ReferencedStateSet*>(_statesetList[i]->s_thing);

        if (ssOrig.valid() && ssReplacement.valid())
        {
            // We need to search through the drawables and nodes that originally had
            // statesets and replace ssOrig with ssReplacement:
            std::vector<osg::Drawable*>::iterator dItr;
            for (dItr=_ssDrawableList.begin(); dItr!=_ssDrawableList.end(); ++dItr)
            {
                if ((*dItr)->getStateSet() == ssOrig.get())
                {
                    (*dItr)->setStateSet(ssReplacement.get());
                }
            }
            std::vector<osg::Node*>::iterator nItr;
            for (nItr=_ssNodeList.begin(); nItr!=_ssNodeList.end(); ++nItr)
            {
                if ((*nItr)->getStateSet() == ssOrig.get())
                {
                    (*nItr)->setStateSet(ssReplacement.get());
                }
            }
        }
    }
    
    if (ssReplacement.valid())
    {
        _statesetList[i] = ssReplacement->getIDSymbol();
        BROADCAST(this, "sis", "setStateSet", i, _statesetList[i]->s_name);
    }
}

void ModelNode::updateStateSet()
{
    // updateStateSet should do nothing for ModelNode.
    this->setStateSet(0, stateset_->s_name);
}

void ModelNode::setLighting(int i)
{

    if (_lightingOverride==(bool)i) return;
    _lightingOverride = (bool)i;

    if (model.valid())// && !stateset->s_thing)
    {
        osg::StateSet *ss = model->getOrCreateStateSet();
        if (_lightingOverride) ss->setMode( GL_LIGHTING, osg::StateAttribute::ON );
        else ss->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    }

    BROADCAST(this, "si", "setLighting", getLighting());

}

// ===================================================================
// ===================================================================
// ===================================================================
void ModelNode::drawModel()
{
	pthread_mutex_lock(&sceneMutex);
	
	if (model.valid())
	{

        _modelAttachmentNode->removeChild(model.get());
            
        model = NULL;
        _centroid->setPosition(osg::Vec3(0.0,0.0,0.0));
		_statesetList.clear();
        _statesetList.push_back(stateset_);
		_ssDrawableList.clear();
		_ssNodeList.clear();

		//if (sceneManager_->sharedStateManager.valid()) sceneManager_->sharedStateManager->prune();
		
		
		for (int i=0; i<MODELNODE_NUM_ANIM_CONTROLS; i++)
		{
			// re-initialize:
			switcher[i] = NULL;
			sequencer[i] = NULL;
			animationMode[i] = OFF;
			_keyframe[i] = 0;
		}
	}

	bool ignoreOnThisHost = (not spinApp::Instance().getContext()->isServer() and (this->getContext()==getHostname()));
	
	if ((modelPath != std::string("NULL")) && !ignoreOnThisHost)
	{

        //osg::setNotifyLevel(osg::DEBUG_FP);
		model = (osg::Group*)(osgDB::readNodeFile( getAbsolutePath(modelPath).c_str() ));
        //osg::setNotifyLevel(osg::FATAL);
        
		if (model.valid())
		{
			
			
			// *****************************************************************
			

			/*
			// This is a better way to do this:
			NodeList foundNodes;
			NodeSearcher nodeSearcher(foundNodes);
			nodeSearcher.search(model.get(), "OSG_Switch");
			if (foundNodes.size())
				std::cout << "found " << foundNodes.size() << " switch nodes" << std::endl;

			int count = 0;
			for (NodeList::iterator itr=foundNodes.begin(); itr!=foundNodes.end(); ++itr)
			{


			}
			*/

			SearchVisitor searchVisitor;
			char buf[16];
			for (int i=0; i<MODELNODE_NUM_ANIM_CONTROLS; i++)
			{

				sprintf( buf, "%02d", i );

				// Check if there are multiple states available from a osg::Switch
				// note: from 3DS exporter, switch nodes are called: OSG_Switch01, etc.
				searchVisitor.searchNode(model.get(), "OSG_Switch"+std::string(buf));

				switcher[i] = searchVisitor.getSwitchNode();
				if (switcher[i].valid())
				{
					std::cout << "found OSG_Switch" << buf << " with " << switcher[i]->getNumChildren() << " frames" << std::endl;
					animationMode[i] = SWITCH;
					// initialize so only first frame is visible:
					switcher[i]->setValue(0, true);
					for (int j=1; j<switcher[i]->getNumChildren(); j++) switcher[i]->setValue(j, false);

				}

				// Check if there is an osg::Sequence node.
				searchVisitor.searchNode(model.get(), "OSG_Sequence"+std::string(buf));
				sequencer[i] = searchVisitor.getSequenceNode();
				if (sequencer[i].valid())
				{
					std::cout << "found OSG_Sequence" << buf << " with " << sequencer[i]->getNumChildren() << " frames" << std::endl;
                    sequencer[i]->setDataVariance(osg::Object::DYNAMIC);
					animationMode[i] = SEQUENCE;
					sequencer[i]->setValue(0);
                    //sequencer[i]->setMode(osg::Sequence::PAUSE);
                    //sequencer[i]->setMode(osg::Sequence::START);
				}

			}
			
			// *****************************************************************
			// search for special "billboard" nodes

			//optimizer.optimize(model.get(), osgUtil::Optimizer::ALL_OPTIMIZATIONS);
			//optimizer.optimize(sceneManager_->worldNode.get(), osgUtil::Optimizer::ALL_OPTIMIZATIONS);
			optimizer.optimize(model.get());
			/*
            optimizer.optimize(model.get(),
                osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS |
                osgUtil::Optimizer::REMOVE_REDUNDANT_NODES |
                osgUtil::Optimizer::REMOVE_LOADED_PROXY_NODES |
                osgUtil::Optimizer::COMBINE_ADJACENT_LODS |
                osgUtil::Optimizer::SHARE_DUPLICATE_STATE |
                osgUtil::Optimizer::MERGE_GEOMETRY |
                osgUtil::Optimizer::CHECK_GEOMETRY |
                osgUtil::Optimizer::SPATIALIZE_GROUPS | 
                osgUtil::Optimizer::COPY_SHARED_NODES | 
                osgUtil::Optimizer::TRISTRIP_GEOMETRY |
                osgUtil::Optimizer::TESSELLATE_GEOMETRY |
                osgUtil::Optimizer::OPTIMIZE_TEXTURE_SETTINGS |
                osgUtil::Optimizer::MERGE_GEODES |
                osgUtil::Optimizer::FLATTEN_BILLBOARDS |
                //osgUtil::Optimizer::TEXTURE_ATLAS_BUILDER |
                osgUtil::Optimizer::STATIC_OBJECT_DETECTION |
                osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS_DUPLICATING_SHARED_SUBGRAPHS 
			);
            */

			StateSetList statesets;
			TextureStateSetFinder f(statesets);
			model->accept(f);
			
			// *****************************************************************
			// Here, we search through all statesets and replace them with our
			// own generated RegisteredStateSet objects. This way, we're able to
			// control texture and shader information via OSC
			if (_registerStates)
			{
				for (StateSetList::iterator itr=statesets.begin(); itr!=statesets.end(); ++itr)
				{
					// check if this stateset has a special texture
					osg::StateAttribute *attr = (*itr)->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
					if (attr)
					{

						std::string imageFile = attr->asTexture()->getImage(0)->getFileName();

						// If file came from other OS, we should fix it:
						imageFile = osgDB::convertFileNameToNativeStyle(imageFile);
						std::string imageFileLessExtension = imageFile.substr(0, imageFile.rfind("."));

						// in Linux, imageFile is relative, so check if it
						// exists and prepend the modelPath in case:
						if (!osgDB::fileExists(imageFile))
						{
							imageFile = osgDB::concatPaths(osgDB::getFilePath(getAbsolutePath(modelPath)), imageFile);
						}

						/*
						std::string vidPath;
						if (imageFileLessExtension.substr(1)=="/")
						{
							// absolute path, so don't change it
							vidPath = imageFileLessExtension;
						}
						else {
							// relative path:
							vidPath = osgDB::concatPaths(osgDB::getFilePath(getAbsolutePath(modelPath)), imageFileLessExtension);
						}
						*/

						// Now, in order to replace this stateset with another
						// in the future, we need to have pointers to all of the
						// drawables/nodes that originally used it. So before we
						// do anything, we store a big ugly array of all these
						// pointers:
						for (int i=0; i < (*itr)->getNumParents(); i++)
						{
							// stateset can be shared by several parents within
							// the model, but they must be of type osg::Drawable
							// or osg::Node

							osg::Drawable *drawable = dynamic_cast<osg::Drawable*>((*itr)->getParent(i));
							osg::Node *node = dynamic_cast<osg::Node*>((*itr)->getParent(i));
							if (drawable) _ssDrawableList.push_back(drawable);
							if (node) _ssNodeList.push_back(node);
						}


						// NEW
						//osg::ref_ptr<ReferencedStateSet> ss = sceneManager_->createStateSet(osgDB::getNameLessExtension(imageFile).c_str());
						osg::ref_ptr<ReferencedStateSet> ss = sceneManager_->createStateSet(imageFile.c_str());
						if (ss.valid())
						{
							ss->replace((*itr).get());
							this->_statesetList.push_back(ss->getIDSymbol());
							std::cout << "  Replaced placeholder texture with " << ss->getClassType() << ": " << ss->getID() << std::endl;
                            
                            //
                            std::cout << "Now we have " << _statesetList.size() << " statesets:" << std::endl;
                            for (int i=0; i<_statesetList.size(); i++)
                            {
                                std::cout << " - " << _statesetList[i]->s_name << std::endl;
                            }

						}

					} // if texture attribute
				} // stateset iterator
			} // if _registerStates
		    
			
			// *****************************************************************

			_modelAttachmentNode->addChild(model.get());
            model->setName(getID()+".file['"+modelPath+"']");
            //model->setName(this->getID() + ".model['" + modelPath + "']");

			std::cout << "Created model " << modelPath << std::endl;
			osg::BoundingSphere bound = model->computeBound();
			osg::Vec3 c = bound.center();
			std::cout << "  center=" <<c.x()<<","<<c.y()<< ","<<c.z()<< "  radius=" << bound.radius() << "  numTextures=" << statesets.size() << std::endl;

            if (_attachCentroid)
                _centroid->setPosition(c);
            
			//osg::StateSet *modelStateSet = new osg::StateSet();
			//modelStateSet->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
			//model->setStateSet(modelStateSet);


			osg::StateSet *ss = model->getOrCreateStateSet();

            // Should we override our _renderBin value using ss->getBinNumber(),
            // or shoudld we apply our currently stored _renderBin to the model?
			ss->setRenderBinDetails( (int)_renderBin, "RenderBin");

			/*
			if (sceneManager_->sharedStateManager.valid())
				sceneManager_->sharedStateManager->share(model.get());
			*/
			
		} else {
			std::cout << "ERROR [ModelNode::drawModel]: Could not find \"" << modelPath << "\". Make sure file exists, and that it is a valid 3D model." << std::endl;
		}
	}
	
	pthread_mutex_unlock(&sceneMutex);
}

std::vector<lo_message> ModelNode::getState () const
{
	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	int i;
	lo_message msg;

	msg = lo_message_new();
	// note: this method MUST precede "setModelFromFile"
	lo_message_add(msg, "si", "setStateRegistration", getStateRegistration());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setModelFromFile", modelPath.c_str());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setAttachCentroid", getAttachCentroid());
	ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "si", "setLighting", getLighting());
    ret.push_back(msg);
    
	for (i=0; i<MODELNODE_NUM_ANIM_CONTROLS; i++)
	{
		if (switcher[i].valid() || sequencer[i].valid())
		{
            msg = lo_message_new();
			lo_message_add(msg, "sii", "setPlaying", i, _playState[i]);
			ret.push_back(msg);

			msg = lo_message_new();
			lo_message_add(msg, "sif", "setKeyframe", i, _keyframe[i]);
			ret.push_back(msg);
		}
	}

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);

	for (i=1; i<_statesetList.size(); i++)
	{
		msg = lo_message_new();
        lo_message_add(msg, "sis", "setStateSet", i, _statesetList[i]->s_name);
        ret.push_back(msg);
	}

	return ret;
}

} // end of namespace spin

