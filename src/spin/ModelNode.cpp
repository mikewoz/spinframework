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

//#include <osgFX/Scribe>
//#include <osgFX/Cartoon>


#include <osgUtil/SmoothingVisitor>
#include <osg/BoundingBox>
#include <osg/ImageStream>

#include "ModelNode.h"
#include "osgUtil.h"
#include "SceneManager.h"
#include "MediaManager.h"
#include "nodeVisitors.h"

#include "ImageTexture.h"
#include "VideoTexture.h"
#include "SharedVideoTexture.h"



using namespace std;

extern pthread_mutex_t pthreadLock;


// ===================================================================
// constructor:
ModelNode::ModelNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".ModelNode");
	nodeType = "ModelNode";

	_registerStates = false;
	_statesetList.clear();
    _renderBin = 10;

	modelPath = "NULL";


}

// ===================================================================
// destructor
ModelNode::~ModelNode()
{
	//std::cout << "Destroying ModelNode: " << this->id->s_name << std::endl;
}


// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void ModelNode::setContext (const char *newvalue)
{
	if (contextString==string(newvalue)) return;

	// need to redraw after setContext() is called:
	ReferencedNode::setContext(newvalue);
	drawModel();
}

void ModelNode::setModelFromFile (const char* filename)
{
	string path = getRelativePath(string(filename));
	
	// don't do anything if the current model is already loaded:
	if (path==modelPath) return;

	modelPath = path;

	drawModel();

	BROADCAST(this, "ss", "setModelFromFile", getModelFromFile());
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

void ModelNode::setStateSet (int i, const char *replacement)
{
	osg::ref_ptr<ReferencedStateSet> ssOrig, ssReplacement;

	if ( i < _statesetList.size() )
	{
		ssOrig = dynamic_cast<ReferencedStateSet*>(_statesetList[i]->s_thing);
	}

	ssReplacement = sceneManager->getStateSet(replacement);

	if (ssOrig.valid() && ssReplacement.valid())
	{
		for (int j=0; j < ssOrig->getNumParents(); j++)
		{
			// stateset can be shared by several parents within the model, but
			// they must be either of type osg::Drawable or osg::Node
			osg::Drawable *drawable = dynamic_cast<osg::Drawable*>(ssOrig->getParent(j));
			osg::Node *node = dynamic_cast<osg::Node*>(ssOrig->getParent(j));
			if (drawable) drawable->setStateSet(ssReplacement.get());
			else if (node) node->setStateSet(ssReplacement.get());
			else std::cout << "ERROR: setStateSet for model " << id->s_name << " had problems." << std::endl;
		}
		_statesetList[i] = ssReplacement->id;
		BROADCAST(this, "sis", "setStateSet", i, _statesetList[i]->s_name);
	}
}


// ===================================================================
// ===================================================================
// ===================================================================
void ModelNode::drawModel()
{
	int i,j;
	
	pthread_mutex_lock(&pthreadLock);

	if (model.valid())
	{

		this->getAttachmentNode()->removeChild(model.get());
		model = NULL;
		
		_statesetList.clear();
		
		if (sceneManager->sharedStateManager.valid()) sceneManager->sharedStateManager->prune();
		
		
		for (i=0; i<MODELNODE_NUM_ANIM_CONTROLS; i++)
		{
			// re-initialize:
			switcher[i] = NULL;
			sequencer[i] = NULL;
			animationMode[i] = OFF;
			animationNumFrames[i] = 0;
			state[i] = 0;
		}
	}

	bool ignoreOnThisHost = (sceneManager->isSlave() && (this->getContext()==getHostname()));
	
	if ((modelPath != string("NULL")) && !ignoreOnThisHost)
	{

		//model = dynamic_cast<osg::Group*>(osgDB::readNodeFile(modelPath));
		model = (osg::Group*)(osgDB::readNodeFile( getAbsolutePath(modelPath).c_str() ));

		if (model.valid())
		{
			
			if (sceneManager->sharedStateManager.valid())
				sceneManager->sharedStateManager->share(model.get());
		
			
			// *****************************************************************
			
			SearchVisitor searchVisitor;
			char buf[16];

			for (i=0; i<MODELNODE_NUM_ANIM_CONTROLS; i++)
			{

				sprintf( buf, "%02d", i );

				// Check if there are multiple states available from a osg::Switch
				// note: from 3DS exporter, switch nodes are called: OSG_Switch01, etc.
				searchVisitor.searchNode(model.get(), "OSG_Switch"+string(buf));

				switcher[i] = searchVisitor.getSwitchNode();
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
				searchVisitor.searchNode(model.get(), "OSG_Sequence"+string(buf));
				sequencer[i] = searchVisitor.getSequenceNode();
				if (sequencer[i].valid())
				{
					std::cout << "found OSG_Sequence0" << i << " with " << sequencer[i]->getNumChildren() << " frames" << std::endl;
					animationMode[i] = SEQUENCE;
					animationNumFrames[i] = sequencer[i]->getNumChildren();
					sequencer[i]->setValue(0);
					sequencer[i]->setMode(osg::Sequence::PAUSE);
				}

			}
			
			// *****************************************************************
			// search for special "billboard" nodes

			if (0)
			{
				NodeList foundNodes;
				NodeSearcher nodeSearcher(foundNodes);
				nodeSearcher.search(model.get(), "bill");

				//std::cout << "looking for billboards...." << std::endl;
				
				if (foundNodes.size())
					std::cout << "found " << foundNodes.size() << " nodes to be converted into billboards" << std::endl;
				
				
				int count = 0;
				for (NodeList::iterator itr=foundNodes.begin(); itr!=foundNodes.end(); ++itr)
				{
					///if (count++ > 300) break;
					
					//std::cout << "checking potential billboard: " << (*itr)->className() << ", " << (*itr)->getName() << std::endl;
					
					// keep reference of node:
					osg::ref_ptr<osg::Group> n = (*itr)->asGroup();
					//osg::ref_ptr<osg::Node> n = (*itr);
										
					
					if (n.valid())
					{
						const osg::BoundingSphere& bs = n->getBound();
						//osg::BoundingSphere bs = n->computeBound();
						//std::cout << "good billboard @ " << bs.center().x()<<","<<bs.center().y()<<","<<bs.center().z()<< std::endl;
						
						// AUTOTRANSFORM METHOD:

						
						// create autotranform node, and attach it to the parent(s) of
						// the billboard node
						osg::AutoTransform *at = new osg::AutoTransform();
						//at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
						at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_CAMERA);
						
						// attach the autotransform to the parent(s) of the 
						// group
						
						for (i=0; i<n->getNumParents(); i++)
						{
							n->getParent(i)->addChild(at);
						}
						
						//model->addChild(at);
						
						// now remove the group from it's parent(s) and attach
						// it to the autotransform:
						while (n->getNumParents())
						{
							n->getParent(0)->removeChild(n.get());
						}
						at->addChild(n.get());
						
						at->setPosition(bs.center());
						at->setPivotPoint(bs.center());

						
						
						
						// BILLBOARD METHOD:

						/*
						//i=0;
						for (i=0; i<n->getNumChildren(); i++)
						{
							// keep ref_ptr for geode
							osg::ref_ptr<osg::Geode> g = n->getChild(i)->asGeode();
							
							if (g.valid())
							{
								
								// create a billboard:
								osg::Billboard *b = new osg::Billboard();
								//osg::Billboard *b = dynamic_castMosg::Billboard*)g->clone(); // new osg::Billboard(g.get());
								b->setName("generated Billboard");
								b->setMode(osg::Billboard::AXIAL_ROT);
								b->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
								//b->setNormal(osg::Vec3(0.0f,1.0f,0.0f));
								//b->setMode(osg::Billboard::POINT_ROT_EYE);
								//b->setMode(osg::Billboard::POINT_ROT_WORLD);
								
								
								osg::PositionAttitudeTransform *PAT = new osg::PositionAttitudeTransform();
								model->addChild(PAT);
								PAT->setPosition(bs.center());
								
								
								// attach the billboard to the group
								PAT->addChild(b);
								
								
								// attach all of the geode's drawables to the billboard:
								for (j=0; j<g->getNumDrawables(); j++)
								{
									osg::Drawable *drawable = g->getDrawable(j);

									osg::Vec3 origPos = g->getPosition(j);
									std::cout << "drawable " << j << " origPos: " <<  origPos.x()<<","<<origPos.y()<<","<<origPos.z() << std::endl;
																	
									b->addDrawable(drawable);
								}
							
								// remove the geode from it's parent(s)
								n->removeChild(g.get());
								

							}
						}
						*/
						

						
					} // if (n.valid())
				} // end for
				
				
				
			}
			
			
		    
			optimizer.optimize(model.get());
			model->setName(string(id->s_name) + ".model['" + modelPath + "']");


			StateSetList statesets;
			TextureStateSetFinder f(statesets);
			model->accept(f);
			
			// *****************************************************************
			// Here, we search through all statesets and replace them with our
			// own generated RegisteredStateSet objects. This way, we're able to
			// control texture and shader information via OSC
			if (_registerStates)
			{
				int texNum = 0;
				for (StateSetList::iterator itr=statesets.begin(); itr!=statesets.end(); ++itr)
				{
					// check if this stateset has a special texture
					osg::StateAttribute *attr = (*itr)->getTextureAttribute(0,osg::StateAttribute::TEXTURE);
					if (attr)
					{

						std::string imageFile = attr->asTexture()->getImage(0)->getFileName();
						size_t pos;

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


						// NEW
						osg::ref_ptr<ReferencedStateSet> ss = sceneManager->createStateSet(osgDB::getNameLessExtension(imageFile).c_str());
						if (ss.valid())
						{
							ss->replace((*itr).get());
							this->_statesetList.push_back(ss->id);
							std::cout << "  Replaced placeholder texture with " << ss->classType << ": " << ss->id->s_name << std::endl;

						}

						// OLD:
						/*
						// check if the imagefile (minus extension) is a directory, and replace if necessary
						if (osgDB::getDirectoryContents(osgDB::getNameLessExtension(imageFile)).size())
						{
							osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->createStateSet(osgDB::getStrippedName(imageFile).c_str(), "VideoTexture"));

							//osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->createStateSet((string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str(), "VideoTexture"));
							//osg::ref_ptr<VideoTexture> vid = new VideoTexture(sceneManager, (string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str());
							if (vid.valid())
							{
								vid->setVideoPath(osgDB::getNameLessExtension(getRelativePath(imageFile)).c_str());
								vid->replace((*itr).get());
								//(*itr) = vid.get();
								std::cout << "  Replaced placeholder texture with " << vid->classType << ": " << vid->id->s_name << std::endl;
							}
						}

						// if filename contains "shared_video_texture", then replace
						// current TextureAttribute with a SharedVideoTexture
						if ((pos=imageFile.find("shared_video_texture01")) != string::npos)
						{
							std::string shID = "shvid_"+imageFile.substr(pos+20, imageFile.rfind(".")-(pos+20));
							osg::ref_ptr<SharedVideoTexture> shvid = dynamic_cast<SharedVideoTexture*>(sceneManager->createStateSet(shID.c_str(), "SharedVideoTexture"));
							if (shvid.valid())
							{
								shvid->setTextureID(shID.c_str());
								(*itr)->removeAttribute(attr);
								shvid->replace((*itr).get());
								std::cout << "  Replaced placeholder texture with " << shvid->classType << ": " << shvid->id->s_name << std::endl;
							}
						}

						// Check if filename (minus extension) is a movie format:
						//
						// ... the idea is that the model was created with using
						// a placeholder image with a name like: texture.avi.jpg
						//
						// ... we look for a video file in the same path, but with
						// the image extension stripped off: eg, texture.avi
						//
						// We replace the image with an ImageStream, and all YUV
						// texture mapping will remain (assuming the image and video
						// have identical resolutions.

						else if (isVideoPath(imageFileLessExtension))
						{
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

							std::cout << "... imageFileLessExtension: " << imageFileLessExtension << std::endl;
							std::cout << "... vidPath: " << vidPath << std::endl;
							std::cout << "--> " << (string(id->s_name)+"/"+osgDB::getStrippedName(imageFileLessExtension)).c_str() << std::endl;

							osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->createStateSet(osgDB::getStrippedName(imageFileLessExtension).c_str(), "VideoTexture"));
							//osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->createStateSet((string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str(), "VideoTexture"));
							//osg::ref_ptr<VideoTexture> vid = new VideoTexture(sceneManager, (string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str());
							if (vid.valid())
							{
								vid->setVideoPath(vidPath.c_str());
								vid->replace((*itr).get());
								//(*itr) = vid.get();
							}
						}
						*/



					} // if texture attribute
				} // stateset iterator
			} // if _registerStates
		    
			
			// *****************************************************************

			this->getAttachmentNode()->addChild(model.get());

			std::cout << "Created model " << modelPath << std::endl;
			osg::BoundingSphere bound = model->computeBound();
			osg::Vec3 c = bound.center();
			std::cout << "  center=" <<c.x()<<","<<c.y()<< ","<<c.z()<< "  radius=" << bound.radius() << "  numTextures=" << statesets.size() << std::endl;


			//osg::StateSet *modelStateSet = new osg::StateSet();
			//modelStateSet->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
			//model->setStateSet(modelStateSet);


			osg::StateSet *ss = model->getOrCreateStateSet();

            // Should we override our _renderBin value using ss->getBinNumber(),
            // or shoudld we apply our currently stored _renderBin to the model?
			ss->setRenderBinDetails( (int)_renderBin, "RenderBin");
			
			
		} else {
			std::cout << "ERROR [ModelNode::drawModel]: Could not find \"" << modelPath << "\". Make sure file exists, and that it is a valid 3D model." << std::endl;
		}
	}
	
	pthread_mutex_unlock(&pthreadLock);

}

std::vector<lo_message> ModelNode::getState ()
{

	// inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();

	lo_message msg;

	msg = lo_message_new();
	// note: this method MUST precede "setModelFromFile"
	lo_message_add(msg, "si", "setStateRegistration", getStateRegistration());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setModelFromFile", modelPath.c_str());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "si", "setRenderBin", getRenderBin());
	ret.push_back(msg);
	
	for (int i=0; _statesetList.size(); i++)
	{
		msg = lo_message_new();
        lo_message_add(msg, "sis", "setStateSet", i, _statesetList[i]->s_name);
        ret.push_back(msg);
	}

	return ret;
}
