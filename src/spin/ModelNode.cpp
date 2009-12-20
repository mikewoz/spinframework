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
#include "VideoTexture.h"



using namespace std;

extern pthread_mutex_t pthreadLock;


// ===================================================================
// constructor:
ModelNode::ModelNode (SceneManager *sceneManager, char *initID) : GroupNode(sceneManager, initID)
{
	this->setName(string(id->s_name) + ".ModelNode");
	nodeType = "ModelNode";

	modelPath = "NULL";


}

// ===================================================================
// destructor
ModelNode::~ModelNode()
{
	std::cout << "Destroying ModelNode: " << this->id->s_name << std::endl;

#ifdef WITH_SHARED_VIDEO
	// unload any registered shared memory textures:
	/*
	std::vector<SharedVideoTexture*>::iterator shvItr = sharedVideoTextures.begin();
	while (shvItr!=sharedVideoTextures.end())
	{
		std::cout << "cleaning SharedVideoTexture " << (*shvItr)->getTextureID() << std::endl;
		(*shvItr)->unload(); 
		sharedVideoTextures.erase(shvItr);
	}

	std::vector< osg::ref_ptr<SharedVideoTexture> >::iterator shvItr = sharedVideoTextures.begin();
	while (shvItr!=sharedVideoTextures.end())
	{
		std::cout << "cleaning SharedVideoTexture " << (*shvItr)->getTextureID() << std::endl;
		(*shvItr)->unload(); 
		sharedVideoTextures.erase(shvItr);
	}
*/

#endif
}


// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void ModelNode::setHost (const char *newvalue)
{
	// need to redraw after setHost() is called:
	if (host != string(newvalue))
	{
		ReferencedNode::setHost(newvalue);
		drawModel();
	}
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

// ===================================================================
// ===================================================================
// ===================================================================
void ModelNode::drawModel()
{
	int i,j;
	
	std::cout << "in drawModel()" << std::endl;

	pthread_mutex_lock(&pthreadLock);

	if (model.valid())
	{

		this->getAttachmentNode()->removeChild(model.get());
		model = NULL;
		
		/*
		if (sceneManager->sharedStateManager.valid()) sceneManager->sharedStateManager->prune();
		*/
		
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

	bool ignoreOnThisHost = (sceneManager->isSlave() && (host==getHostname()));
	
	if ((modelPath != string("NULL")) && !ignoreOnThisHost)
	{

		//model = dynamic_cast<osg::Group*>(osgDB::readNodeFile(modelPath));
		model = (osg::Group*)(osgDB::readNodeFile( getAbsolutePath(modelPath).c_str() ));

		if (model.valid())
		{
			/*
			if (sceneManager->sharedStateManager.valid())
				sceneManager->sharedStateManager->share(model);
			 */
			
			SearchVisitor getAnimInterface;
			char buf[16];

			for (i=0; i<MODELNODE_NUM_ANIM_CONTROLS; i++)
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
			this->getAttachmentNode()->addChild(model.get());
			model->setName(string(id->s_name) + ".model['" + modelPath + "']");

			//osg::StateSet *modelStateSet = new osg::StateSet();
			//modelStateSet->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
			//model->setStateSet(modelStateSet);
			
			// search for all statesets with textures:
			StateSetList statesets;
			statesets.clear();
			TextureStateSetFinder f(statesets);
		    model->accept(f);
			

			std::cout << "Created model " << modelPath << std::endl;
			osg::BoundingSphere bound = model->computeBound();
			osg::Vec3 c = bound.center();
			std::cout << "  center=" <<c.x()<<","<<c.y()<< ","<<c.z()<< "  radius=" << bound.radius() << "  numTextures=" << statesets.size() << std::endl;

		    // Here, we replace some custom placeholder textures with custom
		    // classes.
		    
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
			    		
			    		// in Linux, imageFile is relative, so check if it
			    		// exists and prepend the modelPath in case:
			    		if (!osgDB::fileExists(imageFile))
			    		{
							imageFile = osgDB::concatPaths(osgDB::getFilePath(getAbsolutePath(modelPath)), imageFile);
			    		}	
			    		
			    		// check if the imagefile (minus extension) is a directory, and replace if necessary
			    		if (osgDB::getDirectoryContents(osgDB::getNameLessExtension(imageFile)).size())
			    		{
			    			osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->getOrCreateState(osgDB::getStrippedName(imageFile).c_str(), "VideoTexture"));
			    			//osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->getOrCreateState((string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str(), "VideoTexture"));
					    	//osg::ref_ptr<VideoTexture> vid = new VideoTexture(sceneManager, (string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str());
			    			if (vid.valid())
			    			{
			    				vid->setVideoPath(osgDB::getNameLessExtension(imageFile).c_str());
			    				vid->replace((*itr).get());
			    				//(*itr) = vid.get();
			    				std::cout << "  Replaced placeholder texture with " << vid->classType << ": " << vid->id->s_name << std::endl;
			    			}
			    			
			    		}
			    		
#ifdef WITH_SHARED_VIDEO			    		
			    		// if filename contains "shared_video_texture", then replace
			    		// current TextureAttribute with a SharedVideoTexture
			    		if ((pos=imageFile.find("shared_video_texture01")) != string::npos)
			    		{
			    			if (sceneManager->isGraphical())
				    		{
								// find the shared memory id from the filename:
					    		std::string shID = "shvid_"+imageFile.substr(pos+20, imageFile.rfind(".")-(pos+20));
					    					
								// see if an instance already exists:
								/*
								SharedVideoTexture *shTex=0;
								std::cout << "num existing sharedtextures: " << sharedVideoTextures.size() << std::endl;
								
								//std::vector< osg::ref_ptr<SharedVideoTexture> >::iterator shvItr;
								std::vector<SharedVideoTexture*>::iterator shvItr;
								
								for (shvItr=sharedVideoTextures.begin(); shvItr!=sharedVideoTextures.end(); ++shvItr)
								{
									std::cout << "comparing id with " << (*shvItr)->getTextureID() << std::endl;
									if (string((*shvItr)->getTextureID())==shID)
									{
										std::cout << "... already exists" << std::endl;
										//shTex = (*shvItr).get();
										shTex = (*shvItr);
									}
								}
								
								// if it doesn't exist, create a new one:
								if (!shTex)
								{
									std::cout << "... doesn't exist. creating." << std::endl;
									shTex = new SharedVideoTexture(shID.c_str());
					    		
									// add it to the list:
									sharedVideoTextures.push_back(shTex);
									
								}
								*/
								
								if (!sceneManager->shTex.valid())
								{
									std::cout << "making new SharedVideoTexture" << std::endl;
									sceneManager->shTex = new SharedVideoTexture(shID.c_str());
								}
									//sharedVideoTextures.push_back(shTex);
								
								
								
								// Finally, replace the texture attribute:
								(*itr)->setTextureAttributeAndModes(0, sceneManager->shTex, osg::StateAttribute::ON);
								
					    		// turn off lighting 
					    		(*itr)->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	
				    			
					    		/*
							    osg::ref_ptr<osg::TexMat> texmat(new osg::TexMat);
							    texmat->setScaleByTextureRectangleSize(true);
					
							    osg::ref_ptr<osg::TexEnv> texenv(new osg::TexEnv);
							    texenv->setMode(osg::TexEnv::REPLACE);
					
							    (*itr)->setTextureAttributeAndModes(0, textureRect.get(), osg::StateAttribute::ON);
							    (*itr)->setTextureAttributeAndModes(0, texmat.get(), osg::StateAttribute::ON);
							    (*itr)->setTextureAttributeAndModes(0, texenv.get(), osg::StateAttribute::ON);
					
							    // turn off lighting 
							    (*itr)->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
							    */
					    			
					    		std::cout << "  replaced '" << imageFile.substr(pos) << "' with SharedVideoTexture: " << shID << std::endl;
			    		
				    		}
			    		}
#endif
			    		
			    		// if filename is a movie format, create an ImageStream
			    		// and replace the current TextureAttribute:
			    		else if ((pos=imageFile.find(".mp4") != string::npos) ||
					        (pos=imageFile.find(".avi") != string::npos) ||
					        (pos=imageFile.find(".mov") != string::npos) )
			    		{
			    			osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->getOrCreateState(osgDB::getStrippedName(imageFile).c_str(), "VideoTexture"));
			    			//osg::ref_ptr<VideoTexture> vid = dynamic_cast<VideoTexture*>(sceneManager->getOrCreateState((string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str(), "VideoTexture"));
			    			//osg::ref_ptr<VideoTexture> vid = new VideoTexture(sceneManager, (string(id->s_name)+"/"+osgDB::getStrippedName(imageFile)).c_str());
			    			if (vid.valid())
			    			{
			    				vid->setVideoPath(imageFile.c_str());
			    				vid->replace((*itr).get());
			    				//(*itr) = vid.get();
			    			}
			    		}
			    		
			    		
			    	} // if texture attribute
			    } // stateset iterator
		    } // if (sceneManager->isGraphical())
		    
		    
			
			
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
	lo_message_add(msg, "ss", "setModelFromFile", modelPath.c_str());
	ret.push_back(msg);

	
	return ret;
}
