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

#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
//#include <osgUtil/Optimizer>
//#include <osgUtil/SmoothingVisitor>
//#include <osg/TessellationHints>

#include "SoundNode.h"
#include "SceneManager.h"
#include "osgUtil.h"

using namespace std;

//extern SceneManager *sceneManager;

extern pthread_mutex_t pthreadLock;

// ===================================================================
// constructor:
SoundNode::SoundNode (SceneManager *sceneManager, char *initID) : DSPNode(sceneManager, initID)
{
	nodeType = "SoundNode";
	this->setName(string(id->s_name) + ".SoundNode");

	_rolloff = "default";
	_spread = 1.0f;
	_length = 10.0f;
	
	directivityFlag = 0;
	laserFlag = 0;
	VUmeterFlag = 0;	
	
	currentSoundIntensity = 0.0;
	currentSoundColor = osg::Vec3(0.0,1.0,0.0); //green

	// OSG Stuff:
	laserGeode = NULL;
	directivityGeode = NULL;
	VUmeterTransform = NULL;
	
	


}

// ===================================================================
// destructor
SoundNode::~SoundNode()
{
	//std::cout << "In SoundNode destructor... node: " << this->id->s_name << std::endl;
}



// ===================================================================
// ======================== SET METHODS: =============================
// ===================================================================

void SoundNode::setRolloff (const char *newvalue)
{
	// We store just the id instead of the whole table. If we want, we can always get the table
	// from the database (eg, for drawing).
	_rolloff = newvalue;
	drawDirectivity();
	BROADCAST(this, "ss", "setRolloff", _rolloff.c_str());
}

void SoundNode::setSpread (float newvalue)
{
	_spread = newvalue;
	drawDirectivity();
	BROADCAST(this, "sf", "setSpread", _spread);
}

void SoundNode::setLength (float newvalue)
{
	_length = newvalue;
	drawDirectivity();
	BROADCAST(this, "sf", "setLength", _length);
}

void SoundNode::setDirectivityFlag (float newFlag)
{
	directivityFlag = newFlag;
	drawDirectivity();
	BROADCAST(this, "sf", "setDirectivityFlag", directivityFlag);
}

void SoundNode::setLaserFlag (float newFlag)
{
	laserFlag = newFlag; // note continuous value is used for alpha
	drawLaser();
	BROADCAST(this, "sf", "setLaserFlag", laserFlag);
}

void SoundNode::setVUmeterFlag (float newFlag)
{
	VUmeterFlag = newFlag; // note continuous value is used for alpha
	drawVUmeter();
	BROADCAST(this, "sf", "setVUmeterFlag", VUmeterFlag);
}

void SoundNode::setIntensity (float newvalue)
{
	currentSoundIntensity = newvalue;
	
	float r = currentSoundIntensity / 0.896909;
	if (r > 1.0) r=1.0;
	float g = (1.0 - currentSoundIntensity) / 0.103091;
	if (g > 1.0) g=1.0;
	currentSoundColor = osg::Vec3(r, g, 0.0);
	

	// too heavy!:
	
	//drawVUmeter();
	//drawLaser();
	
	updateVUmeter();
	updateLaser();

	BROADCAST(this, "sf", "setIntensity", currentSoundIntensity);
}

// -----------------------------------------------------------------------------
// Update methods:

void SoundNode::updateVUmeter ()
{

	if (VUmeterTransform.valid())
	{
		
		for (int i=0; i<VUmeterTransform->getNumChildren(); i++)
		{
			// update color of all drawables:
			osg::ref_ptr<osg::Geode> tmpGeode = dynamic_cast<osg::Geode*>(VUmeterTransform->getChild(i));
			if (tmpGeode.valid())
			{
				for (int j=0; j<tmpGeode->getNumDrawables(); j++)
				{
					osg::ShapeDrawable *s = dynamic_cast<osg::ShapeDrawable*>(tmpGeode->getDrawable(j));
					if (s) s->setColor( osg::Vec4(currentSoundColor, VUmeterFlag) );
				}
			}
		}

		VUmeterTransform->setScale( osg::Vec3(1, 1, 1 + currentSoundIntensity) );

	}
}

void SoundNode::updateLaser()
{

	if (laserGeode.valid())
	{
		for (int j=0; j<laserGeode->getNumDrawables(); j++)
		{
			osg::ShapeDrawable *s = dynamic_cast<osg::ShapeDrawable*>(laserGeode->getDrawable(j));
			if (s) s->setColor( osg::Vec4(currentSoundColor, laserFlag) );
		}
	}
}





// -----------------------------------------------------------------------------
// DRAW METHODS:

void SoundNode::drawVUmeter()
{
    pthread_mutex_lock(&pthreadLock);

		
	if (this->getAttachmentNode()->containsNode(VUmeterTransform.get()) )
	{
		this->getAttachmentNode()->removeChild(VUmeterTransform.get());
		VUmeterTransform = NULL;
	}
		
	if (VUmeterFlag>0)
	{
		// draw intensity geodes (cone and cap)

		osg::StateSet* VUmeterStateSet = new osg::StateSet();
		VUmeterStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		VUmeterStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF);
		VUmeterStateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		osg::Geode *VUmeterBaseGeode = new osg::Geode();
		osg::Geode *VUmeterCapGeode = new osg::Geode();
		
		osg::TessellationHints* hints = new osg::TessellationHints;
		hints->setDetailRatio(0.5f);
		
		//osg::ShapeDrawable* VUmeterBaseDrawable = new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(0.0f,0.0f,0.0f),AS_UNIT_SCALE*.1, AS_UNIT_SCALE),hints);
		osg::ShapeDrawable* VUmeterBaseDrawable = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0.0f,0.0f,0.0f),AS_UNIT_SCALE*.1, AS_UNIT_SCALE),hints);
		VUmeterBaseGeode->setStateSet( VUmeterStateSet );
		VUmeterBaseGeode->addDrawable( VUmeterBaseDrawable );
	
		osg::ShapeDrawable *VUmeterCapDrawable = new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f,0.0f,AS_UNIT_SCALE*.5+AS_UNIT_SCALE*.05),AS_UNIT_SCALE*.2,AS_UNIT_SCALE*.2),hints);
		VUmeterCapGeode->setStateSet(VUmeterStateSet );
		VUmeterCapGeode->addDrawable(VUmeterCapDrawable);
		
		VUmeterTransform = new osg::PositionAttitudeTransform();
		VUmeterTransform->addChild(VUmeterBaseGeode);
		VUmeterTransform->addChild(VUmeterCapGeode);
			
		// set the scale/color according to the current sound intensity:
		updateVUmeter();

		VUmeterTransform->setName(string(id->s_name) + ".VUmeterTransform");
		this->getAttachmentNode()->addChild(VUmeterTransform.get());
   }

    pthread_mutex_unlock(&pthreadLock);
}



t_float cardioid_to_cone_map[] = {180.0, 155.047, 147.605, 140.163, 116.907, 113.651, 106.674, 101.344, 97.1575, 93.6512, 90.8605, 89, 82.3043, 79.0484, 74.3973, 71.6065, 69.7461, 67.4205, 64.6298, 59.5135, 57.653, 55.7926, 54.8624, 53.467, 52.0717, 49.7461, 48.3507, 45, 44.9285, 42.9762, 41.8048, 41.4143, 40.6334, 39.8525, 38.6811, 38.2907, 37.1193, 36.3384, 35.9479, 35.167, 34.7766, 34.3861, 33.6052, 32.4338, 31.6529, 30.872, 30.4816, 30.4816, 29.7006, 29.3102, 28.5293, 28.1388, 27.7484, 26.9675, 26.9675, 26.577, 25.7961, 25.4056, 24.6247, 24.2343, 23.0629, 23.0629, 22.3043, 20.9089, 20.9089, 20.4438, 19.9787, 19.5136, 19.0485, 18.5834, 18.1182, 17.6531, 17.188, 16.7229, 16.7229, 16.2578, 16.2578, 15.7927, 15.7927, 15.3275, 15.3275, 15.0515, 14.4993, 14.3973, 13.9322, 13.4671, 13.002, 13.002, 12.7259, 12.4498, 12.4498, 12.4498, 12.1737, 11.6066, 11.6066, 11.6066, 11.6066, 11.3305, 11.3305, 10.6764, 10, 10.1242, 9.84816, 9.28103, 8.81591, 8.81591, 8.81591, 8.53984, 8.53984, 7.88568, 7.42056, 7.14449, 7.14449, 6.49033, 6.49033, 6.0066, 6.0066, 5.70254, 5.39849, 5.39849, 5.09443, 5.38037, 5.38037, 5.10429, 5.10429, 4.82822, 4.82822, 4.82822, 4.82822, 4.82822, 4.55215, 4.27607, 4.27607, 4.27607, 4, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 4, 3.72393, 3.72393, 3.44785, 3.17178, 3.17178, 3.17178, 3.17178, 3.17178, 3.17178, 3.17178, 2.89571, 2.89571, 2.89571, 2.89571, 2.61963, 2.61963, 2.61963, 2.34356, 2.34356, 2.34356, 2.34356, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.27607, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

// ===================================================================
void SoundNode::drawDirectivity()
{
    pthread_mutex_lock(&pthreadLock);

	
	if (this->getAttachmentNode()->containsNode(directivityGeode.get()))
	{
		this->getAttachmentNode()->removeChild(directivityGeode.get());
		directivityGeode = NULL;
	}

	if (directivityFlag > 0)
	{
		t_float val;
		t_float index = _spread * 16.6666;
		if ( (index < 0) || ((int)index >= 199) ) return;
		else val = cardioid_to_cone_map[(int)index];

		//printf("drawing directivity: _spread=%.3f  index=%.3f  val=%.3f\n", _spread, index, val);

		
		// *** NEW METHOD (NOT WORKING YET):
		// directivityGeode = createWireframeRolloff( this->_rolloff, this->_spread, AS_DEBUG_SCALE, osg::Vec4(DEFAULT_DIRECTIVITY_COLOR,1.0) );

		// *** OLD METHOD:
		
		if (val < 90)
		{
			// Place a cone pointing along the +Y axis
			directivityGeode = createHollowCone( _length*AS_DEBUG_SCALE, AS_DEBUG_SCALE*_length*tan(osg::DegreesToRadians(val)), osg::Vec4(DEFAULT_DIRECTIVITY_COLOR,1.0) );
		} else if (val > 90 && val < 180) {
			// Place a cone pointing along the -Y axis
			directivityGeode = createHollowCone( -_length*AS_DEBUG_SCALE, AS_DEBUG_SCALE*_length*tan(osg::DegreesToRadians(val)), osg::Vec4(DEFAULT_DIRECTIVITY_COLOR,1.0) );
		} else {
			// Sphere
			directivityGeode= createHollowSphere(_length*AS_DEBUG_SCALE, osg::Vec4(DEFAULT_DIRECTIVITY_COLOR,1.0) );
		}
		
		// ***
		
		
		
		osg::StateSet *wireframeStateSet = new osg::StateSet();
		osg::PolygonMode *polymode = new osg::PolygonMode;
		polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
		wireframeStateSet->setAttributeAndModes(polymode,osg::StateAttribute::ON);
		wireframeStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
		wireframeStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
		
		directivityGeode->setStateSet(wireframeStateSet);
		
		directivityGeode->setName(string(id->s_name) + ".directivityGeode");
		this->getAttachmentNode()->addChild(directivityGeode.get());

	}
	
    pthread_mutex_unlock(&pthreadLock);

}

// ===================================================================
void SoundNode::drawLaser()
{
	
	if (this->getAttachmentNode()->containsNode(laserGeode.get()))
	{
		this->getAttachmentNode()->removeChild(laserGeode.get());
		laserGeode = NULL;
	}

	if (laserFlag > 0)
	{  
		// create geode to hold lase:
		laserGeode = new osg::Geode();
		laserGeode->setName(string(id->s_name) + ".laserGeode");
 		 
		// draw laser as a cylinder:
		osg::Vec3 center;
		osg::Cylinder *laser;
		osg::Quat rot;
		center = osg::Vec3(0.0f,AS_DEBUG_SCALE*_length*.5,0.0f);
		laser = new osg::Cylinder(center, AS_LASER_RADIUS, AS_DEBUG_SCALE*_length);
		rot.makeRotate(osg::Vec3(0,0,1),center);
		laser->setRotation(rot);
		
		// make drawable and add to geode:
		osg::TessellationHints* hints = new osg::TessellationHints;
		hints->setDetailRatio(GENERIC_SHAPE_RESOLUTION);
		osg::ShapeDrawable* laserDrawable = new osg::ShapeDrawable(laser,hints);
		laserGeode->addDrawable(laserDrawable);
		
		// turn off lighting effects:
		osg::StateSet *laserStateSet = new osg::StateSet;
		laserStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF);			
		laserGeode->setStateSet ( laserStateSet );
		
		// update the color based on the current sound intensity:
		updateLaser();

		// add it to the node:
		this->getAttachmentNode()->addChild( laserGeode.get() );
	}
}

std::vector<lo_message> SoundNode::getState ()
{
	// inherit state from base class
	std::vector<lo_message> ret = DSPNode::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setRolloff", _rolloff.c_str());
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setSpread", _spread);
	ret.push_back(msg);
	
	msg = lo_message_new();
	lo_message_add(msg, "sf", "setLength", _length);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setDirectivityFlag", directivityFlag);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setLaserFlag", laserFlag);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setVUmeterFlag", VUmeterFlag);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "sf", "setIntensity", currentSoundIntensity);
	ret.push_back(msg);
	
	// have to re-send setContext AFTER setPlugin, so that loaded plugins will
	// have the up-to-date parameter
	msg = lo_message_new();
	lo_message_add(msg, "ss", "setContext", getContext());
	ret.push_back(msg);

	
	
	return ret;
}
