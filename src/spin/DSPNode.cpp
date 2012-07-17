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

#include <iostream>
#include "DSPNode.h"
#include "SoundConnection.h"

#include "SceneManager.h"
#include "spinApp.h"
#include "spinBaseContext.h"


#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osg/Material>


namespace {
    const double AS_LASER_RADIUS = 0.0025;
    const osg::Vec3 DEFAULT_DIRECTIVITY_COLOR(0.0,0.0,1.0); //blue
} // end anonymous namespace

extern pthread_mutex_t sceneMutex;

namespace spin
{

//extern SceneManager *sceneManager;

// *****************************************************************************
// constructor:
DSPNode::DSPNode (SceneManager *sceneManager, const char* initID) : GroupNode(sceneManager, initID)
{

	this->setNodeType("DSPNode");
	
	// enable report of globals by default:
	setReportMode(GroupNode::GLOBAL_6DOF);
	
	// connection stuff:
	connectTO.clear();
	connectFROM.clear();
	
	active = 1;
	
	plugin = "empty~";

    _rolloff = "default";
    _spread = 1.0f;
    _length = 1.0f;
    _radius = 0.0f;

    directivityFlag = 0;
    laserFlag = 0;
    VUmeterFlag = 0;
    radiusFlag = 0;

    currentSoundIntensity = 0.0;
    currentSoundColor = osg::Vec3(0.0,1.0,0.0); //green

    debugColor = osg::Vec4(DEFAULT_DIRECTIVITY_COLOR,1.0);

    // OSG Stuff:
    laserGeode = NULL;
    radiusGeode = NULL;
    directivityGeode = NULL;
    VUmeterTransform = NULL;
	
}

// *****************************************************************************
// destructor
DSPNode::~DSPNode()
{
	// we should check if there are any connections on this node, and delete
	// them before this object is gone.

	while (connectTO.size())
	{
		this->disconnect(connectTO[0]->sink->getID().c_str());	
	}
	
	while (connectFROM.size())
    {
        if (connectFROM[0]->source)
            connectFROM[0]->source->disconnect(this->getID().c_str());
	}
	
}

// *****************************************************************************


/*
void DSPNode::callbackUpdate()
{

	osg::Matrix myMatrix = osg::computeLocalToWorld(this->currentNodePath_);
	
	if (this->_globalMatrix != myMatrix)
	{
		this->_globalMatrix = myMatrix;
		osg::Vec3 myPos = myMatrix.getTrans();
		osg::Vec3 myRot = Vec3inDegrees(QuatToEuler(myMatrix.getRotate()));
		
		BROADCAST(this, "sffffff", "global6DOF", myPos.x(), myPos.y(), myPos.z(), myRot.x(), myRot.y(), myRot.z());
	}
}
*/

void DSPNode::callbackUpdate(osg::NodeVisitor* nv)
{
    GroupNode::callbackUpdate(nv);
}


// *****************************************************************************

SoundConnection *DSPNode::getConnection(DSPNode *snk)
{
	if (snk)
	{	
        std::vector<SoundConnection*>::iterator iter;
		for (iter = this->connectTO.begin(); iter != this->connectTO.end(); iter++)
		{
			if ((*iter)->sink == snk) return (*iter);
		}
	}
	
	return NULL;
}

SoundConnection *DSPNode::getConnection(const char *snk)
{
	return getConnection( dynamic_cast<DSPNode*>( sceneManager_->getNode(snk) ) );
}


// *****************************************************************************
void DSPNode::connect(DSPNode *snk)
{
	// check if this connection already exists:
	if (!this->getConnection(snk))
	{
		SoundConnection *conn = new SoundConnection(this->sceneManager_, this, snk);

		// add to the connection lists for each node:
		this->connectTO.push_back(conn);
		conn->sink->connectFROM.push_back(conn);
	}
	
	BROADCAST(this, "ss", "connect", snk->getID().c_str());
}

// *****************************************************************************
void DSPNode::connect(const char *snk)
{
	osg::ref_ptr<DSPNode> sinkNode = dynamic_cast<DSPNode*>( sceneManager_->getNode(snk) );
	if (sinkNode.valid()) this->connect(sinkNode.get());
}

void DSPNode::connectSource(const char *src)
{
	osg::ref_ptr<DSPNode> srcNode = dynamic_cast<DSPNode*>( sceneManager_->getNode(src) );
	if (srcNode.valid()) srcNode->connect(this);	
}


void DSPNode::disconnect(const char *snk)
{
	// check if this connection already exists:
	SoundConnection *conn = this->getConnection(snk);

	if (conn)
	{
		/*
		// if this is the last connection for this node, then disactivate it:
		if (this->connectTO.empty() && this->connectFROM.empty())
			this->setActive(0);
		
		// also check the sink:
		if (conn->sink->connectTO.empty() && conn->sink->connectFROM.empty())
			conn->sink->setActive(0);
		 */
		
		// remove it from the connectTO list:		
        std::vector<SoundConnection*>::iterator iter;
		for (iter = connectTO.begin(); iter != connectTO.end(); iter++)
		{
			if ((*iter) == conn)
			{
				connectTO.erase(iter);
				break;
			}
		}
		
		// and remove it from the sink's connectFROM list:
		for (iter = conn->sink->connectFROM.begin(); iter != conn->sink->connectFROM.end(); iter++)
		{
			if ((*iter) == conn)
			{
				conn->sink->connectFROM.erase(iter);
				break;
			}
		}
		
		
		// now delete the actual object:
		delete conn;

		BROADCAST(this, "ss", "disconnect", snk);
	} else { 
        std::cout << "oops. couldn't find connection: " << this->getID() << " -> " << snk << std::endl;
    }
}


// -----------------------------------------------------------------------------
// - SET METHODS:

void DSPNode::setActive (int i)
{
	active = (bool)i;
	BROADCAST(this, "si", "setActive", (int)active);
}

void DSPNode::setPlugin (const char *newPlugin)
{	
	plugin = std::string(newPlugin);
	BROADCAST(this, "ss", "setPlugin", plugin.c_str());
}

void DSPNode::setRolloff (const char *newvalue)
{
    // We store just the id instead of the whole table. If we want, we can always get the table
    // from the database (eg, for drawing).
    _rolloff = newvalue;
    drawDirectivity();
    BROADCAST(this, "ss", "setRolloff", _rolloff.c_str());
}

void DSPNode::setSpread (float newvalue)
{
    _spread = newvalue;
    drawDirectivity();
    BROADCAST(this, "sf", "setSpread", _spread);
}

void DSPNode::setLength (float newvalue)
{
    _length = newvalue;
    drawDirectivity();
    BROADCAST(this, "sf", "setLength", _length);
}

void DSPNode::setRadius (float newvalue)
{
    _radius = newvalue;
    if (_radius < 0) _radius = 0;
    drawRadius();
    BROADCAST(this, "sf", "setRadius", getRadius());
}

void DSPNode::setDirectivityFlag (float newFlag)
{
    directivityFlag = newFlag;
    drawDirectivity();
    BROADCAST(this, "sf", "setDirectivityFlag", directivityFlag);
}

void DSPNode::setLaserFlag (float newFlag)
{
    laserFlag = newFlag; // note continuous value is used for alpha
    drawLaser();
    BROADCAST(this, "sf", "setLaserFlag", laserFlag);
}

void DSPNode::setVUmeterFlag (float newFlag)
{
    VUmeterFlag = newFlag; // note continuous value is used for alpha
    drawVUmeter();
    BROADCAST(this, "sf", "setVUmeterFlag", VUmeterFlag);
}

void DSPNode::setRadiusFlag (float newFlag)
{
    radiusFlag = newFlag;
    drawRadius();
    BROADCAST(this, "sf", "setRadiusFlag", radiusFlag);
}

void DSPNode::setDebugColor(float r, float g, float b, float a)
{
    debugColor = osg::Vec4(r,g,b,a);
    drawDirectivity();
    drawRadius();
    BROADCAST(this, "sffff", "setDebugColor", r,g,b,a);
}

void DSPNode::setIntensity (float newvalue)
{
    currentSoundIntensity = newvalue;

    float r = currentSoundIntensity / 0.896909;
    if (r > 1.0) r=1.0;
    float g = (1.0 - currentSoundIntensity) / 0.103091;
    if (g > 1.0) g=1.0;
    currentSoundColor = osg::Vec3(r, g, 0.0);


    updateVUmeter();
    updateLaser();

    BROADCAST(this, "sf", "setIntensity", currentSoundIntensity);
}

// -----------------------------------------------------------------------------
// Update methods:

void DSPNode::updateVUmeter ()
{

    if (VUmeterTransform.valid())
    {

        for (unsigned i=0; i < VUmeterTransform->getNumChildren(); i++)
        {
            // update color of all drawables:
            osg::ref_ptr<osg::Geode> tmpGeode = dynamic_cast<osg::Geode*>(VUmeterTransform->getChild(i));
            if (tmpGeode.valid())
            {
                for (unsigned j=0; j<tmpGeode->getNumDrawables(); j++)
                {
                    osg::ShapeDrawable *s = dynamic_cast<osg::ShapeDrawable*>(tmpGeode->getDrawable(j));
                    if (s) s->setColor( osg::Vec4(currentSoundColor, VUmeterFlag) );
                }
            }
        }

        VUmeterTransform->setScale( osg::Vec3(1, 1, 1 + currentSoundIntensity) );

    }
}

void DSPNode::updateLaser()
{

    if (laserGeode.valid())
    {
        for (unsigned j=0; j<laserGeode->getNumDrawables(); j++)
        {
            osg::ShapeDrawable *s = dynamic_cast<osg::ShapeDrawable*>(laserGeode->getDrawable(j));
            if (s) s->setColor( osg::Vec4(currentSoundColor, laserFlag) );
        }
    }
}


// -----------------------------------------------------------------------------
// DRAW METHODS:

void DSPNode::drawVUmeter()
{
    pthread_mutex_lock(&sceneMutex);


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

        VUmeterTransform->setName(this->getID() + ".VUmeterTransform");
        this->getAttachmentNode()->addChild(VUmeterTransform.get());
   }

    pthread_mutex_unlock(&sceneMutex);
}


static t_float cardioid_to_cone_map[] = {180.0, 155.047, 147.605, 140.163, 116.907, 113.651, 106.674, 101.344, 97.1575, 93.6512, 90.8605, 89, 82.3043, 79.0484, 74.3973, 71.6065, 69.7461, 67.4205, 64.6298, 59.5135, 57.653, 55.7926, 54.8624, 53.467, 52.0717, 49.7461, 48.3507, 45, 44.9285, 42.9762, 41.8048, 41.4143, 40.6334, 39.8525, 38.6811, 38.2907, 37.1193, 36.3384, 35.9479, 35.167, 34.7766, 34.3861, 33.6052, 32.4338, 31.6529, 30.872, 30.4816, 30.4816, 29.7006, 29.3102, 28.5293, 28.1388, 27.7484, 26.9675, 26.9675, 26.577, 25.7961, 25.4056, 24.6247, 24.2343, 23.0629, 23.0629, 22.3043, 20.9089, 20.9089, 20.4438, 19.9787, 19.5136, 19.0485, 18.5834, 18.1182, 17.6531, 17.188, 16.7229, 16.7229, 16.2578, 16.2578, 15.7927, 15.7927, 15.3275, 15.3275, 15.0515, 14.4993, 14.3973, 13.9322, 13.4671, 13.002, 13.002, 12.7259, 12.4498, 12.4498, 12.4498, 12.1737, 11.6066, 11.6066, 11.6066, 11.6066, 11.3305, 11.3305, 10.6764, 10, 10.1242, 9.84816, 9.28103, 8.81591, 8.81591, 8.81591, 8.53984, 8.53984, 7.88568, 7.42056, 7.14449, 7.14449, 6.49033, 6.49033, 6.0066, 6.0066, 5.70254, 5.39849, 5.39849, 5.09443, 5.38037, 5.38037, 5.10429, 5.10429, 4.82822, 4.82822, 4.82822, 4.82822, 4.82822, 4.55215, 4.27607, 4.27607, 4.27607, 4, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 3.72393, 4, 3.72393, 3.72393, 3.44785, 3.17178, 3.17178, 3.17178, 3.17178, 3.17178, 3.17178, 3.17178, 2.89571, 2.89571, 2.89571, 2.89571, 2.61963, 2.61963, 2.61963, 2.34356, 2.34356, 2.34356, 2.34356, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.06748, 2.27607, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

void DSPNode::drawDirectivity()
{
    if (this->getAttachmentNode()->containsNode(directivityGeode.get()))
    {
        pthread_mutex_lock(&sceneMutex);
        this->getAttachmentNode()->removeChild(directivityGeode.get());
        directivityGeode = NULL;
        pthread_mutex_unlock(&sceneMutex);
    }

    if (directivityFlag > 0)
    {
        t_float val;
        t_float index = _spread * 16.6666;
        if ( (index < 0) || ((int)index >= 199) ) return;
        else val = cardioid_to_cone_map[(int)index];

        //printf("drawing directivity: _spread=%.3f  index=%.3f  val=%.3f  tan(val)=%.3f  cos(val)=%.3f  sin(val)=%.3f\n", _spread, index, val, tan(osg::DegreesToRadians(val)), cos(osg::DegreesToRadians(val)), sin(osg::DegreesToRadians(val)));


        // *** NEW METHOD (NOT WORKING YET):
        // directivityGeode = createWireframeRolloff( this->_rolloff, this->_spread, AS_DEBUG_SCALE, osg::Vec4(DEFAULT_DIRECTIVITY_COLOR,1.0) );

        // *** OLD METHOD:

        if (val < 90)
        {
            // Place a cone pointing along the +Y axis
            directivityGeode = createHollowCone( _length*AS_DEBUG_SCALE, AS_DEBUG_SCALE*_length*sin(osg::DegreesToRadians(val)), debugColor );
        } else if (val > 90 && val < 180) {
            // Place a cone pointing along the -Y axis
            directivityGeode = createHollowCone( -_length*AS_DEBUG_SCALE, AS_DEBUG_SCALE*_length*sin(osg::DegreesToRadians(val)), debugColor );
        } else {
            // Sphere
            directivityGeode= createHollowSphere(_length*AS_DEBUG_SCALE, debugColor );
        }

        // ***



        osg::StateSet *wireframeStateSet = new osg::StateSet();
        osg::PolygonMode *polymode = new osg::PolygonMode;
        polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
        wireframeStateSet->setAttributeAndModes(polymode,osg::StateAttribute::ON);
        wireframeStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
        wireframeStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

        directivityGeode->setStateSet(wireframeStateSet);

        directivityGeode->setName(this->getID() + ".directivityGeode");
        
        pthread_mutex_lock(&sceneMutex);
        this->getAttachmentNode()->addChild(directivityGeode.get());
        pthread_mutex_unlock(&sceneMutex);
    }

}


void DSPNode::drawRadius()
{
    pthread_mutex_lock(&sceneMutex);


    if (this->getAttachmentNode()->containsNode(radiusGeode.get()))
    {
        this->getAttachmentNode()->removeChild(radiusGeode.get());
        radiusGeode = NULL;
    }

    if (radiusFlag > 0)
    {
        radiusGeode= createHollowSphere(_radius, debugColor );
    
        osg::StateSet *wireframeStateSet = new osg::StateSet();
        osg::PolygonMode *polymode = new osg::PolygonMode;
        polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
        wireframeStateSet->setAttributeAndModes(polymode,osg::StateAttribute::ON);
        wireframeStateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
        wireframeStateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

        radiusGeode->setStateSet(wireframeStateSet);

        radiusGeode->setName(this->getID() + ".radiusGeode");
        this->getAttachmentNode()->addChild(radiusGeode.get());
    }

    pthread_mutex_unlock(&sceneMutex);
}

void DSPNode::drawLaser()
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
        laserGeode->setName(this->getID() + ".laserGeode");

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

// *****************************************************************************

std::vector<lo_message> DSPNode::getState() const
{
    // inherit state from base class
	std::vector<lo_message> ret = GroupNode::getState();
	
	lo_message msg;
	
	msg = lo_message_new();
	lo_message_add(msg, "si", "setActive",(int) this->active);
	ret.push_back(msg);

	msg = lo_message_new();
	lo_message_add(msg, "ss", "setPlugin", plugin.c_str());
	ret.push_back(msg);
	
	/*
	if (connectTO.size())
	{
		msg = lo_message_new();
		lo_message_add_string(msg, "connectedTo");
		for (int i=0; i<connectTO.size(); i++)
			lo_message_add_string(msg, (char*)connectTO[i]->sink->getID().c_str());
		ret.push_back(msg);
	}
	*/
	
	for (int i=0; i<connectTO.size(); i++)	
	{
		msg = lo_message_new();
		lo_message_add(msg, "ss", "connect", (char*)connectTO[i]->sink->getID().c_str());
		ret.push_back(msg);
	}
	
   msg = lo_message_new();
    lo_message_add(msg, "ss", "setRolloff", _rolloff.c_str());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setSpread", getSpread());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setLength", getLength());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setRadius", getRadius());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setDirectivityFlag", directivityFlag);
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sffff", "setDebugColor", debugColor.x(), debugColor.y(), debugColor.z(), debugColor.w());
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setLaserFlag", laserFlag);
    ret.push_back(msg);

    msg = lo_message_new();
    lo_message_add(msg, "sf", "setVUmeterFlag", VUmeterFlag);
    ret.push_back(msg);
    
    msg = lo_message_new();
    lo_message_add(msg, "sf", "setRadiusFlag", radiusFlag);
    ret.push_back(msg);

    // not doing this anymore (not supposed to be saved or refreshed)
    /*
    msg = lo_message_new();
    lo_message_add(msg, "sf", "setIntensity", currentSoundIntensity);
    ret.push_back(msg);
    */

    // have to re-send setContext AFTER setPlugin, so that loaded plugins will
    // have the up-to-date parameter
    msg = lo_message_new();
    lo_message_add(msg, "ss", "setContext", getContext());
    ret.push_back(msg);

	
	return ret;
}

} // end of namespace spin

