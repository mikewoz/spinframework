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

#ifndef __DSPNode_H
#define __DSPNode_H

#include <vector>
#include <lo/lo_types.h>
#include "groupnode.h"
#include "osgutil.h"


namespace osg {
    class Geode;
    class PositionAttitudeTransform;
}

namespace spin
{

class SceneManager;

/**
 * \brief The base class for 3D audio nodes
 * 
 * This class allows for spatially localized sounds in the scene. However, this
 * should not be instantiated directly, rather, one needs to create a subclass
 * for each audio representation.
 * 
 * The main feature of this class is that it maintains the connection logic that
 * describes which node sends sound to another. The connect() and disconnect()
 * methods create the interface for managing connections for all derived classes
 */
class DSPNode : public GroupNode
{

public:
        
        DSPNode (SceneManager *sceneManager, const char* initID);
        virtual ~DSPNode();
        
        virtual void debug();
        
        virtual void callbackUpdate(osg::NodeVisitor* nv);
        bool dumpGlobals(bool forced=false);
        
        /**
         * Set the media for the sound node using a URI pattern.
         * 
         * Examples:
         * file://soundfilename.wav
         * file:///home/johndoe/soundfilename.wav
         * http://www.server.com/soundfile.wav
         * adc://1:1
         * adc://1
         * content://media/external/audio/media/710
         * mms://some_media_stream
         * rtsp://127.0.0.1:12311
         * pd_plugin://audio_plugin_patch.pd
         */
        virtual void setURI (const char *uri);

		/**
	 	* Returns the currently-set URI associated with the sound node.
	 	*/ 
	
        const char* getURI() const { return uri_.c_str(); }
        
        // for sending messages to the connections of this (source) node:
        // virtual void connectionMsg (char *snkName, char *method, float
        // value);
        
        /**
         * Activate or deactivate the DSP processing
         */
        virtual void setActive (int i);

		/**
	 	* Returns whether the DSP processing is active or inactive.
	 	*/
		
        int getActive() const { return (int)active; }
        
        /**
         * For each subclass of ReferencedNode, we override the getState()
         * method to fill the vector with the correct set of methods for this
         * particular node
         */
        virtual std::vector<lo_message> getState() const;


        // SET methods:
        virtual void setRolloff (const char *newvalue);
        virtual void setSpread (float newvalue);
        virtual void setLength (float newvalue);
        virtual void setRadius (float newvalue);

        void setDebugColor(float r, float g, float b, float a);

        void setVUmeterFlag (float newFlag);
        void setDirectivityFlag (float newFlag);
        void setLaserFlag (float newFlag);
        void setRadiusFlag (float newFlag);

        //
        void setIntensity(float newvalue);
        
        // GET methods:
        const char* getRolloff() const { return _rolloff.c_str(); }
        float getSpread() const { return _spread; }
        float getLength() const { return _length; }
        float getRadius() const { return _radius; }

        osg::Vec4 getDebugColor() const { return debugColor; }

        float getVUmeterFlag() const { return VUmeterFlag; }
        float getDirectivityFlag() const { return directivityFlag; }
        float getLaserFlag() const { return laserFlag; }
        float getRadiusFlag() const { return radiusFlag; }

        void updateVUmeter();
        void updateLaser();

        
        // DRAW methods:
        void drawVUmeter();
        void drawDirectivity();
        void drawLaser();
        void drawRadius();
        
private:
    
        bool active;
        
        std::string uri_;

        // TODO: move all graphical items (VUMeter, directivity, laser) into
        // separate node class, and attach as a subgraph, only when needed.

        float currentSoundIntensity;
        osg::Vec3 currentSoundColor;


        std::string _rolloff; // we keep a reference name for the rolloff
							  //	(directivity) table
        float _spread; // propagation cone for source
        float _length; // the length of the laser and cone
        float _radius; // the radius 

        // TODO: add toggle for PRE/POST

        // flags with continuous values (can be used for alpha, etc):
        float VUmeterFlag;
        float directivityFlag;
        float laserFlag;
        float radiusFlag;


        // The following methods and parameters are for drawing aspects of the
        // soundNode using OSG (eg, directivity pattern, laser, etc)

        // directivity patterns:
        osg::ref_ptr<osg::Geode> directivityGeode;
        osg::Vec4 debugColor;

        osg::ref_ptr<osg::Geode> radiusGeode;
        

        // laser beams:
        osg::ref_ptr<osg::Geode> laserGeode;

        // VU meter:
        osg::ref_ptr<osg::PositionAttitudeTransform> VUmeterTransform;



};

} // end of namespace spin

#endif
