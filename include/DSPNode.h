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
#include "GroupNode.h"
#include "osgUtil.h"


namespace osg {
    class Geode;
    class PositionAttitudeTransform;
}

namespace spin
{

class SoundConnection;
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
        
        virtual void callbackUpdate(osg::NodeVisitor* nv);
        
        /**
         * Returns the SoundConnection object that exists between the current
         * node and the sink or null if none is found.
         * @param snk a pointer to the sink node
         */

        SoundConnection *getConnection(DSPNode *snk);

        /**
         * Returns the SoundConnection object that exists between the current
         * node and the sink or null if none is found.
         * @param snk the name of the sink node to search for
         */

        SoundConnection *getConnection(const char *snk);

        /**
         *
         */

        void connect(DSPNode *snk);
        void connect(const char *snk);

        /**
         * Sets the current node as a sink to the specified node.
         */

        void connectSource(const char *src);

        void disconnect(const char *snk);
        void setActive (int i);
        void setPlugin (const char *filename);
        
       /**	for sending messages to the connections of this (source) node:
        *	virtual void connectionMsg (char *snkName, char *method, float
        * 	value);
		*/
        
        int getActive() const { return (int)active; }
        const char* getPlugin() const { return plugin.c_str(); }
        
        /**
         * We maintain 2 lists of all SoundConnection for this node (it is
         * redundant, but useful to have both forward and backward connection
         * pointers).
         */
        std::vector<SoundConnection*> connectTO;
        /**
         * We maintian 2 lists of all SoundConnection for this node (it is
         * redundant, but useful to have both forward and backward connection
         * pointers).
         */
        std::vector<SoundConnection*> connectFROM;
        
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

        void setDirectivityColor(float r, float g, float b, float a);

        void setVUmeterFlag (float newFlag);
        void setDirectivityFlag (float newFlag);
        void setLaserFlag (float newFlag);

        //
        void setIntensity(float newvalue);
        
        // GET methods:
        const char* getRolloff() const { return _rolloff.c_str(); }
        float getSpread() const { return _spread; }
        float getLength() const { return _length; }
        float getRadius() const { return _radius; }

        osg::Vec4 getDirectivityColor() const { return directivityColor; }

        float getVUmeterFlag() const { return VUmeterFlag; }
        float getDirectivityFlag() const { return directivityFlag; }
        float getLaserFlag() const { return laserFlag; }

        void updateVUmeter();
        void updateLaser();

        
        // DRAW methods:
        void drawVUmeter();
        void drawDirectivity();
        void drawLaser();
        
private:
    
        bool active;
        
        /**
         * dsp name (this is the name of a pd abstraction that handles the dsp):
         */
        std::string plugin;
        
        /**
         * This node should always broadcast global position and orientation 
         * so that any audio spatializer software listening to messages can use
         * the data without needing to understand and maintain a scene graph.
         */
        //osg::Matrix _globalMatrix;

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


        // The following methods and parameters are for drawing aspects of the
        // soundNode using OSG (eg, directivity pattern, laser, etc)

        // directivity patterns:
        osg::ref_ptr<osg::Geode> directivityGeode;
        osg::Vec4 directivityColor;

        // laser beams:
        osg::ref_ptr<osg::Geode> laserGeode;

        // VU meter:
        osg::ref_ptr<osg::PositionAttitudeTransform> VUmeterTransform;



};

} // end of namespace spin

#endif
