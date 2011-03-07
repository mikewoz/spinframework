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

#ifndef __AnimationNode_H
#define __AnimationNode_H

#include "GroupNode.h"

#include <osg/AnimationPath>

// forward declarations
class SceneManager;

/**
 * \brief Node for encapsulating 3D animation
 */
class AnimationNode : public GroupNode
{

public:

    AnimationNode(SceneManager *sceneManager, char *initID);
    virtual ~AnimationNode();
    
    enum LoopMode { SWING, LOOP, NO_LOOPING };

    
    
    virtual void callbackUpdate();
    
    /**
     * Performs the actual update of translation, orientation, and scale given
     * a timestamp into the animation
     */
    bool doUpdate(double timestamp);
    
    /**
     * A client interface that allows for setting the animation time using a
     * normalized index in the range [0,1]
     */
    void setIndex (float index);
    
    
    /**
     * \brief Set the update rate (in Hz).
     * 
     * The animation will send setTranslation, setOrientation, and setScale
     * events at this rate (assuming there is a change). Values will be
     * interpolated in between control points.
     */
    void setUpdateRate (float hz);
    float getUpdateRate() const { return _updateRate; }
    
    
    
    void setPlay (int p);
    int getPlay() const { return (int) _play; }
        
    /**
     * Turns on/off automatic recording. This implies that whenever the node
     * gets an update of translation or orientation, a new position will be
     * saved. Each position will be added with a timestamp relative to the start
     * of the recording.
     */
    void setRecord (int r);
    int getRecord() const { return (int) _record; }
    
    /**
     * Sets the loop mode for the animation. eg, SWING, LOOP, NO_LOOPING.
     */
    void setLoopMode (LoopMode mode);
    int getLoopMode() const { return (int) _animationPath->getLoopMode(); }
    
    /**
     * Override setTranslation so we can store updates in record mode
     */
    virtual void setTranslation (float x, float y, float z);

    /**
     * Override setOrientation so we can store updates in record mode
     */
    virtual void setOrientation (float pitch, float roll, float yaw);

    /**
     * Override setScale so we can store updates in record mode
     */
    virtual void setScale (float x, float y, float z);

    
    /**
     * Stores the node's current position/rotation/scale as a control point in
     * the animation sequence
     */
    void storeCurrentPosition ();
    
    
    /**
     * Adds the current position/rotation/scale into the animation sequence at
     * the specified time offset, overriding any previous position at that time.
     */
    void storeCurrentPosition (double timestamp);
    
    /**
     * Explicitely adds a control point into the animation sequence
     */
    void controlPoint (double timestamp, float x, float y, float z, float rotX, float rotY, float rotZ, float rotW, float scaleX, float scaleY, float scaleZ);
    
    /**
     * Clears the current animation sequence
     */
    void clear() { _animationPath->clear(); }

    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

protected:
    
    bool _play, _record;
    osg::Timer_t _startTime, _lastTick;
    float _updateRate;
    
    osg::ref_ptr<osg::AnimationPath> _animationPath;
};

#endif
