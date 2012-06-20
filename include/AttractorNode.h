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


#ifndef __AttractorNode_H
#define __AttractorNode_H

#include "GroupNode.h"

namespace spin
{

class SceneManager;

/**
 * \brief A node to attract (or repulse) other nodes
 */
class AttractorNode : public GroupNode
{

public:

    AttractorNode(SceneManager *sceneManager, char *initID);
    virtual ~AttractorNode();

    /**
     * There are two types of attractorMode. An EXTRINSIC attractor will simply
     * move or translate the target by a small amount each time increment (this
     * is the default behaviour). An INTRISIC attractor actually changes the
     * velocity of the target.
     */
    enum attractorMode { 
    					EXTRINSIC, /*!< translates the attracted target by a
									small amount each time increment */
    					INTRINSIC  /*!< changes the velocity of the attracted
									target */
    					};
    

    virtual void callbackUpdate(osg::NodeVisitor* nv);

    /**
     * setDistanceDecay specifies how the attractive force decays as a function
     * of distance. A decay of 1.0 is a linear decay; greater than 1 is
     * exponential (slow at start, faster as you approach the attractor); less
     * than 1 is logarithmic (fast at start, slower as you approach); and zero
     * implies a constant decay (independent of distance).
     */
    void setDistanceDecay(float decay);
    
    /**
    * @return a float that indicates the decay of the node's attractive force as
    * a function of distance from the node
    */
    
    float getDistanceDecay() const { return distanceDecay_; }

    /**
     * setAngularDecay specifies how the attractive force decays as a function
     * of incidence to the attractor's current orientation. ie, whether the
     * force falls off linearly (1.0), exponentially (>1), logarithmically (<1),
     * or not at all (0.0) if the attractor is not pointing directly at the
     * target.
     */
    void setAngularDecay(float decay);
    
    /**
    * @return a float which indicates how the attractive force of the node
    * decays as a function of incidence to the attractor's current orientation
    * (i.e. at what angle objects are from the attractive node)
    */
    
    float getAngularDecay() const { return angularDecay_; }

    /**
     * Change the mode (see attractorMode enum).
     */
    void setAttractorMode(attractorMode m);
    
    /**
    * @return an int indicating the attractor mode currently set for the node,
    * see the attractorMode enum for types
    */
    
    int getAttractorMode() const { return (int)mode_; }

    /**
     * Set the attractive force (negative force for repulsion)
     */
    void setForce(float force);
    
    /**
    * @return a float which indicates the attractive force of the node
    */
    
    float getForce() const { return force_; }

    /**
     * Add a target node to the list
     */
    void addTarget (const char *targetID);

    /**
     * Remove a target from the list
     */
    void removeTarget (const char *targetID);


    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

protected:
    
    float force_;
    attractorMode mode_;
    float distanceDecay_;
    float angularDecay_;
    
    typedef std::vector< osg::observer_ptr<GroupNode> > targetVector;

    targetVector targets_;

private:
    osg::Timer_t lastTick_;
    osg::Timer_t lastUpdate_;
    
};

} // end of namespace spin

#endif
