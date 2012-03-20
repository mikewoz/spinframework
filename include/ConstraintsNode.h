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


#ifndef __ConstraintsNode_H
#define __ConstraintsNode_H


#include "GroupNode.h"

#include <vector>

namespace spin
{

/**
 * \brief A node with constrained motion
 * 
 * This node operates much like GroupNode, allowing children to be attached, but
 * motion is constrained in a certain way.
 *
 * Foremost, there is one BASIC form of constraint where motion of the node is
 * bound within a cubic volume.
 *
 * Additionally, there are several ConstraintModes, where properties of a target
 * node are used to limit movement of this node. For example, the node will only
 * move along the surface of the target, or bounce off.
 *
 * One needs to specify the target node for some of these special modes to work.
 */
class ConstraintsNode : public GroupNode
{

public:
    

    ConstraintsNode(SceneManager *sceneManager, char *initID);
    virtual ~ConstraintsNode();
    
    /**
     * The following constraints are available:
     *
     * BASIC:
     * The node is just constrained within a cubic volume. Important note: the
     * BASIC constraint is always maintained, even if another mode is chosen.
     *
     * DROP:
     * The node sits on the surface of the target subgraph (ie, follows local
     * Z-axis down until it finds an intersection with the target surface)
     *
     * COLLIDE:
     * A form of collision detection, where the node is blocked (and slides
     * along) the target's surface. Note: sliding only works when the target is
     * locally convex. Concavities are not checked, and may position the node
     * on the other side of the target's surface. Use STICK instead for
     * non-convex geometries.
     *
     * BOUNCE:
     * A form of collision detection, where the node reflects off the parent's
     * surface, and travels in the reflected direction (ie, the orientation of
     * the node is changed). Note: this only works when node is moved using the
     * "translate" command.
     *
     * STICK:
     * Equivalent to COLLIDE, but without sliding.
     *
     * COLLIDE_THRU:
     * A fake constraint, which can be used to simply report when a collision 
     * occurs. The event reported is the same as that for COLLIDE.
     */
    enum ConstraintMode {   BASIC,
                            DROP,
                            COLLIDE,
                            BOUNCE,
                            STICK,
                            COLLIDE_THRU
                        };

        
    virtual void callbackUpdate();
    
    void setTarget(const char *id);
    const char *getTarget() const { return target_->s_name; }
    
    void setConstraintMode(ConstraintMode m);
    int getConstraintMode() const { return (int)constraintMode_; };
    
    void setCubeSize(float xScale, float yScale, float zScale);
    void setCubeOffset(float x, float y, float z);

    osg::Vec3 getCubeSize() const { return cubeSize_; }
    osg::Vec3 getCubeOffset() const { return cubeOffset_; }

    virtual void setTranslation (float x, float y, float z);
    virtual void translate (float x, float y, float z);
    virtual void move (float x, float y, float z);
    
    /**
     * A pseudo-recursive function that checks if a translation results in one
     * or more intersections with the target node. If no intersection, this
     * method defaults to just a setTranslation call. Otherwise, it will do a
     * setTranslation for the collision point, and call itself again until there
     * are no collisions left.
     */
    void applyConstrainedTranslation(osg::Vec3 v);
    
    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

    
private:
    
    enum ConstraintMode constraintMode_;
    
    t_symbol* target_;

    osg::Vec3 cubeSize_;
    osg::Vec3 cubeOffset_;

    osg::ref_ptr< osg::Drawable > lastDrawable;
    int lastPrimitiveIndex;
    int recursionCounter;
};


} // end of namespace spin

    
#endif
