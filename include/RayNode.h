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

#ifndef RayNode_H_
#define RayNode_H_

#include <osg/Geode>

#include "GroupNode.h"
namespace spin
{

/**
 * \brief Describes a ray (actually a line segment) in space.
 *
 * The RayNode "points" in the direction specified by it's current orientation,
 * and is visible up to a certain length.
 *
 * RayNode is used by PointerNode to report which nodes are being pointed at.
 */
class RayNode : public GroupNode
{

public:

    RayNode(SceneManager *sceneManager, char *initID);
    virtual ~RayNode();

    void setVisible        (int visibilityFlag);
    void setLength        (float length);
    void setThickness    (float thickness);
    void setColor        (float red, float green, float blue, float alpha);

    int getVisible() const { return (int) this->visible; }
    float getLength() const { return this->length; };
    float getThickness() const { return this->thickness; };
    osg::Vec4 getColor() const { return this->color;  };


    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

    /**
     * We must include a stateDump() method that simply invokes the base class
     * method. Simple C++ inheritance is not enough, because osg::Introspection
     * won't see it.
     */
    //virtual void stateDump() { ReferencedNode::stateDump(); };


private:

    void drawRay();

    osg::ref_ptr<osg::Geode> rayGeode;

    bool visible;
    float length;
    float thickness;
    osg::Vec4 color;

};

} // end of namespace spin

#endif
