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

#ifndef __GeometryNode_H
#define __GeometryNode_H

#include "GroupNode.h"

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Texture2D>
#include <osgUtil/Optimizer>

namespace spin
{

/**
 * \brief Node to draw arbitrary polygon
 *
 * Allow a low-level specification of a mesh by providing arrays of vertices,
 * colors, and texture coordinates.
 */
class GeometryNode : public GroupNode
{
public:

    GeometryNode(SceneManager *sceneManager, char *initID);
    virtual ~GeometryNode();
    
    virtual void callbackUpdate(osg::NodeVisitor* nv);
    virtual void updateStateSet();
    
    void setNumVertices(int i);
    void setVertex(int index, float x, float y, float z);
    void setColor(int index, float red, float green, float blue, float alpha);
    void setTexCoord(int index, float x, float y);

    virtual std::vector<lo_message> getState() const;

private:
    
    osg::ref_ptr<osg::Geode> geode_;
    osg::ref_ptr<osg::Geometry> geometry_;
    unsigned int numVertices_;
    
    bool updateFlag_;
};

} // end of namespace spin

#endif
