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

    GeometryNode(SceneManager *sceneManager, const char* initID);
    virtual ~GeometryNode();
    
    virtual void callbackUpdate(osg::NodeVisitor* nv);
    virtual void updateStateSet();
    
    /**
     * Sets the number of vertices that this geometry is supposed to have.
     * 
     * This should be a multiple of 3 if you want to draw using GL_TRIANGLES or
     * amultiple of 4 if you want to draw using GL_QUADS. Anything else will
     * draw using GL_LINES.
     *
     * If you grow the number of vertices, all new vertices will be created with
     * default values: position of (0,0,0), full white color, and texcoords of
     * (0,0).
     */ 
    void setNumVertices(int i);
    
    /**
     * Update the position of one vertex in the geometry, using an index.
     */
    void setVertex(int index, float x, float y, float z);

    /**
     * Update the color of one vertex in the geometry, using an index.
     */
    void setColor(int index, float red, float green, float blue, float alpha);

    /**
     * Update the texture coord of one vertex in the geometry, using an index.
     */
    void setTexCoord(int index, float x, float y);
    
    /**
     * Specify whether both sides or only one side of the shape is rendered. ie,
     * whether the backface is culled or not.
     */
    void setSingleSided (int singleSided);

	/**
	 * Returns whether both sides of the shape are being rendered or if the 
	 * backface is being culled.
	 */
	
    int getSingleSided() const { return (int)singleSided_; }


    virtual std::vector<lo_message> getState() const;

private:
    
    osg::ref_ptr<osg::Geode> geode_;
    osg::ref_ptr<osg::Geometry> geometry_;
    unsigned int numVertices_;
    bool singleSided_;
    
    bool updateFlag_;
};

} // end of namespace spin

#endif
