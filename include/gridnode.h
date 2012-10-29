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

#ifndef GridNode_H_
#define GridNode_H_

#include <osg/Vec4>
#include "referencednode.h"

namespace osg {
    class Geode;
}

namespace spin
{

/**
 * \brief Draws a tiled grid, which is useful for debugging and measurements
 */
class GridNode : public ReferencedNode
{
public:
    GridNode(SceneManager *sceneManager, const char* initID);
    virtual ~GridNode();

    /**
     * The grid size is measured in meters, and defines how far out from the
     * center the grid is drawn (ie, half width).
     */

    void setSize        (int _size);

    /**
     * Sets the grid's line color in RGBA value.
     */
    void setColor        (float red, float green, float blue, float alpha);

    /**
     * Returns the size of the grid (in meters from the center).
     */

    int getSize() const { return (int) this->_size; }

    /**
     * Returns the color of the grid's lines in a Vector4 of RGBA values.
     */

    osg::Vec4 getColor() const { return this->_color;  };


    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;


private:

    void drawGrid();

    osg::ref_ptr<osg::Geode> gridGeode;

    int _size;
    float _thickness;
    osg::Vec4 _color;
};


} // end of namespace spin

#endif
