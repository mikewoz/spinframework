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

#ifndef __Fog_H
#define __Fog_H

#include <osg/Vec4>
#include "referencedstateset.h"

namespace osg
{
    class Fog;
}

namespace spin
{

class SceneManager;

/**
 * \brief Basic GL Fog node.
 */
class Fog : public ReferencedStateSet
{
public:

    Fog(SceneManager *sceneManager, const char *initID);
    /**
     * FIXME: should be virtual 
     */
    ~Fog();

    /**
     * need to implement abstract method... ?!
     */
    const char *getPath() const { return ""; }
    
    /**
     * Set fog density (good values are around 0.001 - 0.1)
     */
    void setFogDensity (float density);
    /**
     * Get fog density (good values are around 0.001 - 0.1)
     */
    float getFogDensity() const;

    /**
     * Set fog color in RGBA value.
     */
    void setFogColor (float r, float g, float b, float a);
    /**
     * Returns fog color
     */
    osg::Vec4 getFogColor() const;

    /**
     * must reimplement
     */
    virtual std::vector<lo_message> getState() const;
    
private:
    osg::ref_ptr<osg::Fog> fog_;
};

} // end of namespace spin

#endif
