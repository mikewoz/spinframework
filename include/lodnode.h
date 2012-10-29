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

#ifndef __LODNode_H
#define __LODNode_H

#include "groupnode.h"

#include <osg/LOD>

namespace spin
{

// forward declarations
class SceneManager;

/**
 * The LODNode encapsulates a level-of-detail technique that maintians levels of
 * detail or complexity for a given object, and provides certain hints to
 * automatically choose the appropriate level of the object, for instance,
 * according to the distance from the viewer. It decreases the complexity of the
 * object's representation in the 3D world, and often has an unnoticeable
 * quality loss on a distant object's appearance.
 *
 * To use this node, just attach children to this node and specify their visible
 * range using the setRange method
 *
 */
class LODNode : public GroupNode
{

public:

    LODNode(SceneManager *sceneManager, const char* initID);
    virtual ~LODNode();
        
    virtual void updateNodePath();
        
    /**
     * Set the visible range of a child node
     */
    void setRange (const char* childID, float min, float max);

    /**
     * For each subclass of ReferencedNode, we override the getState() method to
     * fill the vector with the correct set of methods for this particular node
     */
    virtual std::vector<lo_message> getState() const;

protected:
    
    osg::ref_ptr<osg::LOD> LOD_;
};

} // end of namespace spin

#endif
