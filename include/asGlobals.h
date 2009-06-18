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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------


#ifndef __ASGLOBALS_H
#define __ASGLOBALS_H

#include "pdUtil.h"


#define OSG_NUM_LIGHTS 8

// set the following DEBUG flag to 1 for more verbose print statements:
#define DEBUG 0

#define AS_UNIT_SCALE  1.0f // 1m
#define AS_DEBUG_SCALE 4.0f // size of debug views (radiation/sensitivity/etc)

#define GENERIC_SHAPE_RESOLUTION 10.0f

#define AS_NAMESIZE 100 // must be same as NETMAIL_NAMESIZE from netmail.h !!! TODO

// define a couple useful vectors that will be referred to many times:
// I think this is defined in OSG somewhere
/*
#define X_AXIS osg::Vec3(1,0,0)
#define Y_AXIS osg::Vec3(0,1,0)
#define Z_AXIS osg::Vec3(0,0,1)
*/

#define NULL_SYMBOL gensym("NULL")
#define WORLD_SYMBOL gensym("world")


// *****************************************************************************
// NODE MASKS:

// We define several nodemasks that describe how scene graph traversal should be
// done in different modes of operation. A nodeVisitor can be created to visit
// any subset of these nodes
// GEOMETRIC nodes are those that always have physical presence in the scene,
// such as models or shapes. These items will be processed for collisions,
// intersections, etc.
#define GEOMETRIC_NODE_MASK 0x00000001

// DEBUGVIEW nodes are those that should be visible in a viewing window, but do
// not count when doing collision detection or intersection testing.
#define DEBUGVIEW_NODE_MASK 0x00000010

// STATSDATA nodes are those which do not need a visual representation, and so
// they are culled in camera traversals. These nodes are typically used to hold
// information for interaction.
#define STATSDATA_NODE_MASK 0x00000100




#endif
