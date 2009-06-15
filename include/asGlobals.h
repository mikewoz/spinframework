// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================


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
