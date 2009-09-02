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

#ifndef __spinUtil_H
#define __spinUtil_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>




// *****************************************************************************
// GLOBAL DEFINITIONS:


#define OSG_NUM_LIGHTS 8

// set the following DEBUG flag to 1 for more verbose print statements:
#define DEBUG 0

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




// *****************************************************************************
// networking functions

std::string getHostname();
std::string getMyIPaddress();
std::string getMyBroadcastAddress();
bool isMulticastAddress(std::string s);
bool isBroadcastAddress(std::string s);

// *****************************************************************************
// string handling functions

template <class T> bool fromString(T& t, const std::string& s)
{
	std::istringstream iss(s);
	return !(iss >> t).fail();
}

/*
template <typename T> bool fromString(T &aValue, const std::string &aStr)
{
	std::stringstream ss(aStr);
	return ss >> aValue;
}
*/

std::string stringify(float x);

std::string leadingSpaces(int n);


std::vector<std::string> tokenize(const std::string& str, const std::string& delimiters = " ");
std::vector<float> floatsFromString (std::string theString);

// *****************************************************************************
// file helpers

bool fileExists(const std::string& fileName);
std::string getRelativePath(std::string path);
std::string getAbsolutePath(std::string path);



// *****************************************************************************
// gensym stuff (from m_pd.h)

#include <stddef.h>     // just for size_t -- how lame!

class ReferencedNode;

#define EXTERN extern
#define EXTERN_STRUCT extern struct

typedef struct _symbol
{
    char *s_name;
    ReferencedNode *s_thing;
    struct _symbol *s_next;
} t_symbol;


typedef float t_float;
typedef float t_floatarg;  

#define t_class struct _class
typedef t_class *t_pd; 

EXTERN t_symbol *gensym(const char *s);

EXTERN void *getbytes(size_t nbytes);
EXTERN void *copybytes(void *src, size_t nbytes);
EXTERN void freebytes(void *x, size_t nbytes);



// *****************************************************************************
// introspection helpers
#include <osgIntrospection/Type>
#include <osgIntrospection/MethodInfo>

bool introspect_type_order(const osgIntrospection::Type *v1, const osgIntrospection::Type *v2);
void introspect_print_method(const osgIntrospection::MethodInfo &mi);
void introspect_print_type(const osgIntrospection::Type &type);
void introspect_print_all_types();






#endif
