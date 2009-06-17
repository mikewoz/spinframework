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
//    La SociŽtŽ des Arts Technologiques (http://www.sat.qc.ca)
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

#ifndef DEBUGVISITOR_H_
#define DEBUGVISITOR_H_

#include <osg/NodeVisitor>
#include <osg/Notify>
#include <osg/PositionAttitudeTransform>
#include <osg/Geode>

#include <osgIntrospection/Type>
#include <osgIntrospection/Value>


#include "asUtil.h"
#include "asReferenced.h"
#include "asBasicNode.h"
#include "asSoundNode.h"
#include "asSoundSpace.h"
#include "asShape.h"

bool introspect_type_order(const osgIntrospection::Type *v1, const osgIntrospection::Type *v2);
void introspect_print_method(const osgIntrospection::MethodInfo &mi);
void introspect_print_type(const osgIntrospection::Type &type);
void introspect_print_all_types();

/**
 * \brief An OSG NodeVisitor class that pretty prints the scene graph
 */
class DebugVisitor : public osg::NodeVisitor
{
	public:
		
		DebugVisitor() : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN) {}

		virtual void apply(osg::Node &node)
		{
			osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "NODE:  " << node.getName() << std::endl;
			traverse(node);
		}
		virtual void apply(osg::PositionAttitudeTransform &node)
		{
			osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "PAT:   " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			traverse(node);
		}
		virtual void apply(osg::Geode &node)
		{
			osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "GEODE: " << node.getName() << std::endl;
			traverse(node);
		}
		virtual void apply(osg::Group &node)
		{
			if (dynamic_cast<asBasicNode*>(&node)) {
				osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "BASICNODE: " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			} else if (dynamic_cast<asShape*>(&node)) {
				osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "SHAPE: " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			} else if (dynamic_cast<asReferenced*>(&node)) {
				osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "ASREFERENCED: " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			} else {
				osg::notify(osg::NOTICE) << leadingSpaces(getNodePath().size()) << "GROUP: " << node.getName() << "  (" << node.getNumChildren () << " children)" << std::endl;
			}
			traverse(node);
		}
};



#endif /*DEBUGVISITOR_H_*/
