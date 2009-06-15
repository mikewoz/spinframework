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
