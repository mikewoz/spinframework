/*
* This file is part of the SPIN Framework.
*
* Copyright (c) 2009 Mike Wozniewski
* Copyright (c) 2009 Zack Settel
* Copyright (c) 2011 Alexandre Quessy
*
* SPIN Framework is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* SPIN Framework is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
*/

#include <cppintrospection/Attributes>
#include <cppintrospection/Exceptions>
#include <cppintrospection/ExtendedTypeInfo>
#include <cppintrospection/MethodInfo>
#include <cppintrospection/PropertyInfo>
#include <cppintrospection/Reflection>
#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Type>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/Value>
#include <cppintrospection/variant_cast>
#include <iostream>
#include "introspection.h"

namespace spin
{
namespace editor
{
std::vector<std::string> listSpinNodeTypes()
{
    namespace intro = cppintrospection;
    std::vector<std::string> ret;
    try
    {
        const intro::Type &ReferencedNodeType = cppintrospection::Reflection::getType("ReferencedNode");
        const intro::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
        cppintrospection::TypeMap::const_iterator it;
        for (it = allTypes.begin(); it != allTypes.end(); ++it)
        {
            if (((*it).second)->isDefined())
            {
                if ( ((*it).second)->isSubclassOf(ReferencedNodeType) )
                {
                    std::string theType = ((*it).second)->getName();
                    ret.push_back(theType);
                }
            }
        }
    }
    catch (const intro::Exception &ex)
    {
        std::cerr << __FUNCTION__ << ": Could not list node types:\n" << ex.what() << std::endl;
    }
    return ret;
}
} // end of namespace editor
} // end of namespace spin

