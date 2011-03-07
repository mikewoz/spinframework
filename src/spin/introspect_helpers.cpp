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

#include <osgIntrospection/Reflection>
#include <osgIntrospection/PropertyInfo>
#include <iostream>

// *****************************************************************************
// introspection helpers

bool introspect_type_order(const osgIntrospection::Type *v1, const osgIntrospection::Type *v2)
{
	using namespace osgIntrospection;
	
    if (!v1->isDefined()) return v2->isDefined();
    if (!v2->isDefined()) return false;
    return v1->getQualifiedName().compare(v2->getQualifiedName()) < 0;
}

void introspect_print_method(const osgIntrospection::MethodInfo &mi)
{
	using namespace osgIntrospection;
	
    std::cout << "\t    ";

    // display if the method is virtual
    if (mi.isVirtual())
        std::cout << "virtual ";

    // display the method's return type if defined
    if (mi.getReturnType().isDefined())
        std::cout << mi.getReturnType().getQualifiedName() << " ";
    else
        std::cout << "[UNDEFINED TYPE] ";

    // display the method's name
    std::cout << mi.getName() << "(";

    // display method's parameters
    const ParameterInfoList &params = mi.getParameters();
    for (ParameterInfoList::const_iterator k=params.begin(); k!=params.end(); ++k)
    {
        // get the ParameterInfo object that describes the 
        // current parameter
        const ParameterInfo &pi = **k;

        // display the parameter's modifier
        if (pi.isIn())
            std::cout << "IN";
        if (pi.isOut())
            std::cout << "OUT";
        if (pi.isIn() || pi.isOut())
            std::cout << " ";

        // display the parameter's type name
        if (pi.getParameterType().isDefined())
            std::cout << pi.getParameterType().getQualifiedName();

        // display the parameter's name if defined
        if (!pi.getName().empty())
            std::cout << " " << pi.getName();

        if ((k+1)!=params.end())
            std::cout << ", ";
    }
    std::cout << ")";
    if (mi.isConst())
        std::cout << " const";
    if (mi.isPureVirtual())
        std::cout << " = 0";
    std::cout << "\n";
}

void introspect_print_type(const osgIntrospection::Type &type)
{
	using namespace osgIntrospection;
	
    // ignore pointer types and undefined types
    if (!type.isDefined() || type.isPointer() || type.isReference())
    {
    	std::cout << "Error: Type is pointer or undefined." << std::endl;
    	return;
    }
    
    // print the type name
    std::cout << "  " << type.getQualifiedName() << "\n";

    // check whether the type is abstract
    if (type.isAbstract()) std::cout << "\t[abstract]\n";

    // check whether the type is atomic
    if (type.isAtomic()) std::cout << "\t[atomic]\n";

    // check whether the type is an enumeration. If yes, display
    // the list of enumeration labels
    if (type.isEnum()) 
    {
        std::cout << "\t[enum]\n";
        std::cout << "\tenumeration values:\n";
        const EnumLabelMap &emap = type.getEnumLabels();
        for (EnumLabelMap::const_iterator j=emap.begin(); j!=emap.end(); ++j)
        {
            std::cout << "\t\t" << j->second << " = " << j->first << "\n";
        }
    }

    // if the type has one or more base types, then display their
    // names
    if (type.getNumBaseTypes() > 0)
    {
        std::cout << "\tderived from: ";
        for (int j=0; j<type.getNumBaseTypes(); ++j)
        {
            const Type &base = type.getBaseType(j);
            if (base.isDefined())
                std::cout << base.getQualifiedName() << "    ";
            else
                std::cout << "[undefined type]    ";
        }
        std::cout << "\n";
    }

    // display a list of public methods defined for the current type
    const MethodInfoList &mil = type.getMethods();
    if (!mil.empty())
    {
        std::cout << "\t* public methods:\n";
        for (MethodInfoList::const_iterator j=mil.begin(); j!=mil.end(); ++j)
        {
            // get the MethodInfo object that describes the current
            // method
            const MethodInfo &mi = **j;

            introspect_print_method(mi);
        }
    }

    // display a list of protected methods defined for the current type
    const MethodInfoList &bmil = type.getMethods(Type::PROTECTED_FUNCTIONS);
    if (!bmil.empty())
    {
        std::cout << "\t* protected methods:\n";
        for (MethodInfoList::const_iterator j=bmil.begin(); j!=bmil.end(); ++j)
        {
            // get the MethodInfo object that describes the current
            // method
            const MethodInfo &mi = **j;

            introspect_print_method(mi);
        }
    }

    // display a list of properties defined for the current type
    const PropertyInfoList &pil = type.getProperties();
    if (!pil.empty())
    {
        std::cout << "\t* properties:\n";
        for (PropertyInfoList::const_iterator j=pil.begin(); j!=pil.end(); ++j)
        {
            // get the PropertyInfo object that describes the current
            // property
            const PropertyInfo &pi = **j;

            std::cout << "\t    ";

            std::cout << "{";
            std::cout << (pi.canGet()? "G": " ");
            std::cout << (pi.canSet()? "S": " ");
            std::cout << (pi.canCount()? "C": " ");
            std::cout << (pi.canAdd()? "A": " ");
            std::cout << "}  ";

            // display the property's name
            std::cout << pi.getName();

            // display the property's value type if defined
            std::cout << " (";
            if (pi.getPropertyType().isDefined())
                std::cout << pi.getPropertyType().getQualifiedName();
            else
                std::cout << "UNDEFINED TYPE";
            std::cout << ") ";

            // check whether the property is an array property
            if (pi.isArray())
            {
                std::cout << "  [ARRAY]";
            }

            // check whether the property is an indexed property
            if (pi.isIndexed())
            {
                std::cout << "  [INDEXED]\n\t\t       indices:\n";

                const ParameterInfoList &ind = pi.getIndexParameters();

                // print the list of indices
                int num = 1;
                for (ParameterInfoList::const_iterator k=ind.begin(); k!=ind.end(); ++k, ++num)
                {
                    std::cout << "\t\t           " << num << ") ";
                    const ParameterInfo &par = **k;
                    std::cout << par.getParameterType().getQualifiedName() << " " << par.getName();
                    std::cout << "\n";
                }
            }

            std::cout << "\n";
        }
    }
    std::cout << "\n" << std::string(75, '-') << "\n";
}

void introspect_print_all_types()
{
	using namespace osgIntrospection;
	
    // get the map of types that have been reflected
    const TypeMap &tm = Reflection::getTypes();
    
    // create a sortable list of types
    TypeList types(tm.size());
    TypeList::iterator j = types.begin();
    for (TypeMap::const_iterator i=tm.begin(); i!=tm.end(); ++i, ++j)
        *j = i->second;
    
    // sort the map
    std::sort(types.begin(), types.end(), &introspect_type_order);

    // iterate through the type map and display some
    // details for each type
    for (TypeList::const_iterator i=types.begin(); i!=types.end(); ++i)
    {
        introspect_print_type(**i);
    }
}
