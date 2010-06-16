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


#include <boost/utility.hpp>
#include <boost/python.hpp>
#include <boost/python/raw_function.hpp>
#include <boost/python/call_method.hpp>
#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/object/function_object.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/dict.hpp>

#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <string.h>

#include <iostream>
#include <sstream>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include <osgIntrospection/Reflection>
#include <osgIntrospection/Type>
#include <osgIntrospection/Value>
#include <osgIntrospection/variant_cast>
#include <osgIntrospection/Exceptions>
#include <osgIntrospection/MethodInfo>
#include <osgIntrospection/PropertyInfo>

#include <osgIntrospection/ReflectionMacros>
#include <osgIntrospection/TypedMethodInfo>
#include <osgIntrospection/StaticMethodInfo>
#include <osgIntrospection/Attributes>

#include <osgIntrospection/ExtendedTypeInfo>
#include <osgIntrospection/variant_cast>


#include "spinApp.h"
#include "SceneManager.h"

using namespace boost::python;

/******************************************************************************/

class ValueWrapper {

public:
    ValueWrapper();
    ValueWrapper(osgIntrospection::Value& v);
    ValueWrapper(int v);
    ValueWrapper(float v);
    ValueWrapper(std::string& v);
    ValueWrapper(const std::string& v);
    ~ValueWrapper();


    int getInt();
    float getFloat();
    std::string getString();
    boost::python::tuple getVector();

protected:
    osgIntrospection::Value _value;
};
/******************************************************************************/
ValueWrapper::ValueWrapper() {
    _value = osgIntrospection::Value(0);
}
/******************************************************************************/
ValueWrapper::ValueWrapper(osgIntrospection::Value& v) {
    _value = v;
}

/******************************************************************************/
ValueWrapper::ValueWrapper(int v) {
    _value = osgIntrospection::Value(v);
}

/******************************************************************************/
ValueWrapper::ValueWrapper(float v) {
    _value = osgIntrospection::Value(v);
}

/******************************************************************************/
ValueWrapper::ValueWrapper(std::string& v) {
    _value = osgIntrospection::Value(v);
}

/******************************************************************************/
ValueWrapper::ValueWrapper(const std::string& v) {
    _value = osgIntrospection::Value(v);
}

/******************************************************************************/
ValueWrapper::~ValueWrapper() {
}


/******************************************************************************/

int ValueWrapper::getInt() {
    int i = osgIntrospection::variant_cast<int>(_value);
    return i;
}

/******************************************************************************/

float ValueWrapper::getFloat() {
    float i = osgIntrospection::variant_cast<float>(_value);
    return i;
}

/******************************************************************************/

std::string ValueWrapper::getString() {
    return _value.toString();

}
/******************************************************************************/

boost::python::tuple ValueWrapper::getVector() {

    osg::Vec2 v2;
    osg::Vec3 v3;
    osg::Vec4 v4;

    try {
        v2 = osgIntrospection::variant_cast<osg::Vec2>(_value);
        return make_tuple( v2.x(), v2.y() );
    } catch (...) {
        // not it..
    }

    try {
        v3 = osgIntrospection::variant_cast<osg::Vec3>(_value);
        return make_tuple( v3.x(), v3.y(), v3.z() );
    } catch (...) {
        // still not it
    }

    try {
        v4 = osgIntrospection::variant_cast<osg::Vec4>(_value);
        return make_tuple( v4.x(), v4.y(), v4.z(), v4.w() );
    } catch (...) {
        // damn
    }




    return make_tuple(0);


}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/


int invokeMethod(const osgIntrospection::Value classInstance,
                 const osgIntrospection::Type &classType,
                 const std::string& method,
                 osgIntrospection::ValueList theArgs,
                 osgIntrospection::Value &rval)
{

    // TODO: we should try to store this globally somewhere, so that we don't do
    // a lookup every time there is a message:

    /*
    const osgIntrospection::Type &ReferencedNodeType = osgIntrospection::Reflection::getType("ReferencedNode");


    if ((classType==ReferencedNodeType) || (classType.isSubclassOf(ReferencedNodeType)))
    {
    */
        try {
            rval = classType.invokeMethod(method, classInstance, theArgs, true);
            // if we get this far, then the method invocation succeeded and
            // we can return:
            return 1;
        }
        catch (osgIntrospection::Exception & ex)
        {
            //std::cerr << "catch exception: " << ex.what() << std::endl;
        }

        // If the method wasn't found in the classInstance, then we need to go
        // through all base classes to see if method is contained in a parent class:
        for (int i=0; i<classType.getNumBaseTypes(); i++)
        {
            if (invokeMethod(classInstance, classType.getBaseType(i), method, theArgs, rval)) return 1;
        }
   // }

    return 0;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

ValueWrapper SceneManagerCallback_script(const char* symName, const char *types,
                                         std::string args, int argc)
{

    //int i;
    std::string    theMethod;
    osgIntrospection::ValueList theArgs;

    //printf("SceneManagerCallback_script: hi! %s, %s, [%s]\n", symName, types, args.c_str());

    t_symbol *s = gensym( symName );

    if (!s->s_thing)
    {
        std::cout << "oscParser: Could not find referenced object named: " << symName << std::endl;
        return ValueWrapper(0);
    }

    // get osgInrospection::Value from passed UserData by casting as the proper
    // referenced object pointer:

    osgIntrospection::Value classInstance;
    if (s->s_type == REFERENCED_STATESET)
        classInstance = osgIntrospection::Value(dynamic_cast<ReferencedStateSet*>(s->s_thing));
    else
        classInstance = osgIntrospection::Value(dynamic_cast<ReferencedNode*>(s->s_thing));



    // the getInstanceType() method however, gives us the real type being pointed at:
    const osgIntrospection::Type &classType = classInstance.getInstanceType();

    if (!classType.isDefined())
    {
        std::cout << "ERROR: oscParser cound not process message '" << symName
                  << ". osgIntrospection has no data for that node." << std::endl;
        return ValueWrapper(0);
    }

    int argn = 1;
    std::string tok;
    std::istringstream iss(args);
    getline(iss, tok, ' ');

    theMethod = tok;

    while ( getline(iss, tok, ' ') ) {

        //std::cout << tok << std::endl;

        if (lo_is_numerical_type((lo_type)types[argn])) {
            //   theArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
            theArgs.push_back( (float) atof(tok.c_str() ) );
        } else {
            theArgs.push_back( tok );
        }
        argn++;
    }

     // invoke the method on the node, and if it doesn't work, then just forward
    // the message:

    osgIntrospection::Value v;

    if (invokeMethod(classInstance, classType, theMethod, theArgs, v)) {

        if (s->s_type == REFERENCED_NODE) {
            //printf("calling eventscript...\n");
            dynamic_cast<ReferencedNode*>(s->s_thing)->callEventScript( theMethod );
        }

        return ValueWrapper(v);

    } else {
        std::cout << "Ignoring method '" << theMethod << "' for [" << s->s_name
                  << "], but forwarding message anyway...NOT!" << std::endl;
    }


    //pthread_mutex_unlock(&pthreadLock);

    return ValueWrapper(0);
}


/******************************************************************************/

double time_s() {
    return osg::Timer::instance()->time_s();
}
double time_m() {
    return osg::Timer::instance()->time_m();
}
double time_u() {
    return osg::Timer::instance()->time_u();
}
double time_n() {
    return osg::Timer::instance()->time_n();
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

BOOST_PYTHON_MODULE(libSPINPyWrap)
{
    def("callback", &SceneManagerCallback_script);
    def("time_s", &time_s);
    def("time_m", &time_m);
    def("time_u", &time_u);
    def("time_n", &time_n);

    class_<ValueWrapper>("Value")
        .def("getInt", &ValueWrapper::getInt)  // Add a regular member function.
        .def("getFloat", &ValueWrapper::getFloat)  // Add a regular member function.
        .def("getString", &ValueWrapper::getString)  // Add a regular member function.
        .def("getVector", &ValueWrapper::getVector)
    ;

}


/******************************************************************************/
