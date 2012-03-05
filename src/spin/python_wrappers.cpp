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

#ifndef DISABLE_PYTHON

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
#include <cppintrospection/Reflection>
#include <cppintrospection/Type>
#include <cppintrospection/Value>
#include <cppintrospection/variant_cast>
#include <cppintrospection/Exceptions>
#include <cppintrospection/MethodInfo>
#include <cppintrospection/PropertyInfo>
#include <cppintrospection/ReflectionMacros>
#include <cppintrospection/TypedMethodInfo>
#include <cppintrospection/StaticMethodInfo>
#include <cppintrospection/Attributes>
#include <cppintrospection/ExtendedTypeInfo>
#include <cppintrospection/variant_cast>
#include "spinApp.h"
#include "spinBaseContext.h"
#include "SceneManager.h"

using namespace boost::python;

namespace spin
{

/**
 * \brief Utility to wrap types for the Python wrapper.
 */
class ValueWrapper
{
public:
    ValueWrapper();
    ValueWrapper(cppintrospection::Value& v);
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
    cppintrospection::Value _value;
};

ValueWrapper::ValueWrapper() {
    _value = cppintrospection::Value(0);
}

ValueWrapper::ValueWrapper(cppintrospection::Value& v) {
    _value = v;
}

ValueWrapper::ValueWrapper(int v) {
    _value = cppintrospection::Value(v);
}

ValueWrapper::ValueWrapper(float v) {
    _value = cppintrospection::Value(v);
}

ValueWrapper::ValueWrapper(std::string& v) {
    _value = cppintrospection::Value(v);
}

ValueWrapper::ValueWrapper(const std::string& v) {
    _value = cppintrospection::Value(v);
}

ValueWrapper::~ValueWrapper() {
}

int ValueWrapper::getInt()
{
    int i = cppintrospection::variant_cast<int>(_value);
    return i;
}

float ValueWrapper::getFloat()
{
    float i = cppintrospection::variant_cast<float>(_value);
    return i;
}

std::string ValueWrapper::getString()
{
    return _value.toString();
}

boost::python::tuple ValueWrapper::getVector()
{
    try {
        osg::Vec2 v2;
        v2 = cppintrospection::variant_cast<osg::Vec2>(_value);
        return make_tuple( v2.x(), v2.y() );
    } catch (...) {
        // not it..
    }

    try {
        osg::Vec3 v3;
        v3 = cppintrospection::variant_cast<osg::Vec3>(_value);
        return make_tuple( v3.x(), v3.y(), v3.z() );
    } catch (...) {
        // still not it
    }

    try {
        osg::Vec4 v4;
        v4 = cppintrospection::variant_cast<osg::Vec4>(_value);
        return make_tuple( v4.x(), v4.y(), v4.z(), v4.w() );
    } catch (...) {
        // damn
    }
    return make_tuple(0);
}

int invokeMethod(const cppintrospection::Value classInstance,
                 const cppintrospection::Type &classType,
                 const std::string& method,
                 cppintrospection::ValueList theArgs,
                 cppintrospection::Value &rval)
{

    // TODO: we should try to store this globally somewhere, so that we don't do
    // a lookup every time there is a message:
    /*
    //const cppintrospection::Type &ReferencedNodeType = cppintrospection::Reflection::getType("ReferencedNode");
    if ((classType==ReferencedNodeType) || (classType.isSubclassOf(ReferencedNodeType)))
    {
    */
        try {
            rval = classType.invokeMethod(method, classInstance, theArgs, true);
            // if we get this far, then the method invocation succeeded and
            // we can return:
            return 1;
        }
        catch (cppintrospection::Exception & ex)
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

bool pyextract( boost::python::object obj, double* d )
{
    try {
        *d = boost::python::extract<double>( obj );
    } catch (...) {
        PyErr_Clear();
        return false;
    }
    return true;
}

bool pyextract( boost::python::object obj, float* d )
{
    try {
        *d = boost::python::extract<float>( obj );
    } catch (...) {
        PyErr_Clear();
        return false;
    }

    return true;
}

bool pyextract( boost::python::object obj, int* d )
{
    try {
        *d = boost::python::extract<int>( obj );
    } catch (...) {
        PyErr_Clear();
        return false;
    }
    return true;
}

std::string pyextract( boost::python::object obj )
{
    std::string s;
    try {
        s = boost::python::extract<std::string>( obj );
    } catch (...) {
        PyErr_Clear();
    }
    return s;
}

ValueWrapper SceneManagerCallback_script( const char* symName, const char* method,
                                          boost::python::list& argList, int cascadeEvents )
{
    //int i;
    std::string    theMethod( method );
    cppintrospection::ValueList theArgs;
    //printf("SceneManagerCallback_script: hi! %s, %s, [%s]\n", symName, types, args.c_str());
    t_symbol *s = gensym( symName );

    if (!s->s_thing)
    {
        //std::cout << "oscParser: Could not find referenced object named: " << symName << std::endl;
        return ValueWrapper(0);
    }

    // get osgInrospection::Value from passed UserData by casting as the proper
    // referenced object pointer:
    cppintrospection::Value classInstance;
    if (s->s_type == REFERENCED_STATESET)
        classInstance = cppintrospection::Value(dynamic_cast<ReferencedStateSet*>(s->s_thing));
    else
        classInstance = cppintrospection::Value(dynamic_cast<ReferencedNode*>(s->s_thing));

    // the getInstanceType() method however, gives us the real type being pointed at:
    const cppintrospection::Type &classType = classInstance.getInstanceType();

    if (! classType.isDefined())
    {
        std::cout << "ERROR: oscParser cound not process message '" << symName
                  << ". cppintrospection has no data for that node." << std::endl;
        return ValueWrapper(0);
    }

    size_t nbArgs = boost::python::len( argList );
    double da; float fa; int ia; std::string sa;

    for ( size_t i = 0; i < nbArgs; i++ )
    {

        if ( pyextract( argList[i], &da ) )
            theArgs.push_back( (double) da );
        else if ( pyextract( argList[i], &fa ) )
            theArgs.push_back( (float) fa );
        else if ( pyextract( argList[i], &ia ) )
            theArgs.push_back( (int) ia );
        else
        {
            sa = pyextract( argList[i] );
            if (sa != "" )
                theArgs.push_back( (const char*) sa.c_str() );
        }
    }

    // invoke the method on the node, and if it doesn't work, then just forward
    // the message:
    cppintrospection::Value v;
    bool eventScriptCalled = false;
    if (cascadeEvents)
    {
        if (s->s_type == REFERENCED_NODE)
        {
            //printf("calling eventscript...\n");
            eventScriptCalled = dynamic_cast<ReferencedNode*>(s->s_thing)->callEventScript( theMethod, theArgs );
        }
    }

    if (eventScriptCalled)
    { // cascaded event script called successful, do not invokeMethod, return nothing to the python script
        return ValueWrapper(0);
    }
    else
    { // no cascaded event assigned to theMethod or cascaded event script failed.  call invokeMethod and return result to python script
        if (invokeMethod(classInstance, classType, theMethod, theArgs, v))
        {
            return ValueWrapper(v);
        }
        else
        {
            if (spinApp::Instance().getContext()->isServer())
            {
                lo_message msg = lo_message_new();
                lo_message_add_string(msg, theMethod.c_str());
                for ( size_t i = 0; i < nbArgs; i++ )
                {
                    if ( pyextract( argList[i], &da ) ) lo_message_add_float(msg, (float) da );
                    else if ( pyextract( argList[i], &fa ) ) lo_message_add_float(msg, (float) fa );
                    else if ( pyextract( argList[i], &ia ) ) lo_message_add_float(msg, (float) ia );
                    else
                    {
                        sa = pyextract( argList[i] );
                        if (sa != "" ) lo_message_add_string(msg, (const char*) sa.c_str() );
                    }
                }
                std::string path = "/SPIN/" + spinApp::Instance().getSceneID() + "/" + s->s_name;
                std::vector<lo_address>::iterator addrIter;
                for (addrIter = spinApp::Instance().getContext()->lo_txAddrs_.begin(); addrIter != spinApp::Instance().getContext()->lo_txAddrs_.end(); ++addrIter)
                {
                    lo_send_message_from((*addrIter), spinApp::Instance().getContext()->lo_infoServ_, path.c_str(), msg);
                }
            }
        }

    }
    //pthread_mutex_unlock(&sceneMutex);
    return ValueWrapper(0);
}

double time_s()
{
    return osg::Timer::instance()->time_s();
}
double time_m()
{
    return osg::Timer::instance()->time_m();
}
double time_u()
{
    return osg::Timer::instance()->time_u();
}
double time_n()
{
    return osg::Timer::instance()->time_n();
}

/**
 * Please document this.
 */
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

} // end of namespace spin

#endif