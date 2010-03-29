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


#include "spinContext.h"
#include "SceneManager.h"

using namespace boost::python;





//int SceneManagerCallback_script(const char* symName, const char *types, lo_arg **argv, int argc)
int SceneManagerCallback_script(const char* symName, const char *types, std::string args, int argc)
{

    //int i;
    std::string    theMethod;
    osgIntrospection::ValueList theArgs;

    printf("SceneManagerCallback_script: hi! %s, %s, [%s]\n", symName, types, args.c_str());

    // make sure there is at least one argument (ie, a method to call):
    //if (!argc) {
    //    printf("SceneManagerCallback_script: no args -> no method to call\n");
    //    return 1;
    // }
    // get the method (argv[0]):
    //if (lo_is_string_type((lo_type)types[0])) {
    //    theMethod = string((char *)argv[0]);
    //} else return 1;

    t_symbol *s = gensym( symName );

    if (!s->s_thing)
    {
        std::cout << "oscParser: Could not find referenced object named: " << symName << std::endl;
        return 1;
    }

    // get osgInrospection::Value from passed UserData by casting as the proper
    // referenced object pointer:

    osgIntrospection::Value classInstance;
    if (s->s_type == REFERENCED_STATE)
        classInstance = osgIntrospection::Value(dynamic_cast<ReferencedState*>(s->s_thing));
    else
        classInstance = osgIntrospection::Value(dynamic_cast<ReferencedNode*>(s->s_thing));



    // the getInstanceType() method however, gives us the real type being pointed at:
    const osgIntrospection::Type &classType = classInstance.getInstanceType();

    if (!classType.isDefined())
    {
        std::cout << "ERROR: oscParser cound not process message '" << symName << ". osgIntrospection has no data for that node." << std::endl;
        return 1;
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
    if (!invokeMethod(classInstance, classType, theMethod, theArgs))
    {
        std::cout << "Ignoring method '" << theMethod << "' for [" << s->s_name << "], but forwarding message anyway...NOT!" << std::endl;

        // HACK: TODO: fix this
        /*spinContext &spin = spinContext::Instance();
        if (spin.sceneManager->isServer())
        {

            lo_message msg = lo_message_new();
            for (i=0; i<argc; i++)
            {
                if (lo_is_numerical_type((lo_type)types[i]))
                {
                    lo_message_add_float(msg, (float) lo_hires_val((lo_type)types[i], argv[i]));
                } else {
                    lo_message_add_string(msg, (const char*) argv[i] );
                }
            }

            lo_send_message_from(spin.sceneManager->txAddr, spin.sceneManager->txServ, path, msg);

   class_<World>("World")
        .def("greet", &World::greet)
        .def("set", &World::set)
    ;


            }*/
    }


    //pthread_mutex_unlock(&pthreadLock);

    return 1;
}

// =============================================================================

BOOST_PYTHON_MODULE(libSPINPyWrap)
{


    def("callback", &SceneManagerCallback_script);

}
