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

#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include "spinContext.h"


using namespace boost::python;

/*
NOTES:

C declaration of a liblo callback:
int infoChannelCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

registering the callback:
lo_server_thread_add_method(lo_infoServ, NULL, NULL, infoChannelCallback, this);

*/

/*
// In python, we'll need to create an instance of the spinCallback class,
// and override the virtual functions defined here:
struct spinCallbacks
{
	virtual int sceneCallback(args) = 0;
	virtual int nodeCallback() = 0;
	virtual int infoCallback() = 0;
};

class spinCallbacksWrapper : spinCallbacks
{
	spinCallbacksWrapper(PyObject* self_) : self(self_) {}
	int sceneCallback()
	{
		return call_method<int>(self, "sceneCallback", args);
	}
	PyObject* self;
};
*/


class spinContext_wrapped : spinContext
{

public:
	spinContext_wrapped(PyObject* self_) : self(self_) {}

	virtual int sceneCallback(const char *types, lo_arg **argv, int argc)
	{
		// need to convert liblo args into something python will understand:
		
		// ???
		boost::python::tuple args = boost::python::make_tuple();
		
		for (int i=0; i<argc; i++)
		{
			if (lo_is_numerical_type((lo_type)types[i]))
			{
				args + boost::python::make_tuple( lo_hires_val((lo_type)types[i], argv[i]) );
			}
			else {
				args + boost::python::make_tuple( (const char*)argv[i] );
			}
		}
		
		
		call_method< void >(self, "sceneCallback", args);
	}
	
private:
	PyObject* const self;

};


//void (spinContext_wrapped::*sendSceneMessage_var)(const char*, ...) = &spinContext_wrapped::sendSceneMessage;
//typedef void (spinContext::*sendSceneMessage_fnType)(const char*, va_list);



// define member function pointer variables for overloaded functions:
void (spinContext::*sendInfoMessage_var)(std::string, const char*, va_list) = &spinContext::sendInfoMessage;
void (spinContext::*sendNodeMessage_var)(const char*, const char*, va_list) = &spinContext::sendNodeMessage;
void (spinContext::*sendSceneMessage_var)(const char*, va_list) = &spinContext::sendSceneMessage;




BOOST_PYTHON_MODULE(spinFramework)
{
	
	scope in_spinContext = class_<spinContext>("spinContext", init<spinContext::spinContextMode>())
	//class_<spinContext>("spinContext", init<int>())

	.def("start", &spinContext::start)
	.def("stop", &spinContext::stop)
	
	.def("sendInfoMessage", sendInfoMessage_var)
	.def("sendNodeMessage", sendNodeMessage_var)
	.def("sendSceneMessage", sendSceneMessage_var)

	.def("isRunning", &spinContext::isRunning)
	.def("setID", &spinContext::setID)
	.def("setRxAddr", &spinContext::setRxAddr)
	.def("setRxPort", &spinContext::setRxPort)
	.def("setTxAddr", &spinContext::setTxAddr)
	.def("setTxPort", &spinContext::setTxPort)

/*
	.value("SERVER_MODE", spinContext::SERVER_MODE)
	.value("LISTENER_MODE", spinContext::LISTENER_MODE)
	.export_values()
*/
	
	//implicitly_convertible<spinContext::spinContextMode,int>();
	//implicitly_convertible<int,spinContext::spinContextMode>();
	
	;
	
	
	enum_<spinContext::spinContextMode>("mode")
	    .value("SERVER_MODE", spinContext::SERVER_MODE)
	    .value("LISTENER_MODE", spinContext::LISTENER_MODE)
	    ;
	

	/*
	class_<spinContext_wrapped bases<spinContext> >("spinContext", init<spinContext::spinContextMode>())
	.def("sceneCallback", &spinContext_wrapped::sceneCallback)
	;
	*/
	
}
