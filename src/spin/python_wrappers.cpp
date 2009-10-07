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


#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <string.h>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "spinContext.h"


using namespace boost::python;

/*
NOTES:

C declaration of a liblo callback:
int infoChannelCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);

registering the callback:
lo_server_thread_add_method(lo_infoServ, NULL, NULL, infoChannelCallback, this);

*/

class pySpinContext : public spinContext
{
	
	public:
		pySpinContext(spinContextMode initMode);
		virtual ~pySpinContext();
		void sendSceneMessage(PyObject* list);
	
};

pySpinContext::pySpinContext(spinContextMode initMode) : spinContext(initMode)
{}

pySpinContext::~pySpinContext()
{}

void pySpinContext::sendSceneMessage(PyObject* list)
{
	
	lo_message msg = lo_message_new();
	
	// ensure 1st argument is a string (ie, the method name):
	if (!PyString_Check(PyList_GetItem(list, 0)))
	{
		std::cout << "ERROR: sendSceneMessage() must have a method as the first item" << std::endl;
		return;
	}
	
	// add args to lo_message:
	PyObject *item;
	char types[PyList_Size(list)];
	for (int i=0; i<PyList_Size(list); i++)
	{
		/*
		item = PyList_GetItem(list, i);
		if (PyInt_Check(item))
		{
			types[i] = 'i';
			lo_message_add_int32(msg,(int)PyInt_AsLong(item));
		}
		else if (PyFloat_Check(item))
		{
			types[i] = 'f';
			lo_message_add_float(msg,(float)PyFloat_asDouble(item));	
		}	
		else if (PyString_Check(item))
		{
			types[i] = 's';
			lo_message_add_string(msg,PyString_AsString(item));	
		}
		 */
		
		item = PyList_GetItem(list, i);
		if (PyInt_Check(item))
		{
			types[i] = 'i';
			lo_message_add_int32(msg,extract<int>(item));
		}
		else if (PyFloat_Check(item))
		{
			types[i] = 'f';
			lo_message_add_float(msg,extract<float>(item));	
		}	
		else if (PyString_Check(item))
		{
			types[i] = 's';
			lo_message_add_string(msg,extract<char*>(item));	
		}
	}	
	
	std::cout << "got message from python:" << std::endl;
	lo_message_pp(msg);
	//sendSceneMessage(msg);
}


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

/*
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
*/


//void (spinContext_wrapped::*sendSceneMessage_var)(const char*, ...) = &spinContext_wrapped::sendSceneMessage;
//typedef void (spinContext::*sendSceneMessage_fnType)(const char*, va_list);



// define member function pointer variables for overloaded functions:
/*
void (spinContext::*sendInfoMessage_var)(std::string, const char*, va_list) = &spinContext::sendInfoMessage;
void (spinContext::*sendNodeMessage_var)(const char*, const char*, va_list) = &spinContext::sendNodeMessage;
void (spinContext::*sendSceneMessage_var)(const char*, va_list) = &spinContext::sendSceneMessage;
*/

void (pySpinContext::*sendSceneMessage_fromPython)(PyObject* list) = &pySpinContext::sendSceneMessage;




BOOST_PYTHON_MODULE(spinFramework)
{

	scope in_spinContext = class_<spinContext>("spinContext",  init<spinContext::spinContextMode>())
		.def("start", &spinContext::start)
		.def("stop", &spinContext::stop)
		/*
		.def("sendInfoMessage", sendInfoMessage_var)
		.def("sendNodeMessage", sendNodeMessage_var)
		.def("sendSceneMessage", sendSceneMessage_var)
		 */
		
		/*
		.def("isRunning", &spinContext::isRunning)
		.def("setID", &spinContext::setID)
		.def("setRxAddr", &spinContext::setRxAddr)
		.def("setRxPort", &spinContext::setRxPort)
		.def("setTxAddr", &spinContext::setTxAddr)
		.def("setTxPort", &spinContext::setTxPort)
		*/
		;
		
	enum_<spinContext::spinContextMode>("mode")
	    .value("SERVER_MODE", spinContext::SERVER_MODE)
	    .value("LISTENER_MODE", spinContext::LISTENER_MODE)
	    ;
	
	
	class_< pySpinContext, bases<spinContext> >("pySpinContext", init<spinContext::spinContextMode>())

		.def("sendSceneMessage", sendSceneMessage_fromPython)

		;
	
		
}
