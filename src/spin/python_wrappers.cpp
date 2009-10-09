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

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "spinContext.h"


using namespace boost::python;


class pySpinContext : public spinContext
{
	
	public:
		pySpinContext(PyObject *s, const spinContext& x) : spinContext(x), self(s) {}
		virtual ~pySpinContext() {}
		
		virtual bool start();
		
		void sendInfoMessage(PyObject* list);
		void sendSceneMessage(PyObject* list);
		void sendNodeMessage(PyObject* list);
		
		// these functions need to be overridden in Python:
		//virtual void sceneCallback(PyObject* dict) = 0; 
		
		virtual void sceneCallback(PyObject* dict) {
			std::cout << "in sceneCallback" << std::endl;
		}
		
		
	
		PyObject* self;
		
	private:
		lo_message lo_message_from_PyList(PyObject* list);
		
};

// If we have overloaded functions in C++, we need to explicitly define member
// function pointer variables in order to expose them to Python:
void (pySpinContext::*sendInfoMessage_fromPython)(PyObject* list) = &pySpinContext::sendInfoMessage;
void (pySpinContext::*sendSceneMessage_fromPython)(PyObject* list) = &pySpinContext::sendSceneMessage;
void (pySpinContext::*sendNodeMessage_fromPython)(PyObject* list) = &pySpinContext::sendNodeMessage;


// =============================================================================
// CALLBACK FUNCTIONS FROM LIBLO

int sceneCallback_wrapped(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
	pySpinContext *spin = (pySpinContext*) user_data;

	if (!spin) return 0;
	
	// make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

	// get the method (argv[0]):
	string theMethod;
	if (lo_is_string_type((lo_type)types[0]))
	{
		theMethod = string((char *)argv[0]);
	}
	else return 0;
	
	// create list from rest of args:
	boost::python::list args;
	for (int i=1; i<argc; i++)
	{
		if (types[i]=='s') args.append((char*)argv[i]);
		else if (types[i]=='i') args.append( (long)  lo_hires_val( (lo_type)types[i], argv[i] ));
		else if (types[i]=='f') args.append( (float) lo_hires_val( (lo_type)types[i], argv[i] ) );
		else if (types[i]=='d') args.append( (double) lo_hires_val( (lo_type)types[i], argv[i] ) );
	}
	
	// create a dict to return:
	boost::python::dict d;
	d["fullpath"] = path;
	d["method"] = theMethod.c_str();
	d["args"] = args;
	
	std::cout << "pySpinContext is valid... about to call python callback." << std::endl;
	
	
	try {
		call_method<int>(spin->self, "sceneCallback", d);
		return 0;
	} catch (error_already_set) {
		std::cout << "Error calling sceneCallback" << std::endl;
		// exception will be thrown if virtual method was not overridden.
		PyErr_Print();
		PyErr_Clear();
		return -1;
	}
	
	return 1;

}


// =============================================================================

bool pySpinContext::start()
{
	
	if (spinContext::start())
	{
		// now, register our additional callbacks:
		std::string OSCpath = std::string("/SPIN/") + this->id;
		std::cout << "registering callback for " << OSCpath << " on " << lo_server_get_url(lo_server_thread_get_server(this->sceneManager->rxServ)) << std::endl;
		
		//lo_server_thread_add_method(this->sceneManager->rxServ, OSCpath.c_str(), NULL, sceneCallback_wrapped, (void*)this);
		
		return true;
	}
	else {
		std::cout << "spinContext must be running in order to register callbacks. Use the start() method." << std::endl;
		return false;
	}
}

void pySpinContext::sendInfoMessage(PyObject* list)
{
	// ensure 1st argument is an OSC string:
	PyObject *oscPath = PyList_GetItem(list, 0);
	if (!PyString_Check(oscPath))
	{
		std::cout << "ERROR: sendInfoMessage() must provide an OSC-style string as the first item" << std::endl;
		return;
	}
	
	// pass the list, minus the 1st item:
	lo_message msg = lo_message_from_PyList(PyList_GetSlice(list, 1, PyList_Size(list)));
	spinContext::sendNodeMessage(extract<char*>(oscPath), msg);
}

void pySpinContext::sendSceneMessage(PyObject* list)
{
	lo_message msg = lo_message_from_PyList(list);
	spinContext::sendSceneMessage(msg);
}

void pySpinContext::sendNodeMessage(PyObject* list)
{
	// ensure 1st argument is a node name (ie, a string):
	PyObject *nodeName = PyList_GetItem(list, 0);
	if (!PyString_Check(nodeName))
	{
		std::cout << "ERROR: sendNodeMessage() must specify a node name as the first item" << std::endl;
		return;
	}

	// pass the list, minus the 1st item:
	lo_message msg = lo_message_from_PyList(PyList_GetSlice(list, 1, PyList_Size(list)));
	spinContext::sendNodeMessage(extract<char*>(nodeName), msg);
}

lo_message pySpinContext::lo_message_from_PyList(PyObject *list)
{
	lo_message msg = lo_message_new();
	
	// add args to lo_message:
	PyObject *item;
	for (int i=0; i<PyList_Size(list); i++)
	{
		item = PyList_GetItem(list, i);
		if (PyInt_Check(item))
		{
			lo_message_add_int32(msg,extract<int>(item));
		}
		else if (PyFloat_Check(item))
		{
			lo_message_add_float(msg,extract<float>(item));	
		}	
		else if (PyString_Check(item))
		{
			lo_message_add_string(msg,extract<char*>(item));	
		}
	}
	
	//std::cout << "created liblo message from python:" << std::endl;
	//lo_message_pp(msg);
	return msg;
}



// =============================================================================

BOOST_PYTHON_MODULE(spinFramework)
{
	// set the docstring of the current module scope
    //scope().attr("__doc__") = "my module's docstring";
/*
	{
	scope in_spinContext = class_<spinContext>("spinContext",  init<spinContext::spinContextMode>())
		.def("start", &spinContext::start)
		.def("stop", &spinContext::stop)		
		.def("isRunning", &spinContext::isRunning)
		.def("setID", &spinContext::setID)
		.def("setRxAddr", &spinContext::setRxAddr)
		.def("setRxPort", &spinContext::setRxPort)
		.def("setTxAddr", &spinContext::setTxAddr)
		.def("setTxPort", &spinContext::setTxPort)
		;
		
	enum_<spinContext::spinContextMode>("mode")
	    .value("SERVER_MODE", spinContext::SERVER_MODE)
	    .value("LISTENER_MODE", spinContext::LISTENER_MODE)
	    ;
		
	} // close scope for spinContext
*/
	 
	scope in_pySpinContext = class_< spinContext, pySpinContext >( "pySpinContext", init<const spinContext::spinContextMode&>() )
	//scope in_pySpinContext = class_< pySpinContext, bases<spinContext> >("pySpinContext", init<spinContext::spinContextMode>())
		
		.def("sendInfoMessage", sendInfoMessage_fromPython)
		.def("sendSceneMessage", sendSceneMessage_fromPython)
		.def("sendNodeMessage", sendNodeMessage_fromPython)

		.def("start", &pySpinContext::start)
		.def("stop", &spinContext::stop)
		.def("isRunning", &spinContext::isRunning)
		/*
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
}
