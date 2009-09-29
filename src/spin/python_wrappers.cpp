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


#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <string.h>
#include "spinContext.h"


using namespace boost::python;

/*
class spinContext_wrapped : spinContext
{

public:
	spinContext_wrapped(PyObject* self) : m_self(self) {}

	virtual void sendSceneMessage(const char* types, ... )
	{
		va_list args;
		va_start(args, types);
		call_method< void >(m_self, "sendSceneMessage", types, args);
	}
	
private:
  PyObject* const m_self;

};
*/


// define member function pointer variables for overloaded functions:
//void (spinContext::*sendInfoMessage_var)(std::string, const char*, ...) = &spinContext::sendInfoMessage;
void (spinContext::*sendNodeMessage_var)(const char*, const char*, va_list) = &spinContext::sendNodeMessage;
void (spinContext::*sendSceneMessage_var)(const char*, va_list) = &spinContext::sendSceneMessage;

//void (spinContext_wrapped::*sendSceneMessage_var)(const char*, ...) = &spinContext_wrapped::sendSceneMessage;

//typedef void (spinContext::*sendSceneMessage_fnType)(const char*, va_list);

//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(spinContext_overloads, sendSceneMessage, 2, 12)



BOOST_PYTHON_MODULE(spinFramework)
{
	//class_<spinContext, spinContext_wrapped>("spinContext", init<int>())
	class_<spinContext>("spinContext", init<spinContext::spinContextMode>())

	.def("start", &spinContext::start)
	.def("stop", &spinContext::stop)
	
	//.def("sendInfoMessage", sendInfoMessage_var)
	.def("sendNodeMessage", sendNodeMessage_var)
	.def("sendSceneMessage", sendSceneMessage_var)
	
	
	//.def("sendSceneMessage", &spinContext::sendSceneMessage, spinContext_overloads())

	
	//.def("sendSceneMessage", &spinContext_wrapped::sendSceneMessage)
	
	//.def("sendSceneMessage", sendSceneMessage_fnType(&spinContext::sendSceneMessage))
	//.def("sendSceneMessage", raw_function(sendSceneMessage_var)
	
	.def("isRunning", &spinContext::isRunning)
	.def("setID", &spinContext::setID)
	.def("setRxAddr", &spinContext::setRxAddr)
	.def("setRxPort", &spinContext::setRxPort)
	.def("setTxAddr", &spinContext::setTxAddr)
	.def("setTxPort", &spinContext::setTxPort)
	;
}