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

#include <string>
#include <iostream>
#include <pthread.h>
#include <signal.h>


#include <osgDB/Registry>
#include <osgIntrospection/Type>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <boost/python.hpp>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "spinApp.h"
#include "spinBaseContext.h"
#include "SceneManager.h"
#include "spinUtil.h"
#include "spinLog.h"
#include "nodeVisitors.h"


#define UNUSED(x) ( (void)(x) )

extern pthread_mutex_t pthreadLock;


spinApp::spinApp()
{

#ifdef __Darwin
    setenv("OSG_LIBRARY_PATH", "@executable_path/../PlugIns", 1);
    //#define OSG_LIBRARY_PATH @executable_path/../PlugIns
#endif

    // Load the SPIN library:
    /*
    osgDB::Registry *reg = osgDB::Registry::instance();
    if (!osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit("libSPIN")))
    {
        std::cout << "Error: Could not load libSPIN" << std::endl;
    }
    if (!osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit("libSPINwrappers")))
    {
        std::cout << "Error: Could not load libSPINwrappers" << std::endl;
    }
    */


    // Make sure that our OSG nodekit    is loaded (by checking for existance of
    // the ReferencedNode node type):
    try
    {
        /*
        std::cout << "[DEBUG] These are all possible types:" << std::endl;
        const osgIntrospection::TypeMap &allTypes = osgIntrospection::Reflection::getTypes();
        for (osgIntrospection::TypeMap::const_iterator it = allTypes.begin (); it != allTypes.end (); ++it)
        {
            if ( ((*it).second)->isDefined() )
            std::cout << ((*it).second)->getName() << " isAtomic? " << ((*it).second)->isAtomic() << std::endl;
        }
        */
        const osgIntrospection::Type &ReferencedNodeType = osgIntrospection::Reflection::getType("ReferencedNode");
        UNUSED(ReferencedNodeType);
    }
    catch (osgIntrospection::Exception & ex)
    {
        std::cout << "ERROR: " << ex.what() << ". This is likely a dynamic library problem. Make sure that libSPIN exists and can be found." << std::endl;
        exit(1);
    }


    sceneID = "default";

    // check if local user directory exists, otherwise make it:
    try
    {
        using namespace boost::filesystem;

        if (!exists(SPIN_DIRECTORY))
        {
            create_directory(path(SPIN_DIRECTORY));
            create_directory(path(SPIN_DIRECTORY+"/log"));
        }
    }
    catch ( const boost::filesystem::filesystem_error& e )
    {
        std::cout << "ERROR: " << e.what() << std::endl;
        exit(1);
    }

    // timecode init:
    setSyncStart(0);

    _pyInitialized = false;

}

spinApp::~spinApp()
{
    delete sceneManager;
	//spinBaseContext::signalStop = true;
}

// Meyers Singleton design pattern:
spinApp& spinApp::Instance() {
    static spinApp s;
    return s;
}


/**
 * The first thing that any app must do is set the context for the spinApp
 * singleton. This will provide the singleton with access to the liblo addresses
 * so messages can be sent from anywhere.
 */
void spinApp::setContext(spinBaseContext *c)
{
	context = c;
    // make scene manager
    sceneManager = new SceneManager(getSceneID(), lo_address_get_hostname(getContext()->lo_rxAddr), lo_address_get_port(getContext()->lo_rxAddr));
}

// *****************************************************************************
// PYTHON STUFF:


bool spinApp::initPython()
{

    _pyInitialized = false;

    try {
        Py_Initialize();
        _pyMainModule = boost::python::import("__main__");
        _pyNamespace = _pyMainModule.attr("__dict__");

        exec("import sys", _pyNamespace, _pyNamespace);
        //////////exec("sys.path.append('/home/lwi')", _pyNamespace, _pyNamespace);
        ///exec("sys.path.append('/usr/local/share/spinFramework/scripts')", _pyNamespace, _pyNamespace);
        exec("sys.path.append('/usr/local/lib')", _pyNamespace, _pyNamespace);
        //exec("import spin", _pyNamespace, _pyNamespace);
        exec("import libSPINPyWrap", _pyNamespace, _pyNamespace);

    } catch (boost::python::error_already_set const & ) {
        std::cout << "sc: Python error: " << std::endl;
        PyErr_Print();
        return false;
    } catch ( std::exception& e ) {
        std::cout << "sc: what? " << e.what() << std::endl;
        return false;
    } catch(...) {                        // catch all other exceptions
        std::cout << "sc: Caught... something??\n";
        return false;
    }

    _pyInitialized = true;
    return true;

}
bool spinApp::execPython( const std::string& cmd ) {

    if (!_pyInitialized) return false;

    try {
        exec(cmd.c_str(), _pyNamespace, _pyNamespace);

    } catch (boost::python::error_already_set const & ) {
        ///if (PyErr_ExceptionMatches(PyExc_ZeroDivisionError))

        std::cout << "0: Python error: " << std::endl;
        PyErr_Print();
        return false;
    } catch ( std::exception& e ) {
        std::cout << "0: what? " << e.what() << std::endl;
        return false;
    } catch(...) {                        // catch all other exceptions
        std::cout << "0: Caught... something??\n";
        return false;
    }

    return true;
}

// *****************************************************************************

void spinApp::registerUser (const char *id)
{

    if (sceneManager)
    {
        // Here we force the creation of a (local) UserNode, so that we are sure
        // to have one, even if a server is not running. This way, we can create
        // our ViewerManipulator, and tracker node before receiver the official
        // createNode message from the server.

        userNode = dynamic_cast<UserNode*>(sceneManager->getOrCreateNode(id, "UserNode"));

        // We then send a message to the server to create the node. If the
        // server doesn't exist yet, it doesn't really matter, since we are
        // guaranteed to have a local instance. Then, once the server starts,
        // it will send a 'userRefresh' method that will inform it of this node
        // (see the spinApp_sceneCallback method)

        SceneMessage("sss", "createNode", userNode->id->s_name, "UserNode", LO_ARGS_END);

        std::cout << "  Registered user\t\t'" << userNode->id->s_name << "'" << std::endl;
    }

    if (!userNode.valid())
    {
        std::cout << "ERROR: Could not registerUser '" << id << "'. Perhaps SPIN is not running?" << std::endl;
    }
}

// *****************************************************************************
// A BUNCH OF MESSAGE SENDING UTILITIES:

void spinApp::InfoMessage(std::string OSCpath, const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    InfoMessage(OSCpath, types, ap);
}

void spinApp::InfoMessage(std::string OSCpath, const char *types, va_list ap)
{
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);

    if (!err)
    {
        InfoMessage(OSCpath, msg);
    } else {
        std::cout << "ERROR (spinApp::InfoMessage): " << err << std::endl;
    }
}

void spinApp::InfoMessage(std::string OSCpath, lo_message msg)
{
	if (context)
	{
		lo_send_message_from(context->lo_infoAddr, context->lo_infoServ, OSCpath.c_str(), msg);

		// Let's free the message after (not sure if this is necessary):
		lo_message_free(msg);
	}
}


void spinApp::NodeMessage(const char *nodeId, const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    NodeMessage(nodeId, types, ap);
}

void spinApp::NodeMessage(const char *nodeId, const char *types, va_list ap)
{
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);

    if (!err)
    {
        NodeMessage(nodeId, msg);
    } else {
        std::cout << "ERROR (spinApp::NodeMessage): " << err << std::endl;
    }
}


void spinApp::NodeMessage(const char *nodeId, lo_message msg)
{
    if (context && context->isRunning())
    {
        std::string OSCpath = "/SPIN/" + sceneID + "/" + nodeId;

        // If this thread is a listener, then we need to send an OSC message to
        // the rxAddr of the spinServer (unicast)
        if ( !context->isServer() )
        {
            lo_send_message(context->lo_txAddr, OSCpath.c_str(), msg);
        }

        // if, however, this process acts as a server, we can optimize and send
        // directly to the OSC callback function:
        else SceneManagerCallback_node(OSCpath.c_str(), lo_message_get_types(msg), lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)gensym(nodeId));

    } //else std::cout << "Error: tried to send NodeMessage but SPIN is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}


void spinApp::SceneMessage(const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    SceneMessage(types, ap);
}

void spinApp::SceneMessage(const char *types, va_list ap)
{
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);
    if (!err)
    {
        SceneMessage(msg);
    } else {
        std::cout << "ERROR (spinApp::SceneMessage): " << err << std::endl;
    }
}

void spinApp::SceneMessage(lo_message msg)
{
	if (context && context->isRunning())
    {
        std::string OSCpath = "/SPIN/" + sceneID;

        // If this thread is a listener, then we need to send an OSC message to
        // the rxAddr of the spinServer (unicast)
        if ( !context->isServer() )
        {
            lo_send_message(context->lo_txAddr, OSCpath.c_str(), msg);
        }

        // if, however, this process acts as a server, we can optimize and send
        // directly to the OSC callback function:
        else 
            SceneManagerCallback_admin(OSCpath.c_str(), lo_message_get_types(msg), 
                    lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)sceneManager);

    } //else std::cout << "Error: tried to send SceneMssage but SPIN is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}


void spinApp::NodeBundle(t_symbol *nodeSym, std::vector<lo_message> msgs)
{
    std::string OSCpath = "/SPIN/" + sceneID + "/" + std::string(nodeSym->s_name); 
    sendBundle(OSCpath,msgs);
}

void spinApp::SceneBundle(std::vector<lo_message> msgs)
{
    std::string OSCpath = "/SPIN/" + sceneID;
    sendBundle(OSCpath, msgs);
}

void spinApp::sendBundle(std::string OSCpath, std::vector<lo_message> msgs)
{
    lo_bundle b = lo_bundle_new(LO_TT_IMMEDIATE);

    std::vector<lo_message>::iterator iter = msgs.begin();
    while (iter != msgs.end())
    {
        //lo_send_message_from(txAddr, txServ, OSCpath.c_str(), (*iter));
        lo_bundle_add_message(b, OSCpath.c_str(), (*iter));
        msgs.erase(iter); // iterator automatically advances after erase()
    }

    lo_send_bundle(context->lo_txAddr, b);
    lo_bundle_free_messages(b);
}

