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

#include "config.h"

#include <string>
#include <iostream>
#include <pthread.h>
#include <signal.h>

#include <osg/Version>
#include <osgDB/Registry>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <cppintrospection/Type>

#ifndef DISABLE_PYTHON
#include <boost/python.hpp>
#endif

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "spinapp.h"
#include "spindefaults.h"
#include "spinbasecontext.h"
#include "spinservercontext.h"
#include "scenemanager.h"
#include "spinutil.h"
#include "spinlog.h"
#include "nodevisitors.h"

#ifdef WITH_SPATOSC
#include <spatosc/scene.h>
#endif

#define UNUSED(x) ( (void)(x) )

extern pthread_mutex_t sceneMutex;

namespace spin
{

spinApp::spinApp() : hasAudioRenderer(false), userID_(getHostname()), sceneID(spin_defaults::SCENE_ID)
{
/*
#ifdef __Darwin
    setenv("OSG_LIBRARY_PATH", "@executable_path/../PlugIns", 1);
    setenv("DYLD_LIBRARY_PATH", "@executable_path/../libs", 1);
#endif
*/

    osgDB::FilePathList osgLibPaths;
    osgLibPaths.push_back("/opt/local/lib/osgPlugins-"+std::string(osgGetVersion()));
    osgLibPaths.push_back("/usr/lib/osgPlugins-"+std::string(osgGetVersion()));
    osgLibPaths.push_back("/usr/lib64/osgPlugins-"+std::string(osgGetVersion()));
    osgLibPaths.push_back("/usr/local/lib/osgPlugins-"+std::string(osgGetVersion()));
    osgLibPaths.push_back("/usr/local/lib64/osgPlugins-"+std::string(osgGetVersion()));
    osgDB::Registry::instance()->setLibraryFilePathList(osgLibPaths);

    // Set resourcesPath:
    
    // FIXME: this path should be replaced by PACKAGE_DATA/PACKAGE_NAME, not hard-coded
    resourcesPath_ = "/usr/local/share/spinframework";

    // for debugging, we'd like to use the local resources
    if (getenv("PWD"))
    {
        std::string currentDir = getenv("PWD");
        if ((currentDir.length() > 8) && (currentDir.substr(currentDir.length() - 9)) == std::string("/src/spin"))
        {
            resourcesPath_ = "../Resources";
        }
    }

    // get user defined env variable OSG_FILE_PATH
    osgDB::Registry::instance()->initDataFilePathList();

    // add all default resources paths to osg's file path list
    osgDB::FilePathList fpl = osgDB::getDataFilePathList();
    fpl.push_back( resourcesPath_ );
    fpl.push_back( resourcesPath_ + "/scripts/");
    fpl.push_back( resourcesPath_ + "/fonts/");
    fpl.push_back( resourcesPath_ + "/images/");
    fpl.push_back( resourcesPath_ + "/models/");
    fpl.push_back( resourcesPath_ + "/shaders/");
    osgDB::setDataFilePathList( fpl );


    /*
    // Load the SPIN library:
    std::string libname = "libspinframework-1.0.0";
    osgDB::Registry *reg = osgDB::Registry::instance();
    if (!osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit(libname)))
    {
        std::cout << "Error: Could not load " << libname << std::endl;
    } else {
        std::cout << "Successfully loaded " << libname << std::endl;
    }
    */

    // Make sure that our OSG nodekit is loaded (by checking for existence of
    // the ReferencedNode node type):
    try
    {
    	/*
        std::cout << "[DEBUG] These are all possible types:" << std::endl;
        const cppintrospection::TypeMap &allTypes = cppintrospection::Reflection::getTypes();
        for (cppintrospection::TypeMap::const_iterator it = allTypes.begin (); it != allTypes.end (); ++it)
        {
            if ( ((*it).second)->isDefined() )
            {
            	std::cout << ((*it).second)->getName() << " isAtomic?  " << ((*it).second)->isAtomic() << std::endl;
            	//std::cout << ((*it).second)->getName() << " isDefined? " << ((*it).second)->isDefined() << std::endl;
            }
        }
		*/
        const cppintrospection::Type &ReferencedNodeType = cppintrospection::Reflection::getType("spin::ReferencedNode");
        //UNUSED(ReferencedNodeType);
        if (!ReferencedNodeType.isDefined())
        {
            std::cout << "Introspection ERROR: Make sure that libSPIN exists and can be found." << std::endl;
        	exit(1);
        }
        else {
        	//std::cout << "Successfully loaded SPIN library" << std::endl;
        }
    }
    catch (cppintrospection::Exception & ex)
    {
        std::cout << "ERROR: " << ex.what() << ". This is likely a dynamic library problem. Make sure that libSPIN exists and can be found." << std::endl;
        exit(1);
    }

    // create local user directory (create it if doesn't exist):
    if (!osgDB::makeDirectory(SPIN_DIRECTORY))
    {
        std::cout << "ERROR: Could not create data folder for SPIN at: " << SPIN_DIRECTORY << std::endl;
        exit(1);
    }
    if (!osgDB::makeDirectory(SPIN_DIRECTORY+"/log"))
    {
        std::cout << "ERROR: Could not create log folder for SPIN at: " << SPIN_DIRECTORY+"/log" << std::endl;
        exit(1);
    }

    // timecode init:
    setSyncStart(0);

#ifdef WITH_SPATOSC
    audioScene = new spatosc::Scene();
    audioScene->setVerbose(true);
#endif

    _pyInitialized = false;
}

spinApp::~spinApp()
{

	if (userNode.valid())
	{
		sceneManager_->doDelete(userNode.get());
		userNode = 0;
	}
#ifdef WITH_SPATOSC
	delete audioScene;
#endif
	delete sceneManager_;
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
    context_ = c;
    // make scene manager
    //createScene(); // mikewoz moved this to the start of the client/server threads
}

void spinApp::createScene()
{
    if (context_)
    {
        sceneManager_ = new SceneManager(getSceneID());
    }
    else
    {
        std::cout << "ERROR. Cannot createScene because context has not been set in spinApp" << std::endl;
        exit(1);
    }
}
void spinApp::destroyScene()
{
	std::cout << "Cleaning up SceneManager..." << std::endl;
	
    
	// Force a delete (and destructor call) for all nodes still in the scene:
    unsigned int i = 0;
    ReferencedNode *n;
    while (i < sceneManager_->worldNode->getNumChildren())
    {
        if ((n = dynamic_cast<ReferencedNode*>(sceneManager_->worldNode->getChild(i))))
        {
            // delete the graph of any ReferencedNode:
            sceneManager_->deleteGraph(n->getID().c_str());
        }
        else
        {
            // it's possible that there are other nodes attached to worldNode,
            // so just skip them:
            i++;
        }
    }

    // clear any states that are left over:
    sceneManager_->clearStates();
}


// *****************************************************************************
// PYTHON STUFF:


bool spinApp::initPython()
{
    _pyInitialized = false;

#ifndef DISABLE_PYTHON

    try {
        Py_Initialize();
        PyEval_InitThreads();
        _pyMainModule = boost::python::import("__main__");
        _pyNamespace = _pyMainModule.attr("__dict__");

        exec("import sys", _pyNamespace, _pyNamespace);
        //////////exec("sys.path.append('/home/lwi')", _pyNamespace, _pyNamespace);
        ///exec("sys.path.append('/usr/local/share/spinFramework/scripts')", _pyNamespace, _pyNamespace);

        //exec("sys.path.append('/usr/local/lib')", _pyNamespace, _pyNamespace);


        exec(std::string("sys.path.append('" + resourcesPath_ +  "')").c_str(), _pyNamespace, _pyNamespace);

        //exec("print sys.path", _pyNamespace, _pyNamespace);

        //exec("import spin", _pyNamespace, _pyNamespace);
        exec("import spinframework", _pyNamespace, _pyNamespace);

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

#endif
    
    return _pyInitialized;
    
}
bool spinApp::execPython( const std::string& cmd )
{
#ifndef DISABLE_PYTHON
    
    if (!_pyInitialized) return false;

    try {
        exec(cmd.c_str(), _pyNamespace, _pyNamespace);

    } catch (boost::python::error_already_set const & ) {
        std::cout << getCurrentPyException() << std::endl;
        std::cout << "Python error: " << std::endl;
        PyErr_Print();
        PyErr_Clear();
        return false;
    } catch ( std::exception& e ) {
        std::cout << "Python error:" << e.what() << std::endl;
        return false;
    } catch(...) {                        // catch all other exceptions
        std::cout << "Python error: caught (...)\n";
        return false;
    }

    return true;
    
#else
    return false;
#endif
}

std::string spinApp::getCurrentPyException()
{
#ifndef DISABLE_PYTHON
  using namespace boost::python;
  namespace py = boost::python;
  printf("currentException...\n");
  PyObject *exc,*val,*tb;
  PyErr_Fetch(&exc,&val,&tb);
  handle<> hexc(exc),hval(val),htb(tb);
  if(!htb || !hval)
  {
      //MYAPP_ASSERT_BUG(hexc);
    return extract<std::string>(str(hexc));
  }
  else
  {
    object traceback(py::import("traceback"));
    object format_exception(traceback.attr("format_exception"));
    object formatted_list(format_exception(hexc,hval,htb));
    object formatted(str("\n").join(formatted_list));
    return extract<std::string>(formatted);
  }
#else
    return "Python interpreter disabled";
#endif
}


// *****************************************************************************

void spinApp::registerUser()
{
    if (sceneManager_)
    {
        // Here we force the creation of a (local) UserNode, so that we are sure
        // to have one, even if a server is not running. This way, we can create
        // our ViewerManipulator, and tracker node before receiver the official
        // createNode message from the server.

        userNode = dynamic_cast<UserNode*>(sceneManager_->getOrCreateNode(userID_.c_str(), "UserNode"));

        // We then send a message to the server to create the node. If the
        // server doesn't exist yet, it doesn't really matter, since we are
        // guaranteed to have a local instance. Then, once the server starts,
        // it will send a 'userRefresh' method that will inform it of this node
        // (see the spinApp_sceneCallback method)

        SceneMessage("sss", "createNode", userNode->getID().c_str(), "UserNode", SPIN_ARGS_END);
    }

    if (!userNode.valid())
    {
        std::cout << "ERROR: Could not registerUser '" << userID_ << "'. Perhaps SPIN is not running?" << std::endl;
    }
}

// *****************************************************************************
// A BUNCH OF MESSAGE SENDING UTILITIES:

void spinApp::InfoMessage(const std::string &OSCpath, const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    //InfoMessage(OSCpath, types, ap);
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);

    if (!err)
    {
        InfoMessage(OSCpath, msg);
    } else {
        lo_message_free(msg);
        std::cout << "ERROR (spinApp::InfoMessage): " << err << std::endl;
    }

}
/*
void spinApp::InfoMessage(const std::string &OSCpath, const char *types, va_list ap)
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
*/
void spinApp::InfoMessage(const std::string &OSCpath, lo_message msg)
{
    if ((context_) && (context_->doDiscovery_))
    {
        lo_send_message_from(context_->lo_infoAddr, context_->lo_infoServ_, OSCpath.c_str(), msg);

        // Let's free the message after (not sure if this is necessary):
        lo_message_free(msg);
    }
}


void spinApp::NodeMessage(const char *nodeId, const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    //NodeMessage(nodeId, types, ap);
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);

    if (!err)
    {
        NodeMessage(nodeId, msg);
    } else {
        lo_message_free(msg);
        std::cout << "ERROR (spinApp::NodeMessage): " << err << std::endl;
    }
}
/*
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
*/
void spinApp::NodeMessage(const char *nodeId, lo_message msg)
{
    if (context_ && context_->isRunning())
    {
        std::string OSCpath = "/SPIN/" + sceneID + "/" + nodeId;

        // If this thread is a listener, then we need to send an OSC message to
        // the rxAddr of the spinServer (unicast)
        if ( !context_->isServer() )
        {
            std::vector<lo_address>::iterator addrIter;
            for (addrIter = context_->lo_txAddrs_.begin(); addrIter != context_->lo_txAddrs_.end(); ++addrIter)
                lo_send_message((*addrIter), OSCpath.c_str(), msg);

        }

        // if, however, this process acts as a server, we can optimize and send
        // directly to the OSC callback function:
        else context_->nodeCallback(OSCpath.c_str(), lo_message_get_types(msg),
                lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)gensym(nodeId));

    } //else std::cout << "Error: tried to send NodeMessage but SPIN is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}
    
    
void spinApp::BroadcastNodeMessage(const char *nodeId, const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);
    
    if (!err)
    {
        std::string OSCpath = "/SPIN/" + sceneID + "/" + nodeId;
        BroadcastMessage(OSCpath, msg);
    }
    else
    {
        lo_message_free(msg);
    }
}



void spinApp::SceneMessage(const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    //SceneMessage(types, ap);
    
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);
    if (!err)
    {
        SceneMessage(msg);
    }
    else
    {
        lo_message_free(msg);
        std::cout << "ERROR (spinApp::SceneMessage): " << err << std::endl;
    }
}
/*
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
*/
// FIXME: gross, this should probably just be a template method in spinBaseContext with client/server specifics
// in their respective classes
void spinApp::SceneMessage(lo_message msg)
{
    if (context_ && context_->isRunning())
    {
        std::string OSCpath = "/SPIN/" + sceneID;

        // If this thread is a listener, then we need to send an OSC message to
        // the rxAddr of the spinServer (unicast)
        if ( !context_->isServer() )
        {
            std::vector<lo_address>::iterator addrIter;
            for (addrIter = context_->lo_txAddrs_.begin(); addrIter != context_->lo_txAddrs_.end(); ++addrIter)
                lo_send_message((*addrIter), OSCpath.c_str(), msg);
        }
        else
        {
            // if, however, this process acts as a server, we can optimize and send
        // directly to the OSC callback function:
            spinServerContext::sceneCallback(OSCpath.c_str(), lo_message_get_types(msg),
                    lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, NULL);
        }

    } //else std::cout << "Error: tried to send SceneMssage but SPIN is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}

    
void spinApp::BroadcastSceneMessage(const char *types, ...)
{
    va_list ap;
    va_start(ap, types);
    
    lo_message msg = lo_message_new();
    int err = lo_message_add_varargs(msg, types, ap);
    
    if (!err)
    {
        std::string OSCpath = "/SPIN/" + sceneID;
        BroadcastMessage(OSCpath, msg);
    }
    else
    {
        lo_message_free(msg);
    }
}


    
void spinApp::BroadcastMessage(std::string OSCpath, lo_message msg)
{
    if (context_ && context_->isRunning() && context_->isServer())
    {
        spinServerContext *serv = dynamic_cast<spinServerContext*>(context_);
        
        if (serv->hasReliableBroadcast())
        {
            std::map<std::string,lo_address>::iterator addrIter;
            for (addrIter=serv->tcpClientAddrs_.begin(); addrIter!=serv->tcpClientAddrs_.end(); ++addrIter)
            {
                lo_send_message(addrIter->second, OSCpath.c_str(), msg);
            }
        }
        //else
        // Actually, let's always send out on the UDP channels as well.
        {
            std::vector<lo_address>::iterator addrIter;
            for (addrIter = context_->lo_txAddrs_.begin(); addrIter != context_->lo_txAddrs_.end(); ++addrIter)
            {
                lo_send_message((*addrIter), OSCpath.c_str(), msg);
            }                
        }
    }
    
    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}
    

void spinApp::NodeBundle(std::string nodeId, std::vector<lo_message> msgs, lo_address addr)
{
    std::string OSCpath = "/SPIN/" + sceneID + "/" + nodeId;
    sendBundle(OSCpath, msgs, addr);
}

/*
void spinApp::SceneBundle(std::vector<lo_message> msgs)
{
    std::string OSCpath = "/SPIN/" + sceneID;
    sendBundle(OSCpath, msgs);
}
*/

void spinApp::SceneBundle(std::vector<lo_message> msgs, lo_address addr)
{
    std::string OSCpath = "/SPIN/" + sceneID;
    sendBundle(OSCpath, msgs, addr);
}


void spinApp::sendBundle(const std::string &OSCpath, std::vector<lo_message> msgs, lo_address txAddr)
{
    if (!context_->isServer()) return;

    spinServerContext *serv = dynamic_cast<spinServerContext*>(context_);

    // If a txAddr is provided, it is a TCP request from a single host and we 
    // just send there. Otherwise, txAddr will be 0, and we will send to
    // everyone.
    std::vector<lo_address> addressesToSendTo;
    if (txAddr == 0)
    {
        // add all UDP addresses:
        addressesToSendTo = context_->lo_txAddrs_;
        
        // also add all TCP subscribers if reliableBroadcast is enabled:
        if (serv->hasReliableBroadcast())
        {
            std::map<std::string,lo_address>::iterator addrIter;
            for (addrIter=serv->tcpClientAddrs_.begin(); addrIter!=serv->tcpClientAddrs_.end(); ++addrIter)
            {
                addressesToSendTo.push_back(addrIter->second);
            }
        }
    }
    else addressesToSendTo.push_back(txAddr);

/*
	lo_address sendingAddress;
	if (txAddr == 0) sendingAddress = context_->lo_txAddr;
	else sendingAddress = txAddr;
*/

    lo_bundle b = lo_bundle_new(LO_TT_IMMEDIATE);
    std::vector<lo_message>::iterator iter;
    for (iter = msgs.begin(); iter!=msgs.end(); ++iter)
    {
        lo_bundle_add_message(b, OSCpath.c_str(), (*iter));
    }

    std::vector<lo_address>::iterator addrIter;
    for (addrIter = addressesToSendTo.begin(); addrIter != addressesToSendTo.end(); ++addrIter)
    {

        // TCP in liblo can't handle bundles
        if (lo_address_get_protocol(*addrIter) == LO_TCP)
        {
            iter = msgs.begin();
            while (iter != msgs.end())
            {
                lo_send_message((*addrIter), OSCpath.c_str(), (*iter));
                ++iter;
                //msgs.erase(iter); // iterator automatically advances after erase()
            }
        }
        // if it's any UDP socket, then bundle the messages:
        else
        {
            //std::cout << "sending bundle of " << msgs.size() << " to " << lo_address_get_url(txAddr) << ", for:  " << OSCpath << std::endl;
/*
            lo_bundle b = lo_bundle_new(LO_TT_IMMEDIATE);
            for (iter = msgs.begin(); iter!=msgs.end(); ++iter)
            //iter = msgs.begin();
            //while (iter != msgs.end())
            {
                lo_bundle_add_message(b, OSCpath.c_str(), (*iter));
                //msgs.erase(iter); // iterator automatically advances after erase()
            }
*/
            lo_send_bundle((*addrIter), b);
            //lo_bundle_free_messages(b);
        }
    }
    lo_bundle_free_messages(b);
}

} // end of namespace spin
