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
#include <boost/lexical_cast.hpp>

#include <osgDB/Registry>
#include <cppintrospection/Type>
#include <cppintrospection/Value>

#include <osgUtil/Optimizer>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <boost/python.hpp>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "SceneManager.h"
#include "spinBaseContext.h"
#include "spinServerContext.h"
#include "spinClientContext.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "spinLog.h"
#include "nodeVisitors.h"
#include "SoundConnection.h"
#include "spinDefaults.h"
#include "config.h"

#ifdef WITH_SPATOSC
#include <spatosc/spatosc.h>
#endif


#define UNUSED(x) ( (void)(x) )

// TODO: make mutex a static member (perhaps of spinApp)?
pthread_mutex_t sceneMutex = PTHREAD_MUTEX_INITIALIZER;

namespace spin
{

/**
 * All threads need to stop according to the following flag. Note this used to
 * be a public class member, but we have sigHandler, which should be used
 * instead
 */
//namespace {
//    volatile bool signalStop;
//}
volatile bool spinBaseContext::signalStop = false;

spinBaseContext::spinBaseContext() :
    lo_infoAddr(NULL),
    lo_syncAddr(NULL),
    lo_infoServ_(NULL),
    lo_tcpRxServer_(NULL),
    pthreadID(0),
    doDiscovery_(true),
    autoPorts_(true)
{
    using namespace spin_defaults;
    signalStop = true;
    running = false;

    signal(SIGINT, sigHandler);

    // set default addresses (can be overridden):
    lo_infoAddr = lo_address_new(MULTICAST_GROUP, INFO_UDP_PORT);
    //lo_rxAddrs.push_back(lo_address_new(MULTICAST_GROUP, CLIENT_RX_UDP_PORT));
    //lo_txAddrs_.push_back(lo_address_new(MULTICAST_GROUP, CLIENT_TX_UDP_PORT));
    lo_syncAddr = lo_address_new(MULTICAST_GROUP, SYNC_UDP_PORT);
    

    tcpPort_ = SERVER_TCP_PORT;

    // override infoPort based on environment variable:
    char *infoPortStr = getenv("AS_INFOPORT");
    if (infoPortStr)
    {
        std::string tmpStr = std::string(infoPortStr);
        std::string infoAddr = tmpStr.substr(0, tmpStr.rfind(":"));
        std::string infoPort = tmpStr.substr(tmpStr.find(":") + 1);
        lo_address_free(lo_infoAddr);
        lo_infoAddr = lo_address_new(infoAddr.c_str(), infoPort.c_str());
    }
}

spinBaseContext::~spinBaseContext()
{
    this->stop();
	
    std::vector<lo_address>::iterator addrIter;
    for (addrIter = lo_txAddrs_.begin(); addrIter != lo_txAddrs_.end(); ++addrIter)
    {
    	lo_address_free(*addrIter);
    }
    lo_txAddrs_.clear();
    
    for (addrIter = lo_rxAddrs_.begin(); addrIter != lo_rxAddrs_.end(); ++addrIter)
    {
    	lo_address_free(*addrIter);
    }
    lo_rxAddrs_.clear();

    std::vector<lo_server>::iterator servIter;
    for (servIter = lo_rxServs_.begin(); servIter != lo_rxServs_.end(); ++servIter)
        lo_server_free(*servIter);
    lo_rxServs_.clear();

	lo_address_free(lo_infoAddr);
    lo_address_free(lo_syncAddr);

	// stop sceneManager OSC threads:
    //usleep(100);
	
    lo_server_free(lo_infoServ_);
    lo_server_free(lo_tcpRxServer_);
}

void spinBaseContext::debugPrint()
{
    std::cout << "\nSPIN context information:" << std::endl;
    std::cout << "  SceneManager ID:\t\t" << spinApp::Instance().getSceneID() << std::endl;
    std::cout << "  Resources path:\t\t" << spinApp::Instance().sceneManager->resourcesPath << std::endl;
#ifdef WITH_SPATOSC
    std::cout << "  SpatOSC version:\t\t"<< SPATOSC_VERSION << " (enabled=" << spinApp::Instance().hasAudioRenderer << ")" << std::endl;
#else
    std::cout << "  SpatOSC version:\t\tDISABLED" << std::endl;
#endif
    if (doDiscovery_)
    {
        std::cout << "  Auto discovery address:\t" << lo_address_get_url(lo_infoAddr);
        if (lo_address_get_ttl(lo_infoAddr)>0)
            std::cout << " TTL=" << lo_address_get_ttl(lo_infoAddr);
        std::cout << std::endl;
    } else {
        std::cout << "  Auto discovery address:\tOFF" << std::endl;
    }
    std::vector<lo_address>::iterator addrIter;
    for (addrIter = lo_txAddrs_.begin(); addrIter != lo_txAddrs_.end(); ++addrIter)
    {
        std::cout << "  Sending UDP to:\t\t" << lo_address_get_url(*addrIter);
        if (lo_address_get_ttl(*addrIter)>0)
            std::cout << " TTL=" << lo_address_get_ttl(*addrIter);
        std::cout << std::endl;
    }
    std::vector<lo_server>::iterator servIter;
    for (servIter = lo_rxServs_.begin(); servIter != lo_rxServs_.end(); ++servIter)
    {
        std::cout << "  Receiving UDP on:\t\t" << lo_server_get_url(*servIter) << std::endl;
    }
}

void spinBaseContext::addCommandLineOptions(osg::ArgumentParser *arguments)
{
    arguments->getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
    arguments->getApplicationUsage()->addCommandLineOption("--version", "Display the version number and exit.");
    arguments->getApplicationUsage()->addCommandLineOption("--scene-id <id>", "Specify the id of the SPIN scene (Default: default)");
    arguments->getApplicationUsage()->addCommandLineOption("--disable-discovery", "Disables the multicast discovery service");

    // TODO: add discovery addr <host> <port> (defaults is MULTICAST_GROUP:INFO_UDP_PORT)
    // for a client, this would be --recv-udp-discovery <host> <port>
    // for a server, this would be --send-udp-discovery <host> <port>
}

int spinBaseContext::parseCommandLineOptions(osg::ArgumentParser *arguments)
{
    // if user request help or version write it out to cout and quit.
    if (arguments->read("-h") || arguments->read("--help"))
    {
        arguments->getApplicationUsage()->write(std::cout);
        return 0;
    }
    if (arguments->read("--version"))
    {
        std::cout << VERSION << std::endl;
        return 0;
    }

    std::string sceneID;
    if (arguments->read("--scene-id", sceneID))
    {
        spinApp::Instance().setSceneID(sceneID);
    }

    if (arguments->read("--disable-discovery"))
    {
        doDiscovery_ = false;
    }

    std::string translatorName;
    std::string translatorAddr;
    std::string translatorPort;

    if (arguments->read("--spatosc", translatorName, translatorAddr, translatorPort))
    {
        // set up SpatOSC:
        #ifdef WITH_SPATOSC
        std::cout << "got commandline args: " << translatorName << " " << translatorAddr<< " " << translatorPort << std::endl;
        spinApp::Instance().audioScene->addTranslator<spatosc::BasicTranslator>("basic", "localhost", spatosc::BasicTranslator::DEFAULT_SEND_PORT);
        spinApp::Instance().hasAudioRenderer = true;
        #else
        std::cout << "WARNING: commandline option --spatosc not accepted. This version of SPIN was not built with support for SpatOSC."
        #endif
    }

    return 1;
}

void spinBaseContext::setLog(spinLog &log)
{
    std::vector<lo_server>::iterator it;
    for (it = lo_rxServs_.begin(); it != lo_rxServs_.end(); ++it)
    {
        lo_server_add_method((*it), NULL, NULL, logCallback, &log);
    }
}

void spinBaseContext::sigHandler(int signum)
{
    std::cout << "SPIN thread caught signal: " << signum << std::endl;

    spinBaseContext::signalStop = true;
    static int interruptCount = 0;
    const static int MAX_INTERRUPTS = 2;
    ++interruptCount;
    if (interruptCount >= MAX_INTERRUPTS)
        throw std::runtime_error("Got multiple interrupts, exitting rudely");
}

void spinBaseContext::setTTL(int ttl)
{
    std::vector<lo_address>::iterator addrIter;
    for (addrIter = lo_txAddrs_.begin(); addrIter != lo_txAddrs_.end(); ++addrIter)
        lo_address_set_ttl((*addrIter), ttl);

    lo_address_set_ttl(lo_infoAddr, ttl);
    lo_address_set_ttl(lo_syncAddr, ttl);
}
    
void spinBaseContext::addInfoHandler(EventHandler *obs)
{
    infoHandlers.push_back(obs);
}

void spinBaseContext::removeInfoHandler(EventHandler *obs)
{
    std::vector<EventHandler*>::iterator it;
    for (it=infoHandlers.begin(); it!=infoHandlers.end(); ++it)
    {
        if ((*it)==obs)
        {
            infoHandlers.erase(it);
            return;
        }
    }
}
void spinBaseContext::removeHandlerForAllEvents(EventHandler *obs)
{
    removeInfoHandler(obs);
    //removeNodeHandler(obs);
    //removeSceneHandler(obs);
}

/**
 * Startup point of the server's thread.
 */
bool spinBaseContext::startThread( void *(*threadFunction) (void*) )
{
    signalStop = false;

    // create thread:
    if (pthread_attr_init(&pthreadAttr) < 0)
    {
        std::cout << "spinBaseContext: could not prepare child thread" << std::endl;
        return false;
    }
    /*
       if (pthread_attr_setdetachstate(&pthreadAttr, PTHREAD_CREATE_DETACHED) < 0)
       {
       std::cout << "spinBaseContext: could not prepare child thread" << std::endl;
       return false;
       }
     */
    if (pthread_create( &pthreadID, &pthreadAttr, threadFunction, this) < 0)
    {
        std::cout << "spinBaseContext: could not create new thread" << std::endl;
        return false;
    }

    //pthread_join(pthreadID, NULL); // if not DETACHED thread

    // wait until the thread gets into it's loop before returning:
    while (! running)
        usleep(10);

    return true;
}

void spinBaseContext::stop()
{
    if (isRunning())
        signalStop = true;

    if (pthreadID != 0) 
    {
        pthread_join(pthreadID, NULL);
    }

    // wait here until the thread has really exited:
    while (isRunning())
        usleep(10);

}

int spinBaseContext::connectionCallback(const char *path, 
        const char *types, 
        lo_arg **argv, 
        int argc, 
        void * /*data*/, 
        void *user_data)
{
    std::string idStr;
    std::string theMethod;

    // make sure there is at least one argument (ie, a method to call):
    if (! argc)
        return 1;

    // get the method (argv[0]):
    if (lo_is_string_type((lo_type) types[0]))
    {
        theMethod = std::string((char *) argv[0]);
    }
    else
        return 1;

    // get the instance of the connection:
    SoundConnection *conn = (SoundConnection*) user_data;
    if (! conn)
    {
        std::cout << "oscParser: Could not find connection: " << idStr << std::endl;
        return 1;
    }

    // TODO: replace method call with osg::Introspection

    if (theMethod == "stateDump")
        conn->stateDump();
    else if (theMethod == "debug")
        conn->debug();
    else if ((argc==2) && (lo_is_numerical_type((lo_type) types[1])))
    {
        float value = lo_hires_val((lo_type) types[1], argv[1]);

        if (theMethod == "setThru")
            conn->setThru((bool) value);
        else if (theMethod == "setDistanceEffect")
            conn->setDistanceEffect(value);
        else if (theMethod == "setRolloffEffect")
            conn->setRolloffEffect(value);
        else if (theMethod == "setDopplerEffect")
            conn->setDopplerEffect(value);
        else if (theMethod == "setDiffractionEffect")
            conn->setDiffractionEffect(value);
        else
            std::cout << "Unknown OSC command: " << path << " " << theMethod << " (with " << argc-1 << " args)" << std::endl;
    }
    else
        std::cout << "Unknown OSC command: " << path << " " << theMethod << " (with " << argc-1 << " args)" << std::endl;
    return 1;
}

int spinBaseContext::nodeCallback(const char *path, const char *types, lo_arg **argv, int argc, void * /*data*/, void *user_data)
{
    // NOTE: user_data is a t_symbol pointer
    // FIXME: this method is way too long

    int i;
    std::string theMethod, nodeStr;
    cppintrospection::ValueList theArgs;

    // make sure there is at least one argument (ie, a method to call):
    if (! argc)
        return 1;
    // get the method (argv[0]):
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else
        return 1;
    t_symbol *s = (t_symbol*) user_data;
    
    if (! s->s_thing)
    {
        std::cout << "oscParser: Could not find referenced object named: " << nodeStr << std::endl;
        return 1;
    }

    // get osgInrospection::Value from passed UserData by casting as the proper
    // referenced object pointer:

    cppintrospection::Value classInstance;
    if (s->s_type == REFERENCED_STATESET)
    {
        classInstance = cppintrospection::Value(dynamic_cast<ReferencedStateSet*>(s->s_thing));
    }
    else
    {
        classInstance = cppintrospection::Value(dynamic_cast<ReferencedNode*>(s->s_thing));
    }

    // the getInstanceType() method however, gives us the real type being pointed at:
    const cppintrospection::Type &classType = classInstance.getInstanceType();

    if (! classType.isDefined())
    {
        std::cout << "ERROR: oscParser cound not process message '" << path << ". cppintrospection has no data for that node." << std::endl;
        return 1;
    }

    spinApp &spin = spinApp::Instance();

    if (theMethod == "addCronScript")
    {
        // client or server?
        if (! lo_is_numerical_type((lo_type)types[1]))
        {
            std::string s((const char*) argv[1]);
            if (s[0] == 'S' || s[0] == 's')
            {
                if (! spin.getContext()->isServer())
                    return 0;
                theArgs.push_back(true);
            }
            else if (s[0] == 'C' || s[0] == 'c')
            {
                if (spin.getContext()->isServer())
                {
                    lo_message msg = lo_message_new();
                    for (int i = 0; i < argc; i++)
                    {
                        if (lo_is_numerical_type((lo_type) types[i]))
                        {
                            lo_message_add_float(msg, (float) lo_hires_val((lo_type) types[i], argv[i]));
                        }
                        else
                        {
                            lo_message_add_string(msg, (const char*) argv[i] );
                        }
                    }
                    std::vector<lo_address>::iterator addrIter;
                    for (addrIter = spin.getContext()->lo_txAddrs_.begin(); addrIter != spin.getContext()->lo_txAddrs_.end(); ++addrIter)
                    {
                        lo_send_message_from((*addrIter), spin.getContext()->lo_infoServ_, path, msg);
                    }
                }
                theArgs.push_back(false); // serverSide arg
            }
            else
            {
                std::cout << "wrong server / client specifier... must be C or S" << std::endl;
            }

        }
        else
        {
            std::cout << "ERROR: client or server specifier for addCronScript is not a char" << std::endl;
            return 1;
        }

        // label
        if (! lo_is_numerical_type((lo_type)types[2]))
        {
            theArgs.push_back(std::string((const char*) argv[2]));
        }
        else
        {
            std::cout << "ERROR: label for addCronScript is not a string" << std::endl;
            return 1;
        }

        // Script path
        if (! lo_is_numerical_type((lo_type) types[3]))
        {
            theArgs.push_back(std::string((const char*) argv[3]));
        } else {
            std::cout << "ERROR: script path for addCronScript is not a string" << std::endl;
            return 1;
        }

        // frequency
        if (lo_is_numerical_type((lo_type) types[4]))
        {
            theArgs.push_back((double) lo_hires_val((lo_type) types[4], argv[4]) );
        }
        else
        {
            std::cout << "ERROR: frequency for addCronScript is not a number" << std::endl;
            return 1;
        }

        // rest of args are comma separated and passed as a string
        std::stringstream params("");
        for (i = 5; i < argc; i++)
        {
            if (lo_is_numerical_type((lo_type) types[i]))
            {
                params << ", " << (float) lo_hires_val((lo_type) types[i], argv[i]);
            }
            else
            {
                params << ", \"" << (const char*)argv[i] << "\"";
            }
        }
        theArgs.push_back(params.str());
    }
    else if (theMethod == "addEventScript")
    {
        // client or server?
        if (! lo_is_numerical_type((lo_type) types[1]))
        { /// (lo_type)types[1] == LO_CHAR )
            std::string s((const char*) argv[1]);
            if (s[0] == 'S' || s[0] == 's')
            {
                if (! spin.getContext()->isServer())
                    return 0;
                theArgs.push_back(true);
            } 
            else if (s[0] == 'C' || s[0] == 'c')
            {
                if (spin.getContext()->isServer())
                {
                    lo_message msg = lo_message_new();
                    for (int i = 0; i < argc; i++)
                    {
                        if (lo_is_numerical_type((lo_type) types[i]))
                        {
                            lo_message_add_float(msg, (float) lo_hires_val((lo_type) types[i], argv[i]));
                        }
                        else
                        {
                            lo_message_add_string(msg, (const char*) argv[i]);
                        }
                    }
                    std::vector<lo_address>::iterator addrIter;
                    for (addrIter = spin.getContext()->lo_txAddrs_.begin(); addrIter != spin.getContext()->lo_txAddrs_.end(); ++addrIter)
                    {
                        lo_send_message_from((*addrIter), spin.getContext()->lo_infoServ_, path, msg);
                    }
                }
                theArgs.push_back(false); // serverSide arg
            }
        }
        else 
        {
            std::cout << "ERROR: client or server specifier for addCronScript is not a char" << std::endl;
            return 1;
        }

        // label
        if (! lo_is_numerical_type((lo_type) types[2])) {
            theArgs.push_back(std::string((const char*) argv[2]));
        }
        else
        {
            std::cout << "ERROR: label for addEventScript is not a string" << std::endl;
            return 1;
        }

        // event method name
        if (! lo_is_numerical_type((lo_type) types[3]))
        {
            theArgs.push_back(std::string((const char*) argv[3]));
        }
        else 
        {
            std::cout << "ERROR: event method name for addEventScript is not a string" << std::endl;
            return 1;
        }

        // Script path
        if (! lo_is_numerical_type((lo_type) types[4]))
        {
            theArgs.push_back(std::string((const char*) argv[4]));
        }
        else 
        {
            std::cout << "ERROR: script path for addEventScript is not a string" << std::endl;
            return 1;
        }

        // rest of args are comma separated and passed as a string
        std::stringstream params("");
        for (i = 5; i < argc; i++)
        {
            if (lo_is_numerical_type((lo_type) types[i]))
            {
                params << ", " << (float) lo_hires_val((lo_type) types[i], argv[i]);
            }
            else 
            {
                params << ", \"" << (const char*) argv[i] << "\"";
            }
        }
        theArgs.push_back(params.str());
    }
    else 
    {
        for (i = 1; i < argc; i++) 
        {
            if (lo_is_numerical_type((lo_type)types[i]))
            {
                theArgs.push_back((float) lo_hires_val((lo_type)types[i], argv[i]));
            } 
            else
            {
                theArgs.push_back((const char*) argv[i]);
            }
        }
    }

    if (spin.getContext()->isServer() &&
            (theMethod == "enableCronScript" || theMethod == "removeCronScript" ||
             theMethod == "enableEventScript" || theMethod == "removeEventScript"))
    {
        lo_message msg = lo_message_new();
        for (int i = 0; i < argc; i++)
        {
            if (lo_is_numerical_type((lo_type)types[i]))
            {
                lo_message_add_float(msg, (float) lo_hires_val((lo_type)types[i], argv[i]));
            } 
            else
            {
                lo_message_add_string(msg, (const char*) argv[i] );
            }
        }
        std::vector<lo_address>::iterator addrIter;
        for (addrIter = spin.getContext()->lo_txAddrs_.begin(); addrIter != spin.getContext()->lo_txAddrs_.end(); ++addrIter)
        {
            lo_send_message_from((*addrIter), spin.getContext()->lo_infoServ_, path, msg);
        }
    }

    bool eventScriptCalled = false;
    if ( s->s_type == REFERENCED_NODE )
    {
        //printf("calling eventscript...\n");
        eventScriptCalled = dynamic_cast<ReferencedNode*>(s->s_thing)->callEventScript( theMethod, theArgs );
    }

    if (! eventScriptCalled ) 
    {   // if an eventScript was hooked to theMethod, do not execute theMethod.  functionality taken over by script
        // invoke the method on the node, and if it doesn't work, then just forward
        // the message:
        if (! introspector::invokeMethod(classInstance, classType, theMethod, theArgs))
        {
            //std::cout << "Ignoring method '" << theMethod << "' for [" << s->s_name << "], but forwarding message anyway..." << std::endl;
            // HACK: TODO: fix this
            if (spin.getContext()->isServer())
            {
                lo_message msg = lo_message_new();
                for (i = 0; i < argc; i++)
                {
                    if (lo_is_numerical_type((lo_type) types[i]))
                    {
                        lo_message_add_float(msg, (float) lo_hires_val((lo_type) types[i], argv[i]));
                    } else {
                        lo_message_add_string(msg, (const char*) argv[i]);
                    }
                }
                std::vector<lo_address>::iterator addrIter;
                for (addrIter = spin.getContext()->lo_txAddrs_.begin(); addrIter != spin.getContext()->lo_txAddrs_.end(); ++addrIter)
                {
                    lo_send_message_from((*addrIter), spin.getContext()->lo_infoServ_, path, msg);
                }
            }
        }
    }
    //pthread_mutex_unlock(&sceneMutex);
    return 1;
}

int spinBaseContext::sceneCallback(const char *path, const char *types, lo_arg **argv, int argc,
        void * /*data*/, void * /*user_data*/)
{
    spinApp &spin = spinApp::Instance();
    SceneManager *sceneManager = spin.sceneManager;

    // make sure there is at least one argument (ie, a method to call):
    if (!argc) return 1;

    // get the method (argv[0]):
    std::string theMethod;
    if (lo_is_string_type((lo_type)types[0]))
        theMethod = std::string((char *)argv[0]);
    else
        return 1;

    //pthread_mutex_lock(&sceneMutex);

    // note that args start at argv[1] now:
    if (theMethod == "debug")
    {
        if (argc>1)
        {
            std::string debugType = (const char*)argv[1];
            if (debugType=="context")
                sceneManager->debugContext();
            else if (debugType=="nodes")
                sceneManager->debugNodes();
            else if (debugType=="statesets")
                sceneManager->debugStateSets();
            else if (debugType=="scenegraph")
                sceneManager->debugSceneGraph();
            else if (debugType=="spatosc")
            {
                #ifdef WITH_SPATOSC
                if (spinApp::Instance().hasAudioRenderer)
                    spinApp::Instance().audioScene->debugPrint();
                else
                    std::cout << "SpatOSC not enabled for this SPIN application" << std::endl;
                #endif
            }

            // forward debug message to all clients:
            SCENE_MSG("ss", "debug", (const char*)argv[1]);
        }
        else
        {
            sceneManager->debug();
            SCENE_MSG("s", "debug");
        }
    }
    else if (theMethod == "clear")
        sceneManager->clear();
    else if (theMethod == "clearUsers")
        sceneManager->clearUsers();
    else if (theMethod == "clearStates")
        sceneManager->clearStates();
    else if (theMethod == "userRefresh")
    {
        if (spin.getContext()->isServer())
        {
            SCENE_MSG("s", "userRefresh");
        }
        else
        {
            spin.SceneMessage("sss", "createNode", spin.getUserID().c_str(), "UserNode", LO_ARGS_END);
            
            // if the server sends a userRefresh, it's possible that it has
            // only recently come online, so we need to re-subsrcibe:
            spinClientContext *clientContext = dynamic_cast<spinClientContext*>(spin.getContext());
            clientContext->subscribe();
        }
    }
    else if (theMethod == "refresh")
        sceneManager->refreshAll();
    else if (theMethod == "refreshSubscribers")
    {
        if (spin.getContext()->isServer())
        {
            spinServerContext *server = dynamic_cast<spinServerContext*>(spin.getContext());
            server->refreshSubscribers();
        }
    }
    else if (theMethod == "getNodeList")
        sceneManager->sendNodeList("*");
    else if ((theMethod == "nodeList") && (argc>2))
    {
        for (int i = 2; i < argc; i++)
        {
            if (strcmp((char*) argv[i],"NULL") != 0)
                sceneManager->createNode((char*)argv[i], (char*)argv[1]);
        }
    }
    else if ((theMethod=="stateList") && (argc>2))
    {
        for (int i=2; i<argc; i++)
        {
            if (strcmp((char*) argv[i],"NULL") != 0)
                sceneManager->createStateSet((char*) argv[i], (char*) argv[1]);
        }
    }
    else if ((theMethod == "exportScene") && (argc==3))
        sceneManager->exportScene((char*) argv[1], (char*) argv[2]);
    else if ((theMethod == "load") && (argc==2))
        sceneManager->loadXML((char*) argv[1]);
    else if ((theMethod == "save") && (argc==2))
        sceneManager->saveXML((char*) argv[1]);
    else if ((theMethod == "saveAll") && (argc==2))
        sceneManager->saveXML((char*) argv[1], true);
    else if ((theMethod == "saveUsers") && (argc==2))
        sceneManager->saveUsers((char*) argv[1]);
    else if ((theMethod == "createNode") && (argc==3))
        sceneManager->createNode((char*) argv[1], (char*) argv[2]);
    else if ((theMethod == "createStateSet") && (argc==3))
        sceneManager->createStateSet((char*) argv[1], (char*) argv[2]);
    else if ((theMethod == "createStateSet") && (argc==2))
        sceneManager->createStateSet((char*) argv[1]);
    else if ((theMethod == "deleteNode") && (argc==2))
        sceneManager->deleteNode((char*) argv[1]);
    else if ((theMethod == "deleteGraph") && (argc==2))
        sceneManager->deleteGraph((char*) argv[1]);
    else if ((theMethod == "optimize") && (argc==2))
    {
        osgUtil::Optimizer optimizer;
        if (std::string((char*) argv[1]) == "all")
        {
            std::cout << "Optimizing scene (all)" << std::endl;
            pthread_mutex_lock(&sceneMutex); 
            optimizer.optimize(sceneManager->worldNode.get(), osgUtil::Optimizer::ALL_OPTIMIZATIONS);
            pthread_mutex_unlock(&sceneMutex);
        }
        else
        {
            std::cout << "Optimizing scene" << std::endl;
            pthread_mutex_lock(&sceneMutex); 
            optimizer.optimize(sceneManager->worldNode.get());
            pthread_mutex_unlock(&sceneMutex); 
        }
        SCENE_MSG("ss", "optimize", (char*)argv[1]);
    }
    else
    {
        // FIXME: this used to rebroadcast messages that did not match command
#if 0
        spinApp &spin = spinApp::Instance();
        if (spin.sceneManager->isServer())
        {
            lo_message msg = lo_message_new();
            for (int i=0; i<argc; i++)
            {
                if (lo_is_numerical_type((lo_type)types[i]))
                {
                    lo_message_add_float(msg, (float) lo_hires_val((lo_type)types[i], argv[i]));
                } else {
                    lo_message_add_string(msg, (const char*) argv[i] );
                }
            }
            lo_send_message_from(spin.sceneManager->txAddr, spin.sceneManager->txServ, path, msg);
            //std::cout << "Unknown OSC command: " << path << " " << theMethod << " (with " << argc-1 << " args), but forwarding the message anyway." << std::endl;
        } else {
            //std::cout << "Unknown OSC command: " << path << " " << theMethod << " (with " << argc-1 << " args)" << std::endl;
        }
#endif
    }

    //pthread_mutex_unlock(&sceneMutex);

    return 1;
}

int spinBaseContext::logCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    spinLog *log = (spinLog*) user_data;

    *log << path << ' ' << types << ' ';
    for (int i = 0; i < argc; i++)
    {
        if (lo_is_numerical_type((lo_type) types[i]))
        {
            *log << (float) lo_hires_val((lo_type) types[i], argv[i]);
        } else {
            *log << (const char*) argv[i];
        }
        *log << ' ';
    }
    *log << std::endl;

    return 1;
}

int spinBaseContext::debugCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    printf("************ oscCallback_debug() got message: %s\n", (char*)path);
    printf("user_data: %s\n", (char*) user_data);
    for (int i = 0; i < argc; i++)
    {
        printf("arg %d '%c' ", i, types[i]);
        lo_arg_pp((lo_type) types[i], argv[i]);
        printf("\n");
    }
    printf("\n");
    fflush(stdout);
    return 1;
}

void spinBaseContext::oscParser_error(int num, const char *msg, const char *path)
{
    printf("OSC (liblo) error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}

void spinBaseContext::createServers()
{
    using boost::lexical_cast;
    using std::string;

    lo_tcpRxServer_ = lo_server_new_with_proto(tcpPort_.c_str(), LO_TCP, oscParser_error);
    if (lo_tcpRxServer_ == 0)
    {
        if (canAutoAssignPorts())
        {
            // liblo will try a random free port if the default failed
            std::cerr << "TCP receiver port " << tcpPort_ << " failed; trying a random port" << std::endl;
            lo_tcpRxServer_ = lo_server_new_with_proto(NULL, LO_TCP, oscParser_error);
        } else {
            std::cerr << "TCP receiver port " << tcpPort_ << " failed; SPIN was provided this port manually, so it will not attempt to use a random port. Quitting." << std::endl;
            exit(0);
        }
    }

    std::vector<lo_address>::iterator it;
    for (it = lo_rxAddrs_.begin(); it != lo_rxAddrs_.end(); ++it)
    {
        lo_server tmpServ;
        if (isMulticastAddress(lo_address_get_hostname(*it)))
        {
            tmpServ = lo_server_new_multicast(lo_address_get_hostname(*it), lo_address_get_port(*it), oscParser_error);
            if (tmpServ == 0)
            {
                std::cerr << "Multicast server creation on port " << lo_address_get_port(*it) << " failed, trying a random port" << std::endl;
                std::string addr(lo_address_get_hostname(*it));
                tmpServ = lo_server_new_multicast(addr.c_str(), NULL, oscParser_error);
                lo_address_free(*it);
                (*it) = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(tmpServ)).c_str());
            }
        }
        else
        {
            tmpServ = lo_server_new(lo_address_get_port(*it), oscParser_error);
            if (tmpServ == 0)
            {
                std::cerr << "UDP listener creation on port " << lo_address_get_port(*it) << " failed, trying a random port" << std::endl;
                tmpServ = lo_server_new(NULL, oscParser_error);
                std::string addr(lo_address_get_hostname(*it));
                lo_address_free(*it);
                (*it) = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(tmpServ)).c_str());
            }
        }
        lo_rxServs_.push_back(tmpServ);
    }

    // set up infoPort listener thread:
    if (isMulticastAddress(lo_address_get_hostname(lo_infoAddr)))
    {
        lo_infoServ_ = lo_server_new_multicast(lo_address_get_hostname(lo_infoAddr), lo_address_get_port(lo_infoAddr), oscParser_error);
        if (lo_infoServ_ == 0)
        {
            std::cerr << "Multicast info server creation on port " << lo_address_get_port(lo_infoAddr) << 
                " failed, trying a random port" << std::endl;
            std::string addr(lo_address_get_hostname(lo_infoAddr));
            lo_address_free(lo_infoAddr);
            lo_infoServ_ = lo_server_new_multicast(addr.c_str(), NULL, oscParser_error);
            lo_infoAddr = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(lo_infoServ_)).c_str());
        }
    } 
    else if (isBroadcastAddress(lo_address_get_hostname(lo_infoAddr)))
    {
        lo_infoServ_ = lo_server_new(lo_address_get_port(lo_infoAddr), oscParser_error);
        if (lo_infoServ_ == 0)
        {
            std::cerr << "Info server creation on port " << lo_address_get_port(lo_infoAddr) << 
                " failed, trying a random port" << std::endl;
            std::string addr(lo_address_get_hostname(lo_infoAddr));
            lo_address_free(lo_infoAddr);
            lo_infoServ_ = lo_server_new(NULL, oscParser_error);
            lo_infoAddr = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(lo_infoServ_)).c_str());
        }
        int sock = lo_server_get_socket_fd(lo_infoServ_);
        int sockopt = 1;
        setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &sockopt, sizeof(sockopt));
    } 
    else 
    {
        lo_infoServ_ = lo_server_new(lo_address_get_port(lo_infoAddr), oscParser_error);
        if (lo_infoServ_ == 0)
        {
            std::cerr << "Info server creation on port " << lo_address_get_port(lo_infoAddr) << 
                " failed, trying a random port" << std::endl;
            std::string addr(lo_address_get_hostname(lo_infoAddr));
            lo_address_free(lo_infoAddr);
            lo_infoServ_ = lo_server_new(NULL, oscParser_error);
            lo_infoAddr = lo_address_new(addr.c_str(), lexical_cast<string>(lo_server_get_port(lo_infoServ_)).c_str());
        }
    }
}

} // end of namespace spin

