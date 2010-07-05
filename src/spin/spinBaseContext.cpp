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
#include <osgIntrospection/Value>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <boost/python.hpp>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "SceneManager.h"
#include "spinBaseContext.h"
#include "spinServerContext.h"
#include "spinUtil.h"
#include "spinApp.h"
#include "spinLog.h"
#include "nodeVisitors.h"
#include "SoundConnection.h"

#define UNUSED(x) ( (void)(x) )

// TODO: make mutex a static member (perhaps of spinApp)?
pthread_mutex_t pthreadLock = PTHREAD_MUTEX_INITIALIZER;


bool spinBaseContext::signalStop = false;

spinBaseContext::spinBaseContext()
{
    signalStop = true;
    running = false;

    signal(SIGINT, sigHandler);

    // set default addresses (can be overridden):
    lo_infoAddr = lo_address_new("226.0.0.1", "54320");
    lo_rxAddr = lo_address_new("226.0.0.1", "54323");
    lo_txAddr = lo_address_new("226.0.0.1", "54324");
    lo_syncAddr = lo_address_new("226.0.0.1", "54321");

    // override infoPort based on environment variable:
    char *infoPortStr = getenv("AS_INFOPORT");
    if (infoPortStr)
    {
        std::string tmpStr = std::string(infoPortStr);
        std::string infoAddr = tmpStr.substr(0,tmpStr.rfind(":"));
        std::string infoPort = tmpStr.substr(tmpStr.find(":")+1);
        lo_infoAddr = lo_address_new(infoAddr.c_str(), infoPort.c_str());
    }

}

spinBaseContext::~spinBaseContext()
{
    this->stop();

    if (lo_infoServ)
        lo_server_free(lo_infoServ);
    if (lo_rxAddr)
        lo_address_free(lo_rxAddr);
    if (lo_txAddr)
        lo_address_free(lo_txAddr);
    if (lo_infoAddr)
        lo_address_free(lo_infoAddr);
    if (lo_syncAddr)
        lo_address_free(lo_syncAddr);
    if (lo_tcpRxServer_)
        lo_server_free(lo_tcpRxServer_);
    // stop sceneManager OSC threads:
    usleep(100);

    // FIXME: is this necessary/a good idea?
    if (lo_rxServ_)
        lo_server_free(lo_rxServ_);
}

void spinBaseContext::setLog(spinLog &log)
{
    lo_server_add_method(lo_rxServ_, NULL, NULL, logCallback, &log);
}

void spinBaseContext::sigHandler(int signum)
{
    std::cout << " Caught signal: " << signum << std::endl;

//    spinApp &spin = spinApp::Instance();

    // unlock mutex so we can clean up:
    pthread_mutex_unlock(&pthreadLock);

#if 0
    // TODO: we really shouldn't do anything like this here. Can we get rid of
    // this?:
    if (spin.userNode.valid())
    {
        ReferencedNode *heldPointer = spin.userNode.get();
        //spin.userNode.release();
        spin.userNode = NULL;
        // now deleting the node in the sceneManager should release the last instance:
        spin.sceneManager->doDelete(heldPointer);
    }
#endif

    spinBaseContext::signalStop = true;
}

bool spinBaseContext::startThread( void *(*threadFunction) (void*) )
{
	std::cout << "  SceneManager ID:\t\t" << spinApp::Instance().getSceneID() << std::endl;
    std::cout << "  INFO channel:\t\t\t" << lo_address_get_url(lo_infoAddr) << std::endl;
    std::cout << "  SYNC channel:\t\t\t" << lo_address_get_url(lo_syncAddr) << std::endl;
    std::cout << "  TX channel:\t\t\t" << lo_address_get_url(lo_txAddr) << std::endl;

    signalStop = false;

    // create thread:
    if (pthread_attr_init(&pthreadAttr) < 0)
    {
        std::cout << "spinBaseContext: could not prepare child thread" << std::endl;
        return false;
    }
    if (pthread_attr_setdetachstate(&pthreadAttr, PTHREAD_CREATE_DETACHED) < 0)
    {
        std::cout << "spinBaseContext: could not prepare child thread" << std::endl;
        return false;
    }
    if (pthread_create( &pthreadID, &pthreadAttr, threadFunction, this) < 0)
    {
        std::cout << "spinBaseContext: could not create new thread" << std::endl;
        return false;
    }

    //pthread_join(pthreadID, NULL); // if not DETACHED thread

    // wait until the thread gets into it's loop before returning:
    while (!running) usleep(10);

    return true;
}

void spinBaseContext::stop()
{
    if (isRunning())
    {
        std::cout << "Stopping spinBaseContext..." << std::endl;
        signalStop = true;
        while (running) usleep(10);
    }
}

int spinBaseContext::connectionCallback(const char *path, 
        const char *types, 
        lo_arg **argv, 
        int argc, 
        void * /*data*/, 
        void *user_data)
{
    std::string theMethod, idStr;

    // make sure there is at least one argument (ie, a method to call):
    if (!argc) return 1;

    // get the method (argv[0]):
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else return 1;

    // get the instance of the connection:
    SoundConnection *conn = (SoundConnection*) user_data;
    if (!conn)
    {
        std::cout << "oscParser: Could not find connection: " << idStr << std::endl;
        return 1;
    }

    // TODO: replace method call with osg::Introspection

    if (theMethod=="stateDump")
        conn->stateDump();
    else if (theMethod=="debug")
        conn->debug();
    else if ((argc==2) && (lo_is_numerical_type((lo_type)types[1])))
    {
        float value = lo_hires_val((lo_type)types[1], argv[1]);

        if (theMethod=="setThru")
            conn->setThru((bool) value);
        else if (theMethod=="setDistanceEffect")
            conn->setDistanceEffect(value);
        else if (theMethod=="setRolloffEffect")
            conn->setRolloffEffect(value);
        else if (theMethod=="setDopplerEffect")
            conn->setDopplerEffect(value);
        else if (theMethod=="setDiffractionEffect")
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

    int i;
    std::string theMethod, nodeStr;
    osgIntrospection::ValueList theArgs;

    // make sure there is at least one argument (ie, a method to call):
    if (!argc) return 1;

    // get the method (argv[0]):
    if (lo_is_string_type((lo_type)types[0]))
    {
        theMethod = std::string((char *)argv[0]);
    }
    else
        return 1;

    // get the instance of the node, which is the last token of the OSCpath:
    // TODO: use user_data instead!
    /*
    nodeStr = string(path);
    nodeStr = nodeStr.substr(nodeStr.rfind("/")+1);
    t_symbol *s = gensym(nodeStr.c_str());
    */
    t_symbol *s = (t_symbol*) user_data;

    if (!s->s_thing)
    {
        std::cout << "oscParser: Could not find referenced object named: " << nodeStr << std::endl;
        return 1;
    }

    // get osgInrospection::Value from passed UserData by casting as the proper
    // referenced object pointer:

    osgIntrospection::Value classInstance;
    if (s->s_type == REFERENCED_STATESET)
    {
        classInstance = osgIntrospection::Value(dynamic_cast<ReferencedStateSet*>(s->s_thing));
    }
    else
    {
        classInstance = osgIntrospection::Value(dynamic_cast<ReferencedNode*>(s->s_thing));

    }


    // the getInstanceType() method however, gives us the real type being pointed at:
    const osgIntrospection::Type &classType = classInstance.getInstanceType();



    if (!classType.isDefined())
    {
        std::cout << "ERROR: oscParser cound not process message '" << path << ". osgIntrospection has no data for that node." << std::endl;
        return 1;
    }

    spinApp &spin = spinApp::Instance();

    if (theMethod == "addCronScript") {

        // client or server?
        if ( !lo_is_numerical_type((lo_type)types[1]) ) {

            std::string s( (const char*)argv[1] );
            if ( s[0] == 'S' || s[0] == 's' ) {

                if ( !spin.getContext()->isServer() ) return 0;
                theArgs.push_back( true );

            } else if ( s[0] == 'C' || s[0] == 'c' ) {

                if ( spin.getContext()->isServer() ) {
                    lo_message msg = lo_message_new();
                    for (int i = 0; i < argc; i++) {
                        if (lo_is_numerical_type((lo_type)types[i])) {
                            lo_message_add_float(msg, (float) lo_hires_val((lo_type)types[i], argv[i]));
                        } else {
                            lo_message_add_string(msg, (const char*) argv[i] );
                        }
                    }
                    lo_send_message_from(spin.getContext()->lo_txAddr, spin.getContext()->lo_infoServ, path, msg);

                }
                theArgs.push_back( false ); // serverSide arg

            } else {
                std::cout << "wrong server / client specifier... must be C or S" << std::endl;
            }

        } else {
            std::cout << "ERROR: client or server specifier for addCronScript is not a char" << std::endl;
            return 1;
        }

        // label
        if (!lo_is_numerical_type((lo_type)types[2])) {
            theArgs.push_back( std::string( (const char*)argv[2] ) );
        } else {
            std::cout << "ERROR: label for addCronScript is not a string" << std::endl;
            return 1;
        }

        // Script path
        if (!lo_is_numerical_type((lo_type)types[3])) {
            theArgs.push_back( std::string( (const char*)argv[3] ) );
        } else {
            std::cout << "ERROR: script path for addCronScript is not a string" << std::endl;
            return 1;
        }

        // frequency
        if (lo_is_numerical_type((lo_type)types[4])) {
            theArgs.push_back( (double) lo_hires_val((lo_type)types[4], argv[4]) );
        } else {
            std::cout << "ERROR: frequency for addCronScript is not a number" << std::endl;
            return 1;
        }

        // rest of args are comma separated and passed as a string
         std::stringstream params("");
        for (i = 5; i < argc; i++) {
            if (lo_is_numerical_type((lo_type)types[i]))  {
                params << ", " << (float) lo_hires_val((lo_type)types[i], argv[i]);
            } else {
                params << ", \"" << (const char*)argv[i] << "\"";
            }
        }
        theArgs.push_back( params.str() );

    } else if (theMethod == "addEventScript") {

        // client or server?
        if ( !lo_is_numerical_type((lo_type)types[1]) ) { ///(lo_type)types[1] == LO_CHAR ) {
            std::string s( (const char*)argv[1] );

            if ( s[0] == 'S' || s[0] == 's' ) {

                if ( !spin.getContext()->isServer() ) return 0;
                theArgs.push_back( true );

            } else if ( s[0] == 'C' || s[0] == 'c' ) {

                if ( spin.getContext()->isServer() ) {
                    lo_message msg = lo_message_new();
                    for (int i = 0; i < argc; i++) {
                        if (lo_is_numerical_type((lo_type)types[i])) {
                            lo_message_add_float(msg, (float) lo_hires_val((lo_type)types[i], argv[i]));
                        } else {
                            lo_message_add_string(msg, (const char*) argv[i] );
                        }
                    }
                    lo_send_message_from(spin.getContext()->lo_txAddr, spin.getContext()->lo_infoServ, path, msg);
                }
                theArgs.push_back( false ); // serverSide arg
            }
        } else {
            std::cout << "ERROR: client or server specifier for addCronScript is not a char" << std::endl;
            return 1;
        }

        // label
        if (!lo_is_numerical_type((lo_type)types[2])) {
            theArgs.push_back( std::string( (const char*)argv[2] ) );
        } else {
            std::cout << "ERROR: label for addEventScript is not a string" << std::endl;
            return 1;
        }

        // event method name
        if (!lo_is_numerical_type((lo_type)types[3])) {
            theArgs.push_back( std::string( (const char*)argv[3] ) );
        } else {
            std::cout << "ERROR: event method name for addEventScript is not a string" << std::endl;
            return 1;
        }

        // Script path
        if (!lo_is_numerical_type((lo_type)types[4])) {
            theArgs.push_back(  std::string( (const char*)argv[4] ) );
        } else {
            std::cout << "ERROR: script path for addEventScript is not a string" << std::endl;
            return 1;
        }

        // rest of args are comma separated and passed as a string
        std::stringstream params("");
        for (i = 5; i < argc; i++) {
            if (lo_is_numerical_type((lo_type)types[i]))  {
                params << ", " << (float) lo_hires_val((lo_type)types[i], argv[i]);
            } else {
                params << ", \"" << (const char*)argv[i] << "\"";
            }
        }

        theArgs.push_back( params.str() );

    } else {

        for (i=1; i<argc; i++) {
            if (lo_is_numerical_type((lo_type)types[i]))  {
                theArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
            } else {
                theArgs.push_back( (const char*) argv[i] );
            }
        }

    }


    if ( spin.getContext()->isServer() &&
         (theMethod == "enableCronScript" || theMethod == "removeCronScript" ||
          theMethod == "enableEventScript" || theMethod == "removeEventScript") ) {

        lo_message msg = lo_message_new();
        for (int i = 0; i < argc; i++) {
            if (lo_is_numerical_type((lo_type)types[i])) {
                lo_message_add_float(msg, (float) lo_hires_val((lo_type)types[i], argv[i]));
            } else {
                lo_message_add_string(msg, (const char*) argv[i] );
            }
        }
        lo_send_message_from(spin.getContext()->lo_txAddr, spin.getContext()->lo_infoServ, path, msg);

    }

    bool eventScriptCalled = false;
    if ( s->s_type == REFERENCED_NODE ) {
        //printf("calling eventscript...\n");
        eventScriptCalled = dynamic_cast<ReferencedNode*>(s->s_thing)->callEventScript( theMethod, theArgs );
    }

    if ( !eventScriptCalled ) { // if an eventScript was hooked to theMethod, do not execute theMethod.  functionality taken over by script
        // invoke the method on the node, and if it doesn't work, then just forward
        // the message:
        if (!invokeMethod(classInstance, classType, theMethod, theArgs))
        {
            //std::cout << "Ignoring method '" << theMethod << "' for [" << s->s_name << "], but forwarding message anyway..." << std::endl;
            // HACK: TODO: fix this
            if (spin.getContext()->isServer())
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
                lo_send_message_from(spin.getContext()->lo_txAddr, spin.getContext()->lo_infoServ, path, msg);
            }
        }
    }

    //pthread_mutex_unlock(&pthreadLock);

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

    //pthread_mutex_lock(&pthreadLock);

    // note that args start at argv[1] now:
    if (theMethod=="debug")
        sceneManager->debug();
    else if (theMethod=="clear")
        sceneManager->clear();
    else if (theMethod=="clearUsers")
        sceneManager->clearUsers();
    else if (theMethod=="clearStates")
        sceneManager->clearStates();
    else if (theMethod=="userRefresh")
    {
    if (spin.getContext()->isServer())
    {
            SCENE_MSG("s", "userRefresh");
    }
    else {
        if (spin.userNode.valid()) spin.SceneMessage("sss", "createNode", spin.userNode->id->s_name, "UserNode", LO_ARGS_END);
    }
    }
    else if (theMethod=="refresh")
        sceneManager->refresh();
    else if (theMethod=="refreshSubscribed")
    {
    	if (spin.getContext()->isServer())
    	{
    		spinServerContext *server = dynamic_cast<spinServerContext*>(spin.getContext());
    		server->refreshSubscribed();
    	}
    }
    else if (theMethod=="getNodeList")
        sceneManager->sendNodeList("*");
    else if ((theMethod=="nodeList") && (argc>2))
    {
        for (int i=2; i<argc; i++)
        {
            if (strcmp((char*)argv[i],"NULL")!=0) sceneManager->createNode((char*)argv[i], (char*)argv[1]);
        }
    }
    else if ((theMethod=="stateList") && (argc>2))
    {
        for (int i=2; i<argc; i++)
        {
            if (strcmp((char*)argv[i],"NULL")!=0) sceneManager->createStateSet((char*)argv[i], (char*)argv[1]);
        }
    }
    else if ((theMethod=="exportScene") && (argc==3))
        sceneManager->exportScene((char*)argv[1], (char*)argv[2]);
    else if ((theMethod=="load") && (argc==2))
        sceneManager->loadXML((char*)argv[1]);
    else if ((theMethod=="save") && (argc==2))
        sceneManager->saveXML((char*)argv[1]);
    else if ((theMethod=="saveAll") && (argc==2))
        sceneManager->saveXML((char*)argv[1], true);
    else if ((theMethod=="saveUsers") && (argc==2))
        sceneManager->saveUsers((char*)argv[1]);
    else if ((theMethod=="createNode") && (argc==3))
        sceneManager->createNode((char*)argv[1], (char*)argv[2]);
    else if ((theMethod=="createStateSet") && (argc==3))
        sceneManager->createStateSet((char*)argv[1], (char*)argv[2]);
    else if ((theMethod=="createStateSet") && (argc==2))
        sceneManager->createStateSet((char*)argv[1]);
    else if ((theMethod=="deleteNode") && (argc==2))
        sceneManager->deleteNode((char*)argv[1]);
    else if ((theMethod=="deleteGraph") && (argc==2))
        sceneManager->deleteGraph((char*)argv[1]);
    else {
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

    //pthread_mutex_unlock(&pthreadLock);

    return 1;
}



int spinBaseContext::logCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
    spinLog *log = (spinLog*) user_data;

    *log << path << ' ' << types << ' ';
    for (int i=0; i<argc; i++)
    {
        if (lo_is_numerical_type((lo_type)types[i]))
        {
            *log << (float) lo_hires_val((lo_type)types[i], argv[i]);
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
    for (int i=0; i<argc; i++) {
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
