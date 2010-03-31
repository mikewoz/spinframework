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

#ifndef SPINCONTEXT_H_
#define SPINCONTEXT_H_

#include <boost/python.hpp>

#include "spinUtil.h"
#include "SceneManager.h"
#include "MediaManager.h"



/**
 * \brief A class to facilitate communication with SPIN from any application.
 *
 * An instance of this class should be included in any process that needs to
 * maintain scene state.
 *
 * By instantiating this class, we load the OSG nodekit for SPIN -- otherwise
 * known as libSPIN. Furthermore, all of the network handlers are automatically
 * generated in order to receive OSC messages from SPIN and update state.
 *
 * Note that this class can be instantiated in different modes, depending on
 * whether the process is to act as a server or a client.
 *
 */
class spinContext
{

    public:


        //spinContext(spinContextMode initMode=LISTENER_MODE);
        //virtual ~spinContext();

        // Meyers Singleton design pattern:
        static spinContext& Instance() {
            static spinContext spinInstance;
            return spinInstance;
        }

        enum spinContextMode { SERVER_MODE, LISTENER_MODE };

        bool isServer() { return (mode==SERVER_MODE); }
        bool isSlave() { return (mode!=SERVER_MODE); }


        bool setMode(spinContextMode m);

        /**
         * Starts the thread (pointed to by *threadFunction), according to the
         * current spinContextMode
         */
        virtual bool start();
        /**
         * Stops the currently running thread
         */
        virtual void stop();

        /**
         * Starts the thread that sends synchronization timecode (syncThread)
         */
        void startSync();

        /**
         * This method should be used to register a user for a listener-style
         * SPIN client. The user is definitively created and stored in the
         * current application context, even if a server is not running.
         */
        void registerUser(const char *id);

        /**
         * This sends a variable length message.
         *
         * IMPORTANT: the list must be terminated with LO_ARGS_END, or this call
         * will fail.  This is used to do simple error checking on the sizes of
         * parameters passed.
         */
        void InfoMessage(std::string OSCpath, const char *types, ...);
        void InfoMessage(std::string OSCpath, const char *types, va_list ap);
        void InfoMessage(std::string OSCpath, lo_message msg);

        void SceneMessage(const char *types, ...);
        void SceneMessage(const char *types, va_list ap);
        void SceneMessage(lo_message msg);

        void NodeMessage(const char *nodeId, const char *types, ...);
        void NodeMessage(const char *nodeId, const char *types, va_list ap);
        void NodeMessage(const char *nodeId, lo_message msg);


        virtual int sceneCallback(const char *types, lo_arg **argv, int argc);

        bool isRunning() { return running; }

        void setID(std::string s) { id = s; }

        /*
        void setRxAddr(std::string s) { rxAddr = s; }
        void setRxPort(std::string s) { rxPort = s; }
        void setTxAddr(std::string s) { txAddr = s; }
        void setTxPort(std::string s) { txPort = s; }
        */

        spinContextMode mode;

        osg::ref_ptr<UserNode> userNode;
        //t_symbol *user;

        std::string id;
        //std::string rxAddr, rxPort;
        //std::string txAddr, txPort;
        //std::string infoAddr, infoPort;

        lo_address lo_rxAddr, lo_txAddr;

        lo_address lo_infoAddr;
        //lo_server  lo_infoServ;
        lo_server_thread lo_infoServ;

        lo_address lo_syncAddr;
        lo_server_thread lo_syncServ;  // client only

        SceneManager *sceneManager;
        MediaManager *mediaManager;

        bool signalStop;
        bool running;

        /**
         * We store a function pointer in the class, which can be dynamically
         * swapped depending on spinContextMode (ie, different thread for server
         * mode versus listener mode).
         */
        void *(*threadFunction) (void*);

        bool initPython();
        bool execPython( const std::string& cmd );
        boost::python::object _pyMainModule;
        boost::python::object _pyNamespace;
        bool _pyInitialized;


    protected:

        std::string spinFolder;

    private:

        // singleton constructors & desctructor (hidden):
        spinContext();
        spinContext(spinContext const&); // copy constructor
        // hide the assignment operator, otherwise it would be possible to
        // assign the singleton spinContext to itself:
        spinContext& operator=(spinContext const&);
        ~spinContext();



        // pthread stuff
        pthread_t pthreadID; // id of child thread
        pthread_attr_t pthreadAttr;

        pthread_t syncThreadID; // id of sync thread
        pthread_attr_t syncthreadAttr;

};



/**
 * The spinListenerThread is a simple thread that starts a sceneManager and
 * listens to incoming SPIN messages. It does NOT re-transmit those messages,
 * and it does NOT perform an update traversal.
 */
static void *spinListenerThread(void *arg);

/**
 * The spinServerThread is mainly differentiated from a listener thread by the
 * fact taht all received messages are re-transmit upon processing. This allows
 * all clients to keep up-to-date whenever new state information is received by
 * the server.
 *
 * Additionally, the spinServerThread will periodically broadcast a ping on
 * infoport, and will perform an update traversal on the scene graph for any
 * nodes who need periodic (scheduled) processing.
 */
static void *spinServerThread(void *arg);

/**
 * The syncThread just sends timecode on an independent multicast UDP port
 */
static void *syncThread(void *arg);


static int spinContext_sceneCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int spinContext_infoCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);
static int spinContext_syncCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data);


#endif
