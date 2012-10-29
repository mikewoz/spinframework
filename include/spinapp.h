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

#ifndef __spinApp_H
#define __spinApp_H

#include "config.h"

#ifndef DISABLE_PYTHON
//#include "Python.h"
//#include <Python.h>
#include <boost/python.hpp>

#endif

#include <stdarg.h>
#include <osg/Timer>
#include <lo/lo_types.h>
#include <osg/ref_ptr>

#include "spinutil.h"

namespace spatosc
{
class Scene;
}

namespace spin
{

class spinBaseContext;
class SceneManager;
class ReferencedNode;
class UserNode;

#define SPIN_ARGS_END LO_ARGS_END

/**
 * \brief A singleton class to facilitate communication with SPIN.
 *
 * By instantiating this class, we load the SPIN library, 
 *
 *
 * IMPORTANT: The message API in this class (ie, InfoMessage, NodeMessage,
 * SceneMessage, etc.) is intended as an end-user API for creating C++ 
 * applications linking with SPIN (particularly client-side applications).
 * These messages should NOT be used from within the core of SPIN itself, since
 * it is easy to create infinite loops if not used carefully. For example,
 * NodeMessage() on a sever will cause a message to be sent to all clients, and
 * if the client receiver also uses NodeMessage to send to the server, then a
 * loop will be formed.
 */
class spinApp
{
    public:
        /**
         * Meyers Singleton design pattern
         */
        static spinApp& Instance();

        void setContext(spinBaseContext *c);
        spinBaseContext *getContext() { return context_; }

        void createScene();
        void destroyScene();

        /**
         * This method should be used to register a user for a listener-style
         * SPIN client. The user is definitively created and stored in the
         * current application context, even if a server is not running.
         */
        void registerUser();


        /**
         * Send a variable length message on the INFO channel. Note: the list
         * must be terminated with SPIN_ARGS_END, or this call will fail.
         */
        void InfoMessage(const std::string &OSCpath, const char *types, ...);
        void InfoMessage(const std::string &OSCpath, lo_message msg);

        /**
         * Send a variable length message to a particular node. Note: the list
         * must be terminated with SPIN_ARGS_END, or this call will fail.
         */
        void NodeMessage(const char *nodeId, const char *types, ...);
        void NodeMessage(const char *nodeId, lo_message msg);

        void BroadcastNodeMessage(const char *nodeId, const char *types, ...);
    
        /**
         * Send a variable length message to the SCENE namespace. Note: the list
         * must be terminated with SPIN_ARGS_END, or this call will fail.
         */
        void SceneMessage(const char *types, ...);
        void SceneMessage(lo_message msg);
        void BroadcastSceneMessage(const char *types, ...);


        void BroadcastMessage(std::string OSCpath, lo_message msg);
    
        void NodeBundle(std::string nodeId, std::vector<lo_message> msgs, lo_address addr = 0);
        void SceneBundle(std::vector<lo_message> msgs, lo_address addr = 0);

private:
        // helper functions for dealing with va_lists
        //void InfoMessage(const std::string &OSCpath, const char *types, va_list ap);
        //void SceneMessage(const char *types, va_list ap);
        //void NodeMessage(const char *nodeId, const char *types, va_list ap);
    
public:    
    
        void setSceneID(const std::string &s) { sceneID = s; }
        std::string getSceneID() const { return sceneID; }

        void setSyncStart(osg::Timer_t t) { _syncStartTick = t; }
        osg::Timer_t getSyncStart() const { return _syncStartTick; }

        /**
         * initializes the embedded python interpreter
         */
        bool initPython();
        
        /**
         * Runs some Python code and returns its success or not.
         */
        bool execPython( const std::string& cmd );
        /** 
         * Returns a string containing the most recent Python exception, if any.
         */
        std::string getCurrentPyException();
    
#ifndef DISABLE_PYTHON
        boost::python::object _pyMainModule;
        boost::python::object _pyNamespace;
#endif
        bool _pyInitialized;

        osg::ref_ptr<UserNode> userNode;

#ifdef WITH_SPATOSC
        spatosc::Scene *audioScene;
#endif

        /**
         * A spinApp instance may or may not have an embedded audio renderer
         * using SpatOSC. For instance, there may be a renderer on the server
         * side or viewer side, but probably not both.
         */
        bool hasAudioRenderer;

        SceneManager *sceneManager_;
        
        void setUserID(const std::string &id) { userID_ = id; }
        std::string getUserID() const { return userID_; }

    private:
    
        std::string userID_;
        void sendBundle(const std::string &OSCpath, std::vector<lo_message> msgs, lo_address txAddr = 0);

        // singleton constructors & desctructor (hidden):
        spinApp();
        spinApp(spinApp const&); // copy constructor
        // hide the assignment operator, otherwise it would be possible to
        // assign the singleton spinApp to itself:
        spinApp& operator=(spinApp const&);
        ~spinApp();

        std::string sceneID;

        osg::Timer_t _syncStartTick;

        spinBaseContext *context_;
};

} // end of namespace spin


// Internal server-side MACROS for sending messages. Clients should NEVER use
// these macros, and should rather use spinApp message methods (eg, NodeMessage,
// SceneMessage, etc). But just in case, the macros always check that spinApp
// is operating in server mode


/*
#define SCENE_MSG(types, ...) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::vector<lo_address>::iterator addrIter=spinApp::Instance().getContext()->lo_txAddrs_.begin(); addrIter != spinApp::Instance().getContext()->lo_txAddrs_.end(); ++addrIter) \
            lo_send((*addrIter), \
            ("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), \
            types, ##__VA_ARGS__, SPIN_ARGS_END)

#define SCENE_MSG_TCP(types, ...) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::map<std::string,lo_address>::iterator addrIter=dynamic_cast<spinServerContext*>(spinApp::Instance().getContext())->tcpClientAddrs_.begin(); addrIter != dynamic_cast<spinServerContext*>(spinApp::Instance().getContext())->tcpClientAddrs_.end(); ++addrIter) \
            lo_send(addrIter->second, \
            ("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), \
            types, ##__VA_ARGS__, SPIN_ARGS_END)

#define SCENE_LO_MSG(msg) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::vector<lo_address>::iterator addrIter=spinApp::Instance().getContext()->lo_txAddrs_.begin(); addrIter != spinApp::Instance().getContext()->lo_txAddrs_.end(); ++addrIter) \
            lo_send_message((*addrIter), \
            ("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), msg)

#define SCENE_LO_MSG_TCP(msg, addr) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
        lo_send_message((addr), \
            ("/SPIN/" + spinApp::Instance().getSceneID()).c_str(), \
            msg)

#define NODE_MSG(pNode, types, ...) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::vector<lo_address>::iterator addrIter=spinApp::Instance().getContext()->lo_txAddrs_.begin(); addrIter != spinApp::Instance().getContext()->lo_txAddrs_.end(); ++addrIter) \
            lo_send((*addrIter), \
            ("/SPIN/" + spinApp::Instance().getSceneID() + "/" + \
            pNode->getID()).c_str(), \
            types, ##__VA_ARGS__, SPIN_ARGS_END)

#define NODE_MSG_TCP(pNode, types, ...) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::map<std::string,lo_address>::iterator addrIter=dynamic_cast<spinServerContext*>(spinApp::Instance().getContext())->tcpClientAddrs_.begin(); addrIter != dynamic_cast<spinServerContext*>(spinApp::Instance().getContext())->tcpClientAddrs_.end(); ++addrIter) \
            lo_send(addrIter->second, \
            ("/SPIN/" + spinApp::Instance().getSceneID() + "/" + \
            pNode->getID()).c_str(), \
            types, ##__VA_ARGS__, SPIN_ARGS_END)

#define NODE_LO_MSG(s, pNode, msg) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::vector<lo_address>::iterator addrIter=spinApp::Instance().getContext()->lo_txAddrs_.begin(); addrIter != spinApp::Instance().getContext()->lo_txAddrs_.end(); ++addrIter) \
            lo_send_message((*addrIter), \
            ("/SPIN/" + spinApp::Instance().getSceneID() + "/" + \
            pNode->getID()).c_str(), msg)

#define NODE_LO_MSG_TCP(s, pNode, msg) \
    if (spinApp::Instance().getContext() && \
        (spinApp::Instance().getContext()->isRunning()) && \
        (spinApp::Instance().getContext()->isServer()) ) \
            for (std::map<std::string,lo_address>::iterator addrIter=dynamic_cast<spinServerContext*>(spinApp::Instance().getContext())->tcpClientAddrs_.begin(); addrIter != dynamic_cast<spinServerContext*>(spinApp::Instance().getContext())->tcpClientAddrs_.end(); ++addrIter) \
            lo_send_message(addrIter->second, \
            ("/SPIN/" + spinApp::Instance().getSceneID() + "/" + \
            pNode->getID()).c_str(), msg)
*/

// backwards compatibility (TODO: replace all BROADCAST messages with NODE_MSG)
//#define BROADCAST(pNode, types, ...) NODE_MSG(pNode, types, ##__VA_ARGS__)
#define BROADCAST(pNode, types, ...) spinApp::Instance().BroadcastNodeMessage(pNode->getID().c_str(), types, ##__VA_ARGS__, SPIN_ARGS_END)



#endif
