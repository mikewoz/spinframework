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
//  You should have received a copy of the Lesser GNU General Public License
//  along with SPIN Framework. If not, see <http://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

#include <string>
#include <iostream>
#include <pthread.h>

#include <osgDB/Registry>
#include <osgIntrospection/Type>

#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "asUtil.h"
#include "vessThreads.h"
#include "vessLog.h"

using namespace std;


pthread_mutex_t pthreadLock = PTHREAD_MUTEX_INITIALIZER;



vessThread::vessThread(vessMode initMode)
{

	// Load the SPIN library:
	osgDB::Registry *reg = osgDB::Registry::instance();
	osgDB::DynamicLibrary::loadLibrary(reg->createLibraryNameForNodeKit("libSPIN"));

	// Make sure that our OSG nodekit is loaded (by checking for existance of
	// the asReferenced node type):
	const osgIntrospection::Type &asReferencedType = osgIntrospection::Reflection::getType("asReferenced");
	if (!asReferencedType.isDefined())
	{
		std::cout << "ERROR: libSPIN was not found. Please check dynamic libraries. Could not start vessThread." << std::endl;
		exit(1);
	}

	running = false;
	id = "default";

	if (!this->setMode(initMode))
	{
		std::cout << "ERROR: Unknown mode for vessThreads" << std::endl;
		exit(1);
	}

	// default infoAddr:
	infoAddr = "224.0.0.1";
	infoPort = "54320";

	// override infoPort based on environment variable:
	char *infoPortStr = getenv("AS_INFOPORT");
	if (infoPortStr)
	{
	    string tmpStr = string(infoPortStr);
		infoAddr = tmpStr.substr(0,tmpStr.rfind(":"));
		infoPort = tmpStr.substr(tmpStr.find(":")+1);
	}

	lo_infoAddr = lo_address_new(infoAddr.c_str(), infoPort.c_str());

	if (isMulticastAddress(infoAddr))
	{
		lo_infoServ = lo_server_new_multicast(infoAddr.c_str(), NULL, oscParser_error);

	} else if (isBroadcastAddress(infoAddr))
	{
		lo_infoServ = lo_server_new(NULL, oscParser_error);
		int sock = lo_server_get_socket_fd(lo_infoServ);
		int sockopt = 1;
		setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &sockopt, sizeof(sockopt));

	} else {
		lo_infoServ = lo_server_new(NULL, oscParser_error);
	}

	std::cout << "  INFO channel: " << lo_address_get_url(lo_infoAddr) << std::endl;

}

vessThread::~vessThread()
{
	this->stop();
	usleep(100);

	if (lo_infoServ)
	{
		//lo_server_thread_stop(lo_infoServ);
		lo_server_free(lo_infoServ);
	}

	if (lo_txAddr)
	{
		lo_address_free(lo_txAddr);
	}

	if (lo_infoAddr)
	{
		lo_address_free(lo_infoAddr);
	}
}


bool vessThread::setMode(vessMode m)
{
	if (running)
	{
		stop();
	}

	switch (m)
	{
		case LISTENER_MODE:
			rxAddr = "224.0.0.1";
			rxPort = "54323";
			txAddr = "224.0.0.1";
			txPort = "54324";
			threadFunction = &vessListenerThread;
			break;
		case SERVER_MODE:
			rxAddr = getMyIPaddress();
			rxPort = "54324";
			txAddr = "224.0.0.1";
			txPort = "54323";
			threadFunction = &vessServerThread;
			break;
		default:
			return false;
	}

	this->mode = m;

	return true;
}

bool vessThread::start()
{
	lo_txAddr = lo_address_new(txAddr.c_str(), txPort.c_str());

	// create thread:
	if (pthread_attr_init(&pthreadAttr) < 0)
	{
		std::cout << "vessThread: could not prepare child thread" << std::endl;
		return false;
	}
	if (pthread_attr_setdetachstate(&pthreadAttr, PTHREAD_CREATE_DETACHED) < 0)
	{
		std::cout << "vessThread: could not prepare child thread" << std::endl;
		return false;
	}
	if (pthread_create( &pthreadID, &pthreadAttr, threadFunction, this) < 0)
	{
		std::cout << "vessThread: could not create new thread" << std::endl;
		return false;
	}

	//pthread_join(pthreadID, NULL); // if not DETACHED thread

	// wait until the thread gets into it's loop before returning:
	while (!running) usleep(10);

	return true;
}

void vessThread::stop()
{
	std::cout << "Stopping vessThread..." << std::endl;
	this->running = false;
}

void vessThread::sendNodeMessage(t_symbol *nodeSym, lo_message msg)
{
	if (isRunning())
	{
		std::string OSCpath = "/vess/" + id + "/" + nodeSym->s_name;

		// If this thread is a listener, then we need to send an OSC message to
		// the rxAddr of the vess server (unicast)
		if ( this->mode == LISTENER_MODE )
		{
			lo_send_message(lo_txAddr, OSCpath.c_str(), msg);
		}

		// if, however, this process acts as a server, we can optimize and send
		// directly to the OSC callback function:
		else asSceneManagerCallback_node(OSCpath.c_str(), lo_message_get_types(msg), lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)nodeSym);

	} else std::cout << "Error: tried to send message but vess is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}

void vessThread::sendNodeMessage(t_symbol *nodeSym, const char *types, ...)
{
	lo_message msg = lo_message_new();

	va_list ap;
	va_start(ap, types);

	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendNodeMessage(nodeSym, msg);
	} else {
		std::cout << "ERROR (vessThread::sendNodeMessage): " << err << std::endl;
	}

}


void vessThread::sendSceneMessage(lo_message msg)
{
	if (isRunning())
	{
		std::string OSCpath = "/vess/" + id;

		// If this thread is a listener, then we need to send an OSC message to
		// the rxAddr of the vess server (unicast)
		if ( this->mode == LISTENER_MODE )
		{
			lo_send_message(lo_txAddr, OSCpath.c_str(), msg);
		}

		// if, however, this process acts as a server, we can optimize and send
		// directly to the OSC callback function:
		else asSceneManagerCallback_admin(OSCpath.c_str(), lo_message_get_types(msg), lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)sceneManager);

	} else std::cout << "Error: tried to send message but vess is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}

void vessThread::sendSceneMessage(const char *types, ...)
{
	lo_message msg = lo_message_new();

	va_list ap;
	va_start(ap, types);

	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendSceneMessage(msg);
	} else {
		std::cout << "ERROR (vessThread::sendSceneMessage): " << err << std::endl;
	}
}

// *****************************************************************************

static void *vessListenerThread(void *arg)
{
	vessThread *vess = (vessThread*) arg;

	std::cout << "  vessThread started in Listener mode" << std::endl;

	vess->sceneManager = new asSceneManager(vess->id, vess->rxAddr, vess->rxPort);

	vess->running = true;
	while (vess->isRunning())
	{
		sleep(1);
		// do nothing (assume the app is doing updates - eg, in a draw loop)
	}

	// clean up:
	delete vess->sceneManager;

	pthread_exit(NULL);
}

static void *vessServerThread(void *arg)
{
	vessThread *vess = (vessThread*) arg;

	std::cout << "  vessThread started in Server mode" << std::endl;
	std::cout << "  broadcasting info messages on " << vess->txAddr << ", port: " << vess->infoPort << std::endl;

	vess->sceneManager = new asSceneManager(vess->id, vess->rxAddr, vess->rxPort);
	vess->sceneManager->setTXaddress(vess->txAddr, vess->txPort);

	string myIP = getMyIPaddress();
	osg::Timer_t lastTick = osg::Timer::instance()->tick();
	osg::Timer_t frameTick = lastTick;

	// convert ports to integers for sending:
	int i_rxPort, i_txPort;
	fromString<int>(i_rxPort, vess->rxPort);
	fromString<int>(i_txPort, vess->txPort);

	asSceneUpdateVisitor visitor;

	vess->running = true;
	while (vess->isRunning())
	{
		frameTick = osg::Timer::instance()->tick();
		if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
		{
			lo_send_from(vess->lo_infoAddr, vess->lo_infoServ, LO_TT_IMMEDIATE, "/ping/vess", "ssisi", vess->id.c_str(), myIP.c_str(), i_rxPort, vess->txAddr.c_str(), i_txPort);
			lastTick = frameTick;
		}

		pthread_mutex_lock(&pthreadLock);
		visitor.apply(*(vess->sceneManager->rootNode.get())); // only server should do this
		pthread_mutex_unlock(&pthreadLock);


		pthread_mutex_lock(&pthreadLock);
		vess->sceneManager->updateGraph();
		pthread_mutex_unlock(&pthreadLock);


		usleep(10);
	}

	// clean up:
	delete vess->sceneManager;

	pthread_exit(NULL);
}
