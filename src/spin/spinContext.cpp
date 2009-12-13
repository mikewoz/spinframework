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


#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

#include "spinUtil.h"
#include "spinContext.h"
#include "nodeVisitors.h"

using namespace std;


pthread_mutex_t pthreadLock = PTHREAD_MUTEX_INITIALIZER;





static void sigHandler(int signum)
{
	std::cout << " Caught signal: " << signum << std::endl;
	
	spinContext &spin = spinContext::Instance();
	
	spin.stop();

	// TODO: this doesn't work; something else must be holding onto a reference
	// for the user (viewer?). We should figure out how to proper destroy him.
	if (spin.user.valid()) spin.user.release();

	//exit(0);
}



//spinContext::spinContext(spinContextMode initMode)
spinContext::spinContext()
{

	spinContextMode  initMode = spinContext::LISTENER_MODE;
	
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
	
    
    signal(SIGINT, sigHandler);


    
    
    
	
	// Make sure that our OSG nodekit	 is loaded (by checking for existance of
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
    }
    catch (osgIntrospection::Exception & ex)
	{    	
		std::cout << "ERROR: " << ex.what() << ". This is likely a dynamic library problem. Make sure that libSPINwrappers exists and can be found." << std::endl;
		exit(1);
	}
	

	running = false;
	id = "default";

	if (!this->setMode(initMode))
	{
		std::cout << "ERROR: Unknown mode for spinContext" << std::endl;
		exit(1);
	}

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
		lo_infoServ = lo_server_thread_new_multicast(infoAddr.c_str(), infoPort.c_str(), oscParser_error);

	} else if (isBroadcastAddress(infoAddr))
	{
		lo_infoServ = lo_server_thread_new(infoPort.c_str(), oscParser_error);
		int sock = lo_server_get_socket_fd(lo_server_thread_get_server(lo_infoServ));
		int sockopt = 1;
		setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &sockopt, sizeof(sockopt));

	} else {
		lo_infoServ = lo_server_thread_new(infoPort.c_str(), oscParser_error);
	}

	// info channel callback (receives pings from client apps):
	lo_server_thread_add_method(lo_infoServ, NULL, NULL, spinContext_infoCallback, this);
	
	lo_server_thread_start(lo_infoServ);
	

	std::cout << "  INFO channel: " << lo_address_get_url(lo_infoAddr) << std::endl;
}

spinContext::~spinContext()
{
	this->stop();

	if (lo_infoServ)
	{
		lo_server_thread_stop(lo_infoServ);
		lo_server_thread_free(lo_infoServ);
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



bool spinContext::setMode(spinContextMode m)
{
	if (running)
	{
		stop();
	}

	if (m==SERVER_MODE)
	{
		rxAddr = getMyIPaddress();
		rxPort = "54324";
		txAddr = "224.0.0.1";
		txPort = "54323";
		threadFunction = &spinServerThread;
	}
	
	else
	{
		rxAddr = "224.0.0.1";
		rxPort = "54323";
		txAddr = "224.0.0.1";
		txPort = "54324";
		threadFunction = &spinListenerThread;
	}

	this->mode = m;

	return true;
}

bool spinContext::start()
{
	lo_txAddr = lo_address_new(txAddr.c_str(), txPort.c_str());

	signalStop = false;
	
	// create thread:
	if (pthread_attr_init(&pthreadAttr) < 0)
	{
		std::cout << "spinContext: could not prepare child thread" << std::endl;
		return false;
	}
	if (pthread_attr_setdetachstate(&pthreadAttr, PTHREAD_CREATE_DETACHED) < 0)
	{
		std::cout << "spinContext: could not prepare child thread" << std::endl;
		return false;
	}
	if (pthread_create( &pthreadID, &pthreadAttr, threadFunction, this) < 0)
	{
		std::cout << "spinContext: could not create new thread" << std::endl;
		return false;
	}

	//pthread_join(pthreadID, NULL); // if not DETACHED thread

	// wait until the thread gets into it's loop before returning:
	while (!running) usleep(10);

	return true;
}

void spinContext::stop()
{
	if (isRunning())
	{
		std::cout << "Stopping spinContext..." << std::endl;
		signalStop = true;
		while (running) usleep(10);
	}
}


void spinContext::registerUser (const char *id)
{
	if (isRunning())
	{
		// Here, we (definitively) create a UserNode and store it in this
		// client's sceneManager.
		user = dynamic_cast<UserNode*>(sceneManager->getOrCreateNode(id, "UserNode"));

		// We then send a message to the server to create the node. If the
		// server doesn't exist yet, it doesn't really matter, since we are
		// guaranteed to have a local instance. Then, once the server starts,
		// it will send a 'userRefresh' method that will inform it of this node
		// (see the spinContext_sceneCallback method)
		
		sendSceneMessage("sss", "createNode", user->id->s_name, "UserNode", LO_ARGS_END);

		std::cout << "  Registered user '" << user->id->s_name << "'" << std::endl;
	}

	if (!user.valid())
	{
		std::cout << "ERROR: Could not registerUser '" << id <<	"' because SPIN is not running." << std::endl;
	}
}


void spinContext::sendInfoMessage(std::string OSCpath, const char *types, ...)
{
	va_list ap;
	va_start(ap, types);
	sendInfoMessage(OSCpath, types, ap);

	/*
	va_start(ap, types);

	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendInfoMessage(OSCpath, msg);
	} else {
		std::cout << "ERROR (spinContext::sendInfoMessage): " << err << std::endl;
	}
	*/
}


void spinContext::sendInfoMessage(std::string OSCpath, const char *types, va_list ap)
{
	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendInfoMessage(OSCpath, msg);
	} else {
		std::cout << "ERROR (spinContext::sendInfoMessage): " << err << std::endl;
	}
}

void spinContext::sendInfoMessage(std::string OSCpath, lo_message msg)
{
	lo_send_message_from(lo_infoAddr, lo_server_thread_get_server(lo_infoServ), OSCpath.c_str(), msg);

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}



/*
void spinContext::sendNodeMessage(t_symbol *nodeSym, const char *types, ...)
{
	lo_message msg = lo_message_new();

	va_list ap;
	va_start(ap, types);

	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendNodeMessage(nodeSym, msg);
	} else {
		std::cout << "ERROR (spinContext::sendNodeMessage): " << err << std::endl;
	}

}
*/

void spinContext::sendNodeMessage(const char *nodeId, const char *types, ...)
{
	va_list ap;
	va_start(ap, types);
	sendNodeMessage(nodeId, types, ap);
}

void spinContext::sendNodeMessage(const char *nodeId, const char *types, va_list ap)
{
	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendNodeMessage(nodeId, msg);
	} else {
		std::cout << "ERROR (spinContext::sendNodeMessage): " << err << std::endl;
	}
}


void spinContext::sendNodeMessage(const char *nodeId, lo_message msg)
{
	if (isRunning())
	{
		std::string OSCpath = "/SPIN/" + id + "/" + nodeId;

		// If this thread is a listener, then we need to send an OSC message to
		// the rxAddr of the spinServer (unicast)
		if ( this->mode != SERVER_MODE )
		{
			lo_send_message(lo_txAddr, OSCpath.c_str(), msg);
		}

		// if, however, this process acts as a server, we can optimize and send
		// directly to the OSC callback function:
		else SceneManagerCallback_node(OSCpath.c_str(), lo_message_get_types(msg), lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)gensym(nodeId));

	} else std::cout << "Error: tried to send message but SPIN is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}



/*
void sendNodeMessage(const char *nodeName, const char *types, ...)
{
	lo_message msg = lo_message_new();

	va_list ap;
	va_start(ap, types);

	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendNodeMessage(gensym(nodeName), msg);
	} else {
		std::cout << "ERROR (spinContext::sendNodeMessage): " << err << std::endl;
	}	
}
*/

void spinContext::sendSceneMessage(const char *types, ...)
{
	va_list ap;
	va_start(ap, types);
	sendSceneMessage(types, ap);
	
	/*
	va_start(ap, types);

	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);

	if (!err)
	{
		sendSceneMessage(msg);
	} else {
		std::cout << "ERROR (spinContext::sendSceneMessage): " << err << std::endl;
	}
	*/
}

void spinContext::sendSceneMessage(const char *types, va_list ap)
{
	lo_message msg = lo_message_new();
	int err = lo_message_add_varargs(msg, types, ap);
	if (!err)
	{
		sendSceneMessage(msg);
	} else {
		std::cout << "ERROR (spinContext::sendSceneMessage): " << err << std::endl;
	}
}

void spinContext::sendSceneMessage(lo_message msg)
{
	if (isRunning())
	{
		std::string OSCpath = "/SPIN/" + id;

		// If this thread is a listener, then we need to send an OSC message to
		// the rxAddr of the spinServer (unicast)
		if ( this->mode != SERVER_MODE )
		{
			lo_send_message(lo_txAddr, OSCpath.c_str(), msg);
		}

		// if, however, this process acts as a server, we can optimize and send
		// directly to the OSC callback function:
		else SceneManagerCallback_admin(OSCpath.c_str(), lo_message_get_types(msg), lo_message_get_argv(msg), lo_message_get_argc(msg), NULL, (void*)sceneManager);

	} else std::cout << "Error: tried to send message but SPIN is not running" << std::endl;

    // Let's free the message after (not sure if this is necessary):
    lo_message_free(msg);
}

int spinContext::sceneCallback(const char *types, lo_arg **argv, int argc)
{

	std::cout << "Got sceneCallback method call, with types=" << types << std::endl;

	for (int i=0; i<argc; i++)
	{
		if (lo_is_numerical_type((lo_type)types[i]))
		{
			std::cout << i << ": " << lo_hires_val((lo_type)types[i], argv[i]) << std::endl;
		}
		else {
			std::cout << i << ": " << (const char*) argv[i] << std::endl;
		}
	}
	
	/*
    for (int i=0; i<argc; i++)
    {
        if (types[i]=='s')
        {
            std::cout << i << ": " << va_arg(ap, const char*) <<  std::endl;
        }
        else if (types[i]=='i')
        {
        	std::cout << i << ": " << va_arg(ap, int) <<  std::endl;
        }
        else if (types[i]=='f' || types[i]=='d')
        {
        	std::cout << i << ": " << va_arg(ap, double) <<  std::endl;
        }
    }
    */
	
    return 1;
}


// *****************************************************************************

static void *spinListenerThread(void *arg)
{
	spinContext *spin = (spinContext*) arg;

	std::cout << "  spinContext started in Listener mode" << std::endl;

	spin->sceneManager = new SceneManager(spin->id, spin->rxAddr, spin->rxPort);

	// register our special scene callback:
	lo_server_thread_add_method(spin->sceneManager->rxServ, ("/SPIN/" + spin->id).c_str(), NULL, spinContext_sceneCallback, NULL);

	spin->running = true;
	while (!spin->signalStop)
	{
		sleep(1);
		// do nothing (assume the app is doing updates - eg, in a draw loop)
	}
	spin->running = false;
	
	// clean up:
	delete spin->sceneManager;
	
	pthread_exit(NULL);
}

static void *spinServerThread(void *arg)
{
	//spinContext *spin = (spinContext*) arg;
	spinContext &spin = spinContext::Instance();

	std::cout << "  spinContext started in Server mode" << std::endl;
	std::cout << "  broadcasting info messages on " << spin.txAddr << ", port: " << spin.infoPort << std::endl;

	spin.sceneManager = new SceneManager(spin.id, spin.rxAddr, spin.rxPort);
	spin.sceneManager->setTXaddress(spin.txAddr, spin.txPort);

	
	// create log filename based on datetime:
	time_t t = time(NULL);
	tm* tmp = localtime(&t);
	char dateString[128];
	strftime(dateString, sizeof(dateString), "%Y-%m-%d_%H-%M-%S", tmp);

	// start spinLog, and disable console printing:
	string logFilename = SPIN_DIRECTORY + "/log/spinLog_" + string(dateString) + ".txt";
	spinLog log(logFilename.c_str());
	log.enable_cout(false);
	spin.sceneManager->setLog(log);
	

	string myIP = getMyIPaddress();
	osg::Timer_t lastTick = osg::Timer::instance()->tick();
	osg::Timer_t frameTick = lastTick;

	// convert ports to integers for sending:
	int i_rxPort, i_txPort;
	fromString<int>(i_rxPort, spin.rxPort);
	fromString<int>(i_txPort, spin.txPort);

	UpdateSceneVisitor visitor;

	
	//lo_server_thread_add_method(spin->sceneManager->rxServ, NULL, NULL, sceneCallback, spin);
	

	spin.running = true;
	while (!(spin.signalStop))
	{
		frameTick = osg::Timer::instance()->tick();
		if (osg::Timer::instance()->delta_s(lastTick,frameTick) > 5) // every 5 seconds
		{
			spin.sendInfoMessage("/ping/SPIN", "ssisi", spin.id.c_str(), myIP.c_str(), i_rxPort, spin.txAddr.c_str(), i_txPort, LO_ARGS_END);
			//lo_send_from(spin->lo_infoAddr, spin->lo_infoServ, LO_TT_IMMEDIATE, "/ping/SPIN", "ssisi", spin->id.c_str(), myIP.c_str(), i_rxPort, spin->txAddr.c_str(), i_txPort);
			lastTick = frameTick;
		}

		pthread_mutex_lock(&pthreadLock);
		visitor.apply(*(spin.sceneManager->rootNode.get())); // only server should do this
		pthread_mutex_unlock(&pthreadLock);

		usleep(10);
	}
	spin.running = false;


	// clean up:
	delete spin.sceneManager;

	pthread_exit(NULL);
}


static int spinContext_sceneCallback(const char *path, const char *types, lo_arg **argv, int argc, lo_message msg, void *user_data)
{
	//std::cout << "got to spinContext_sceneCallback function" << std::endl;
	
	spinContext &spin = spinContext::Instance();
	
	// make sure there is at least one argument (ie, a method to call):
	if (!argc) return 0;

	// get the method (argv[0]):
	string theMethod;
	if (lo_is_string_type((lo_type)types[0]))
	{
		theMethod = string((char *)argv[0]);
	}
	else return 0;
	
	// bundle all other arguments
	vector<float> floatArgs;
	vector<const char*> stringArgs;
	for (int i=1; i<argc; i++)
	{
		if (lo_is_numerical_type((lo_type)types[i]))
		{
			floatArgs.push_back( (float) lo_hires_val((lo_type)types[i], argv[i]) );
		} else {
			stringArgs.push_back( (const char*) argv[i] );
		}
	}

	// now, here are some special messages that we need to look for:
	
	if (theMethod=="userRefresh")
	{
		if (spin.user.valid())
		{
			spin.sendSceneMessage("sss", "createNode", spin.user->id->s_name, "UserNode", LO_ARGS_END);
		}
	}
	
	//return spin.sceneCallback(types, argv, argc);
	
}

static int spinContext_infoCallback(const char *path, const char *types, lo_arg **argv, int argc, void *data, void *user_data)
{
	spinContext *spin = (spinContext*) user_data;
	
	if (!spin) return 0;
	
	/*
	std::cout << "************ infoChannelCallback() got message: " << path << std::endl;
	
	printf("************ infoChannelCallback() got message: %s\n", (char*)path);
	printf("user_data: %s\n", (char*) user_data);
	for (int i=0; i<argc; i++) {
		printf("arg %d '%c' ", i, types[i]);
    	lo_arg_pp((lo_type) types[i], argv[i]);
    	printf("\n");
	}
	printf("\n");
	fflush(stdout);
	 */
	
	if (spin->mode == spinContext::SERVER_MODE)
	{
		// TODO: monitor /ping/user messages, keep timeout handlers, 
		
	}
	
}
