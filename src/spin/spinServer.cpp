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

#include <osgViewer/CompositeViewer>
#include <osg/ArgumentParser>
#include <osgDB/ReadFile>
#include <osg/Timer>

#include "SceneManager.h"
#include "spinApp.h"
#include "spinServerContext.h"
#include "spinLog.h"
#include "config.h"

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
int main(int argc, char **argv)
{
	using namespace spin;
	//spinServerContext *server = new spinServerContext();
	spinServerContext server;

	// *************************************************************************
	// ARGUMENTS::

	osg::ArgumentParser arguments(&argc,argv);

    /*
	std::string sceneID = spinApp::Instance().getSceneID();
    
    // UDP from clients
    std::string recv_udp_msg_addr =  spin::spin_defaults::MULTICAST_GROUP;
    std::string recv_udp_msg_port =  spin::spin_defaults::CLIENT_RX_UDP_PORT;

    // UDP multicast (or you can specify this arg multiple times for
    // multi-unicast)
    std::string send_udp_msg_addr =  spin::spin_defaults::MULTICAST_GROUP;
    std::string send_udp_msg_port =  spin::spin_defaults::SERVER_RX_UDP_PORT;

    // TCP port where we listen for subscription requests from clients
    // (note, clients may also send scene events to this port, if they desire
    // reliability)
    std::string recv_tcp_msg_port =  spin::spin_defaults::CLIENT_TCP_PORT;

    // Port for timecode (SYNC) messages
    std::string send_udp_sync_addr =  spin::spin_defaults::MULTICAST_GROUP;
    std::string send_udp_sync_port =  spin::spin_defaults::SYNC_UDP_PORT;

    int ttl=1;
*/
	// set up the usage document, which a user can acess with -h or --help
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is a server for the SPIN Framework.");
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] <scene-to-load.xml>");
	arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
	arguments.getApplicationUsage()->addCommandLineOption("--version", "Display the version number and exit.");

    server.addCommandLineOptions(&arguments);

    /*
    arguments.getApplicationUsage()->addCommandLineOption("--scene-id <uniqueID>", "Specify a unique ID for this scene (Default: default)"); 
    arguments.getApplicationUsage()->addCommandLineOption("--recv-udp-msg <host> <port>", "Set the address/port for listening to UDP messages from clients. This argument may be repeated for multiple multicast groups and/or ports (Default: " + recv_udp_msg_address + " " + recv_udp_msg_port + ")");
    arguments.getApplicationUsage()->addCommandLineOption("--send-udp-msg <host> <port>", "Set the address/port for UDP multicast of scene events, or this argument may be repeated for several unicast connections (Default: " + send_udp_msg_address + " " + send_udp_msg_port + ")");
    arguments.getApplicationUsage()->addCommandLineOption("--recv-tcp-msg <port>", "Set the port where we listen for subscription requests from clients. Clients may also send scene events to this port is they desire reliability. (Default: " + recv_tcp_msg_port + ")");
    arguments.getApplicationUsage()->addCommandLineOption("--send-udp-sync <host> <port>", "Set the address/port for timecode (sync) messages (Default: " + send_udp_sync_address + " " + send_udp_sync_port + ")");
    arguments.getApplicationUsage()->addCommandLineOption("--ttl <number>", "Set the TTL (time to live) for multicast packets in order to hop across routers (Default: 1)");
*/

	// PARSE ARGS:

	// if user request help or version write it out to cout and quit.
	if (arguments.read("-h") || arguments.read("--help"))
	{
		arguments.getApplicationUsage()->write(std::cout);
		return 0;
	}
    if (arguments.read("--version"))
    {
        std::cout << VERSION << std::endl;
        return 0;
    }

    server.parseCommandLineOptions(&arguments);

    /*
    osg::ArgumentParser::Parameter param_spinID(sceneID);
	arguments.read("--scene-id", param_spinID);
	spinApp::Instance().setSceneID(sceneID);

	bool passed_addrs = false;
    while (arguments.read("--send-udp-msg", send_udp_msg_addr, send_udp_msg_port)) {
		if (!passed_addrs) server->lo_txAddrs_.clear();
		server->lo_txAddrs_.push_back(lo_address_new(send_udp_msg_addr.c_str(), send_udp_msg_port.c_str()));
		passed_addrs = true;
	}

	passed_addrs = false;
	while (arguments.read("--recv-udp-msg", recv_udp_msg_addr, recv_udp_msg_port)) {
		if (!passed_addrs) server->lo_rxAddrs_.clear();
		server->lo_rxAddrs_.push_back(lo_address_new(recv_udp_msg_addr.c_str(), recv_udp_msg_port.c_str()));
		passed_addrs = true;
	}

	arguments.read("--recv-tcp-msg", server->tcpPort_);

	while (arguments.read("--send-udp-sync", send_udp_sync_addr, send_udp_sync_port)) {
		server->lo_syncAddr = lo_address_new(send_udp_sync_addr.c_str(), send_udp_sync_port.c_str());
	}

    while (arguments.read("--ttl", ttl)) {
        server->setTTL(ttl);
    }
	*/

	// any option left unread are converted into errors to write out later
	arguments.reportRemainingOptionsAsUnrecognized();

	
	// *************************************************************************
	// remaining arguments are assumed to be scene elements to be loaded (.xml)
	
	std::vector<std::string>::iterator arg;
	for (int i=1; i<argc; i++) //arg=arguments.begin(); arg!=arguments.end(); ++arg)
	{
		if (arguments[i][0]!='-')
		{
			// not an option so assume string is a filename
			spinApp::Instance().sceneManager->loadXML(arguments[i]);
		} else i++;
	}
	 	
	// any option left unread are converted into errors
	//arguments.reportRemainingOptionsAsUnrecognized();
	
	if (arguments.errors())
	{
		arguments.writeErrorMessages(std::cout);
		return 1;
	}

	// *************************************************************************
	// start spin:
	server.start();

	// *************************************************************************
	// send a userRefresh message:
    SCENE_MSG("s", "userRefresh");
	
	// *************************************************************************
	// loop:
    std::cout << "\nSTARTING spinserver..." << std::endl;
    try {	
        while (server.isRunning())
        {
            sleep(1);
            // loop until a quit message is received (TODO)
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Got exception " << e.what() << std::endl;
        return 1;
    }

    usleep(100);
    std::cout << "spinserver exited normally." << std::endl;

    return 0;
}
