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

#ifndef MAINPAGE_H_
#define MAINPAGE_H_

/** @mainpage SPIN (Spatial Interaction) Framework
*
*
* @section intro SPIN Framework API Documentation
*
* This documentation is generated from source using Doxygen.
* 
* For class API documentation, see the
* @htmlonly <a href="annotated.html">Class List</a>@endhtmlonly
*
* For the full list of OSC messages accepted by SPIN, see the
* @htmlonly <a href="oscprotocol.html">OSC Protocol Documentation</a>@endhtmlonly
* 
* we can type stuff here
*/

/** @page oscprotocol OSC Protocol
 */

/** @page commandlineargs Command Line Arguments

These are the command line arguements for spinserver:
 
 -h or -help brings up a menu showing essentially the contents of this list (this command does not open the spinserver).
 --disable-auto-cleanup disables the auto-cleanup of user nodes if they stop sending ping messages. Normally participants in a SPIN scene ping (send a message to the server to test speed) every 20 seconds. If they do not for 60 seconds, spinserver removes them from the session and they must re-connect. Use this option to prevent users from getting kicked off if the network conditions are not optimal.
 --disable-discovery disables multicast discovery services. This essentially turns off the automatic connection to network channels, requiring the user to connect manually. This could be useful in a situation where several spinservers are operating on the same network.
 --http-port <port> sets a local port from which messages can be sent to your spinserver. This is used for building controls for SPIN into a webpage. The default is port 9980.
 --recv-tcp-msg <port> sets the port where spinserver listens to TCP messages. Subscription requests from client computers and from your own spinviewer will come here by default. It is also possible to send scene events here for increased reliability, though by default they use UDP and a different port. The default is port 54322.
 --recv-udp-msg <host> <port> sets the port address where spinserver listens for UDP messages from clients. The majority of messages are sent this way by default, the creation and manipulation of objects in the scene, for example.
 This argument may be repeated to assign multiple hosts and ports. The default values are host 239.0.0.1 and port 54323).
 --scene-id <name> to change the name of the session from default to something else. In this way it is possible to run multiple SPIN sessions on the same network.
 --send-udp-msg <host> <port> sets the address/port for UDP multicast of scene events, or this argument may be repeated for several unicast connections. This is used to change the port where spinserver listens for its commands.
 (default: 239.0.0.1 54324)
 --send-udp-sync <host> <port> sets the address/port for timecode (sync) messages. This allows all connected machines to be synchronized in time. In this way, the contents of the scene will be roughly the same for all participants at the same time. However, it does not account for network lag.
 --spatosc <translator> <URL> enables SPIN’s internal SpatOSC scene. Example: --spatosc BasicTranslator osc.tcp://localhost:18033
 --ttl <number> sets the TTL (time to live) for multicast packets in order to hop across routers. (default 1).
 
 --version displays the version number and exits. 
 
*/



#endif
