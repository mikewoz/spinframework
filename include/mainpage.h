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
* This documentation is generated from source using Doxygen.
*
* @htmlonly <br><br> @endhtmlonly
* 
* @section manual SPIN Manual
*
* @htmlonly <a href="introtospin.html">Introduction to SPIN</a><br>
* <a href="basicnavigation.html">Basic Navigation</a><br>
* <a href="console.html">Console Commands</a><br><br>@endhtmlonly
* 
*
* @section API SPIN Framework API Documentation
*
* For class API documentation, see the
* @htmlonly <a href="annotated.html">Class List</a>@endhtmlonly
*
* For the full list of OSC messages accepted by SPIN, see the
* @htmlonly <a href="oscprotocol.html">OSC Protocol Documentation</a>@endhtmlonly
*/

/** @page oscprotocol OSC Protocol
 */

/** @page introtospin Introduction to SPIN
 * 
 * @htmlonly <br><br> @endhtmlonly
 * @image html SPIN_setup.png
 * 
 * @htmlonly <br><br> @endhtmlonly
 * @section SPIN SPIN:
 * 
 * A SPIN setup comprises three main components, the spinserver application, the
 * spinviewer application, and a control mechanism of some sort, which could be:
 * 
 * - Pure data (the popular visual programming environment) and pdsheefa
	 (Pure data patches designed specifically for SPIN)
 * - a web application built to interact with SPIN
 * - any other means of sending OSC (Open Sound Control format) messages
 * - Python scripts attached to objects in the SPIN scene through one of these
 *	   other methods
 * 
 * @htmlonly <br> @endhtmlonly
 *
 * @section spinserver Spinserver
 *
 * The spinserver is the core of any SPIN setup--the actual server application
 * to which control messages are sent and to which additional computers (if any)
 * connect for the purposes of collaborative work. Connected computers could be
 * in the same room in a LAN setup, or they could be across the ocean.
 *
 * Once activated, spinserver is an important source of information about the
 * state of the setup. Messages will appear here to announce other computers
 * connecting to your session, commands received by those computers, and debug
 * information for when things are not working correctly.
 *
 * Spinserver can be activated either by executing it through its icon or by the
 * console command 'spinserver'
 * 
 * Here is an in-depth technical explanation of how spinserver operates:
 *
 * @htmlonly <br> @endhtmlonly
 *
 * @section spinviewer Spinviewer
 *
 * Spinviewer is the application through which it is possible to view the SPIN 
 * scene. It looks similar to other media applications; it is just a scalable 
 * window through which the 3d contents of the scene are displayed. On Mac the
 * frame of the application has a button for toggling a world grid on or off in
 * the scene (a grid of lines that can help orient the user with the center of 
 * the scene).
 *
 * It is possible for numerous spinviewers to connect to the same scene. The
 * first instance of spinviewer on each machine automatically instantiates a
 * UserNode, which is essentially a controllable virtual camera that can move
 * around in 3d space within the scene. UserNodes are by default invisible, so 
 * multiple users in the same scene will not see each other unless other visible
 * objects are attached to them.
 *
 * If multiple spinviewers are opened on the same machine, each one will
 * navigate by means of the same UserNode, unless a second one is created with
 * the command line argument --user-id. (See @htmlonly 
     <a href=console.html>Console Commands</a>@endhtmlonly).
 *
 * @htmlonly <br> @endhtmlonly
 *
 * @section control Control Mechanisms
 *
 * There are several possible ways to control SPIN, and we will devote a
 * a separate page to describing all of them. The most common is Pure data
 * extended and a suite of Pure data patches built specifically for SPIN called
 * pdsheefa. These are being used in the image above.
 *
 * @htmlonly <br><br> @endhtmlonly
 *
 * Next page, @htmlonly <a href=basicnavigation.html>Basic Navigation</a><br><br>
 * <a href=index.html>Return to Index</a> @endhtmlonly
 */

 /** @page basicnavigation Basic Navigation
 * 
 * @section controls UserNode controls
 * 
 * The easiest way to navigate the SPIN scene is through use of a basic mouse. 
 * Holding down the left button while moving the mouse allows you to move the
 * camera left, right, forward and backward (XY plane), and holding down the
 * right button allows you to move left, right, up and down (XZ plane). Holding
 * down both buttons allows you to "look" around in 360 degrees without moving
 * from your position.
 * 
 * A 3d mouse works very well and is recommended, if possible.
 * 
 * The UserNode is an object with a specific location in the virtual space--if
 * it is visible (meaning that another visible shape or model is attached to
 * it), it will be seen to move around by others as you navigate the scene. If
 * not, UserNodes are invisible by default and no one else will see your
 * position in the scene.
 * 
 * @section other Other Controls
 * 
 * There are several other keyboard controls that affect the behaviour of SPIN.
 * These controls are sent directly to the spinviewer application and have no
 * effect on other participants at different computers, unlike moving one's
 * camera as in the descriptions above (in which case your UserNode, if visible,
 * will be seen to move around by other participants). These commands are as
 * follows:
 * 
 * - h : toggle onscreen help within the spinviewer
 * - < > : decrease/increase the screen resolution (in windowed mode only)
 * - S : output stats to console
 * - s : onscreen stats*
 * - b : toggle backface culling
 * - e : toggle the placement of the end of the frame barrier
 * - f : toggle full screen
 * - l : toggle lighting 
 * - m : toggle threading mode
 * - t : toggle texturing
 * - w : toggle polygon fill mode between fill, line (wireframe), and points
 * 
 * 
 * 
 * * On screen stats has several layers, each accessed by pressing s again
 * 
 * first press : framerate
 * 
 * second : information about how the computer draws the scene (threading)
 *
 * third : information about what is visible to the camera (lights, polygons, etc)
 *
 * fourth : information about the contents of the entire scene
 * 
 * fifth : clears the on-screen statistics
 * 
 * 
 * Next page, @htmlonly <a href=console.html>Console Commands</a><br><br>
 * <a href=index.html>Return to Index</a> @endhtmlonly
 * 
 */

 /**
 * 
 * @page console Console Commands
 * 
 * When run through the command line, there are a number of arguments that
 * modify the behaviour of SPIN. These are used by typing them after the command
 * for opening the application. For example, to open spinserver help, you would
 * type 'spinserver -help' into the console.
 * 
 * @htmlonly <br> @endhtmlonly
 * @section spinserver spinserver
 * 
 * - -h or -help : brings up a menu showing essentially the contents of this
 * list (this command does not open the spinserver application)
 *
 * - --disable-auto-cleanup : disables the auto-cleanup of user nodes if they 
 * stop sending ping messages. Normally participants in a SPIN scene ping (send
 * a message toe the spinserver to test speed) every 20 seconds. If they do not
 * do this for 60 seconds, spinserver removes them from the session and they
 * must re-connect. Use this option to prevent users from getting kicked off if
 * the network conditions are not optimal.
 * - --disable-discovery : disables the multicast discovery services. This
 * essentially turns off the automatic connection to network channels, requiring
 * the user to connect manually. This could be useful in a situation where
 * several spinservers are operating on the same network.
 * - --http-port <port> : sets a local port from which messages can be sent to
 * your spinserver. This is used for building controls for SPIN into a webpage.
 * The default is port 9980.
 * - --recv-tcp-msg <port> : sets the port where spinserver listens to TCP
 * messages. Subscription requests from client computers and from your own
 * spinserver will come here by default. It is also possible to send scene
 * events here for increased reliability, though by default they use UDP and a
 * different port. The default port is 54322.
 * - --recv-udp-msg <host> <port> : sets the port address where spinserver
 * listens for UDP messages from clients. The majority of messages are sent this
 * way by default, for example, the creation and manipulation of objects in the
 * scene. This argument may be repeated to assign multiple hosts and ports. The
 * default values are host 239.0.0.1 and port 54323).
 * - --scene-id <name> : to change the name of the session from default to
 * something else. In this way it is possible to run multiple SPIN sessions on
 * the same network.
 * - --send-udp-msg <host> <port> : sets the address/port for UDP multicast of
 * scene events, or this argument may be repeated for several unicast
 * connections. This is used to change the port where spinserver listens for its
 * commands. (default: 239.0.0.1 54324)
 * - --send-udp-sync <host> <port> : sets the address/port for timecode sync
 * messages. This allows all connected machines to be synchronized in time. In
 * this way, the contents of the scene will be roughly the same for all
 * participants at the same time. However, it does not account for network lag.
 * - --spatosc <translator> <URL> enables SPIN's internal spatOSC scene.
 * Example: --spatosc BasicTranslator osc.tcp://localhost:18033
 * - --ttl <number> sets the TTL (time to live) for multicast packets in order
 * to hop across routers (default 1).
 * - --version displays the version number and exits spinserver.
 * 
 * @htmlonly <br> @endhtmlonly
 * @section spinviewer spinviewer
 * 
 * Like spinserver, use of the viewer can be modified with the addition of
 * arguments when run from a command line utility:
 * 
 * - -h or -help : displays information about the command line arguments
 * (roughly similar to the list below)
 *
 * - --clipping <near far> : sets fixed clipping planes (the minimum and maximum
 * distance from the camera at which objects will be rendered).
 * - --config <filename> : provides a configuration file to customize the setup
 * of multiple windows/cameras.
 * - --disable-camera-controls : disables the mouse-based camera controls for
 * this user. This is helpful when using a mouse picker (the mouse selects and
 * manipulates objects in the scene rather than moving the camera).
 * - --disable-discovery : disables the multicast discovery service. This could
 * be useful in a situation where several spinservers are operating on the same
 * network.
 * - --enable-mouse-picker : enables the mouse picker and sends events to the
 * server.
 * - --frustum <x y z a b c> : TO BE UPDATED
 * - --recv-tcp-msg <host> <port> : sets the desired receiving address/port when
 * subscribing for TCP with spinserver (default 54325).
 * - --recv-udp-msg <host> <port> : sets the receiving address/port for UDP
 * messages from the server. The address can be a multicast address, or
 * 'localhost.' This is where you will receive update messages about the SPIN
 * scene (default 54323).
 * - --recv-udp-sync <adresss> <port> : sets the address/port for timecode sync.
 * This is where you receive messages that keep your viewer synchronized in time
 * with the server and the other participants in a scene (default 239.0.0.1 54321).
 * - --scene-id <name> : is important for making sure the viewer connects to the
 * appropriate spinserver. Scene name is "default" by default.
 * - --screen <num> : sets which screen to display on, in the case of multiple
 * monitors on one computer (default: all of them).
 * - --send-tcp-msg <host> <port> : allows you to manually set the address/port
 * through which to send TCP messages to a spinserver and can be used to connect
 * to SPIN sessions that are not on the same local network, i.e. across the
 * ocean (default localhost 54322).
 * - --send-udp-msg <host> <port> : is used to specify the port through
 * which we will send UDP messages to the spinserver, for the same reasons as in
 * the entry above (it is non-local). This port is used the most often. It is
 * less reliable but creates considerably less network traffic. Events with
 * really high occurrence, such as the movement of an object in the scene, are
 * handled this way by default.
 * - --spatosc <translator> <URL> : enables SPIN's internal spatOSC scene.
 * example: --spatosc BasicTranslator osc.tcp://localhost:18033
 * - --ttl <number> : is used to set the TTL (time to live) for multicast
 * packets in order to hop across routers (default 1).
 * - --user-id <uniqueID> : is used to specify a User ID for this viewer, which
 * by default would be the name of the computer.
 * - --version : displays the version number and then exits spinviewer.
 * - --window <x y w h> : sets the position (w,y) and size (w, h) of the view
 * window (deafult 50 50 640 480).
 *
 * @htmlonly <br> @endhtmlonly
 * Next page, @htmlonly <a href=console.html>Console Commands</a><br><br>
 * <a href=index.html>Return to Index</a> @endhtmlonly
 * 
 */ 

#endif
