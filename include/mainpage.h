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
* <br><br>
* 
* @section welcome Welcome
*
* The <b>SPIN Framework</b> is a software suite and set of libraries for Linux
* and OSX that support '<b>Sp</b>atial <b>In</b>teraction' and 3d visualization
* for networked virtual environments. Use the links below to get started:
* 
* <a href="overview.html">Overview of SPIN</a><br>
* <a href="quickosx.html">Quick Start Guide (OSX)</a><br>
* <a href="quickubuntu.html">Quick Start Guide (Ubuntu)</a><br><br>
*
* @section manual SPIN Manual
*
* <a href="introtospin.html">Introduction to SPIN</a><br>
* <a href="basicnavigation.html">Basic Navigation</a><br>
* <a href="console.html">Console Commands</a><br>
* <a href="messaging.html">SPIN Messaging</a><br><br>
* 
*
* @section API SPIN Framework API Documentation
*
* For class API documentation, see the
* <a href="annotated.html">Class List</a>
*
* For the full list of OSC messages accepted by SPIN, see the
* <a href="oscprotocol.html">OSC Protocol Documentation</a>
*/

/**
 * @page overview Overview
 * 
 * The <b>SPIN Framework</b> is a software suite and set of libraries for Linux
 * and OSX that support '<b>Sp</b>atial <b>In</b>teraction' and 3d visualization
 * in networked virtual environments. It uses a client-server model to support
 * distributed immersive environments and collaborative interaction in shared
 * virtual reality.
 * 
 * - Open source (LGPL license)
 * - Extensible C++ architecture using <a href="http://openscenegraph.org">OpenSceneGraph</a> / OpenGL
 * - Uses <a href="http://opensoundcontrol.org">OpenSoundControl</a> (OSC) for network messaging
 * - Embedded Python interpreter to add custom 'behvaiours' to nodes
 * - Provides high-level control of scene state (affine transformations, texture
                                                 mapping, lighting control, interaction
                                                 logic, 3d sound, etc).
 * - Supports a number of 3d model formats (.osg, .3ds, .obj, .dae, .fbx, etc).
 * - Texture mapping support for videos (via FFMPEG) and most common image formats
 *
 * There are many potential domains of use for the framework, as outlined below: 
 * 
 * <br>
 * @section virtual Networked Immersive Environments and Virtual Reality
 *
 * @image html spin-banners-immersive.png
 *
 * The spinviewer application supports customizable screen configurations,
 * allowing for projection on domes, panoscopes, cylindrical displays, CAVEs,
 * and multimedia powerwalls. A networked infrastructure allows for the
 * distribution of rendering to several machines, thus balancing the load for
 * complex scenes. It also provides a mechanism to share virtual worlds between
 * several participants.
 *
 * <br>
 * @section kiosks Kiosks and Interactive Displays
 * 
 * @image html spin-banners-touch.png
 *
 * The SPIN Framework is well suited to the design and deployment of interactive
 * displays. Rich 3d graphics and unique interactivity features allow for a
 * variety of visualization paradigms. Support for bleeding edge controllers,
 * cameras, spatial audio, and tracking technology allows for highly
 * customizable experiences.
 *
 * <br>
 * @section immersive Immersive Telepresence
 *
 * @image html spin-banners-telepresence.png
 *
 * High fidelity telepresence integration using the <a href=http://scenic.sat.qc.ca/en/Scenic>Scenic</a>
 * library allows SPIN to integrate remote audio and video feeds into one 3d
 * scene. The framework supports multiple textures and several channels of audio,
 * depending on bandwidth, and supports texture replacement of UVW maps for fine
 * control of texture mapping.
 *
 * <br>
 * @section installations Artistic Installations
 *
 * @image html spin-banners-installations.png
 * 
 * Artistics works developed with the SPIN framework have been presented in a
 * number of international festivals, conferences, and performances. See the
 * <a href=showcase.html>showcase</a> section for several examples.
 *
 * <br>
 * @section audio Rich support for 3d audio:
 *
 * @image html spin-banners-cyclo.png
 *
 * Along with <a href=https://github.com/sat-metalab/spatosc>SpatOSC</a> and
 * <a href=https://code.sat.qc.ca/trac/pdsheefa>Pdsheefa</a>, the framework can
 * provide opportunities for deep experimentation with spatial audio.
 *
 * <br>
 * @section prototype Rapid prototyping of 3d interaction:
 *
 * @image html spin-banners-wii.png
 *
 * Network communication for SPIN is based on OpenSoundControl (OSC), a standard
 * protocol used by rapid prototyping systems like Pure Data, Max/MSP,
 * OpenFrameWorks, Processing, etc. SPIN provides generica mechanisms for
 * spatial manipulation, based on geometric events (movement, intersection,
 * collisions, etc.) so a number of interaction paradigms can be explored.
 *
 */

/** @page oscprotocol OSC Protocol
 */

/** @page introtospin Introduction to SPIN
 * 
 * <br><br>
 * @image html SPIN_setup.png
 * 
 * <br><br>
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
 * <br>
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
 * <br>
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
 * the command line argument --user-id. (See
     <a href=console.html>Console Commands</a>).
 *
 * <br>
 *
 * @section control Control Mechanisms
 *
 * There are several possible ways to control SPIN, and we will devote a
 * a separate page to describing all of them. The most common is Pure data
 * extended and a suite of Pure data patches built specifically for SPIN called
 * pdsheefa. These are being used in the image above.
 *
 * <br><br>
 *
 * Next page, <a href=basicnavigation.html>Basic Navigation</a><br><br>
 * <a href=index.html>Return to Index</a>
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
 * Next page, <a href=console.html>Console Commands</a><br><br>
 * <a href=index.html>Return to Index</a>
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
 * <br>
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
 * <br>
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
 * <br>
 * Next page, <a href=messaging.html>SPIN Messaging</a><br><br>
 * <a href=index.html>Return to Index</a>
 * 
 */

 /**
 * @page messaging Spin Messaging
 * 
 * <br>
 * @section about About SPIN Messaging
 * 
 * The SPIN Framework is designed so multiple processes can share high-level 3d
 * state over a network using <a href=http://opensoundcontrol.org>
 * OpenSoundControl</a> (OSC) messages. By default, all messages
 * are sent from the server via UDP multicast on the address of 239.0.0.1. An
 * INFO channel (port) is used to send notifications about available ports,
 * where clients can listen or connect and send messages to the server.
 * 
 * Below is a table that summarizes the important networking channels used by
 * SPIN. We will explain each of these in more detail below.
 * 
 * <br>
 * @htmlonly
 * <table align="center" border="1" cellpadding="3" cellspacing="0">
 * <tr>
 * <th>channel</th><th>Rx proto</th><th>Rx addr</th><th>Rx port</th>
 * <th>Tx proto</th><th>Tx addr</th><th>Tx port</th>
 * </tr>
 * <tr>
 * <th>INFO</th><td>multicast UDP</td><td>239.0.0.1</td><td>54320</td>
 * <td>multicast UDP</td><td>239.0.0.1</td><td>54320</td>
 * </tr>
 * <tr>
 * <th>SERVER</th><td>unicast UDP</td><td>[serverIP]</td><td>54324</td>
 * <td>multicast UDP</td><td>239.0.0.1</td><td>54323</td>
 * </tr>
 * <tr>
 * <th>SECURE</th><td>TCP</td><td>[serverIP]</td><td>54322</td><td>TCP</td>
 * <td align="center" colspan="2" rowspan="1">[client subscribes]</td>
 * </tr>
 * <tr>
 * <th>CLIENT</th><td>multicast UDP</td><td>239.0.0.1</td><td>54323</td>
 * <td>unicast UDP</td><td>[serverIP]</td><td>54324</td>
 * </tr>
 * <tr>
 * <th>TIMECODE</th><td align="center" colspan="2" rowspan="1">
 * same as client Rx addr</td><td>54321</td>
 * <td align="center" colspan="2" rowspan="1">same as client Tx addr</td>
 * <td>54321</td>
 * </tr>
 * </table>
 * <br>
 * <small>
 * NOTE: These are default settings, and can be overridden using command-line
 * arguments or environment variables.</small>
 * @endhtmlonly
 * 
 * <br>
 * Using the above table, we can imagine a basic setup consisting of one
 * spinserver and one spinviewer (a type of client). Any time the state changes
 * on the server, a multicast message is emitted by the server on port 54323.
 * The viewer listens on this port and updates its internal represenation so
 * that the change is drawn in the next frame. The client may want to send some
 * messages to the server. For instance, a spinviewer allows a user to pick and
 * move an object using the mouse. In this case, messages are sent to the server
 * on port 54324; the server state will be updates, and the resulting change is
 * again multicast on port 54323 and will be displayed.
 * 
 * <br>
 * @section protocol SPIN Message Protocol
 * 
 * In general, all messaging conforms to a particular protocol: 
 * 
 * 
 * <tt>/SPIN/&lt;sceneID&gt;/&lt;nodeID&gt; method &lt;var_args&gt;</tt>
 * 
 * All OSC messages are contained withint the /SPIN namespace, and may control a
 * scene element or a particular node within a scene. For example, a message to
 * /SPIN/someScene will control a scene with id "someScene" at the scene level,
 * allowing you to create/delete nodes, clear/refresh the scene, etc. If the
 * selector is more specific, for example /SPIN/someScene/someModel1, the
 * intention is to send messages to a particular node, and not the scene in
 * general. Wildcards are acceptable in the selector, so /SPIN/ @htmlonly
 * &#42;/shape&#42; @endhtmlonly would
 * target all nodes that start with "shape" on all servers on the network.
 *
 * <b>
 * For a full reference of all messages, see the</b> <a href=osc_protocol.html>
 * osc protocol page</a>.
 * 
 * <br>
 * @section info The Info Channel
 * 
 * To facilitate the network binding procedure, the server periodically
 * multicasts its information on the INFO CHANNEL, notifying potential clients
 * of its existence and providing the ports to which they must send messages.
 * The server's info messages looks like this:
 * 
 * <tt>/SPIN/__server__sceneID addr rxUDPport rxTCPport txAddr txPort syncPort</tt>
 *
 * In the case of our defaults described in the table above, a real example
 * might look like this: 
 * 
 *
 * <tt>/SPIN/__server__default 192.168.1.100 54322 54322 224.0.0.1 54323 54321</tt>
 * 
 * <br>
 * @section clientinfo Client INFO (and UserNodes)
 *
 * An "official" SPIN client process that communicates with a server should
 * create a UnserNode in the scene with a UserID as an identifier (typically the
 * computer's hostname is used as the UserID).
 *
 * The client is then required to periodically send the "ping" message to the
 * UserNode it created in order to assert the client's continued presence. If a
 * client suddenly disappears, the server will see an interruption in the stream
 * of "ping" messages to that UserNode and will thus clean up unnecessary state
 * (i.e. delete the UserNode and its subgraph).
 *
 * A client's "ping" message to the UserNode it created should be sent at least
 * once every 20 seconds; the message should be structed like this:
 * 
 * <tt> /SPIN/sceneID UserNodeID ping </tt>
 * 
 * <b>IMPORTANT:</b> Any UserNode created on the server (and any attached child
 * nodes) will be automatically removed by the server if an info message is not 
 * received within a regular interval (eg, within 60 seconds).
 *
 * <br>
 * @section securetcp Secure TCP Channel
 * 
 * The server listens to messages on UDP for non-critical control data, while
 * important state messages should be sent to the server via TCP. Any one may
 * send messages to the TCP channel, but it should be understood that this is a
 * one-way connection; resulting chages to the scene state will be sent out via
 * UDP multicast as usual.
 * 
 * If a client wants to receive important state from the server via TCP, a
 * subscription is required. The client must send a "subscribe" message to
 * the server's TCP port, in the following form:
 * 
 * <tt> /SPIN/sceneId subscribe UserID clientAddr ClientTCPport</tt>
 * 
 * The server will then form a unicast TCP connection to the client's specified
 * address, and send any requested state to that port. However, only the
 * following TCP queries are currently supported:
 * 
 * @htmlonly
 * <br>
 * <table align="center" border="1" cellpadding="3" cellspacing="0">
 * <tr>
 * <th>MESSAGE</th><th>RESULT</th>
 * </tr>
 * <tr>
 * <td>/SPIN/\<sceneID\> refresh</td><td>SPIN will send the entire scene in
 * bundles to the TCP receiver</td>
 * </tr>
 * <tr>
 * <td>/SPIN/\<sceneID\> getState \<nodeID\></td>
 * <td>SPIN will send the state of a node to the TCP receiver</td>
 * </tr>
 * </table><br> @endhtmlonly
 * 
 * @section sync Timecode and synchronization
 * 
 * A timecode is sent in a separate multicast OSC channel, allowing clients to
 * manage their own animations while maintaining sync with the server:
 * 
 * <tt> /SPIN/\<sceneID\> sync \<elappsed-time-in-milliseconds\></tt>
 * 
 * <br>
 * @section Example
 * 
 * To summarize, when SPIN launches, it periodically sends an info message on
 * the INFO channel:
 * 
 * <tt> /SPIN/__server__sceneID rxAddr rxUDPport rxTCPport txAddr txPort syncPort</tt>
 * 
 * Clients can just listen on UDP port txAddrL:txPort for scene updates, or they
 * can connect (via unicast to rxAddr:rxUDPport) and send server-level messages.
 * Example:
 * 
 * <tt> /SPIN/sceneID debug </tt><br>
 * <tt> /SPIN/sceneID createNode \<nodeType\> \<nodeID\></tt><br>
 * <tt> /SPIN/sceneID deleteNode \<nodeID\></tt><br>
 * <tt> /SPIN/sceneID load \<scenefile.xm\></tt><br>
 * <tt> /SPIN/sceneID save \<scenefile.xml\></tt><br>
 * <tt> /SPIN/sceneID clear</tt>
 * 
 * Once a particular node is created on the server, messages can be sent to it
 * as follows:
 * 
 * <tt> /SPIN/sceneID/nodeID setTranslation 1 2 3</tt><br>
 * <tt> /SPIN/sceneID/nodeID setOrientation 1 2 3</tt><br>
 * <tt> /SPIN/sceneID/nodeID setParent \<parentID\></tt>
 * 
 * All clients listening on the server's txAddr:txPort will be notified on these
 * updates, and can update themselves accordingly.
 * 
 * <br>
 * Next page, <a href=annotated.html>Class Reference</a><br><br>
 * <a href=index.html>Return to Index</a>
 *
 */ 



#endif
