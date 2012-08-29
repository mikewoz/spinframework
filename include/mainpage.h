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
* <br>
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
* <a href="buildosx.html">Building from Source on OSX</a><br><br>
*
* @section manual SPIN Manual
*
* <a href="introtospin.html">Introduction to SPIN</a><br>
* <a href="basicnavigation.html">Basic Navigation</a><br>
* <a href="pdsheefa.html">Introduction to Pure Data Extended and pdsheefa</a>
* <br><br>
* <a href="console.html">Console Commands</a><br>
* <a href="messaging.html">Technical Information on SPIN Messaging</a><br><br>
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
 * @page buildosx Building from Source on OSX
 * 
 * For developes working on OSX, here are detailed instructions for setting up a
 * development environment so that you can complile your own version of SPIN.
 * 
 * <br>
 * <b>STEP 1:</b> Set up OSX development features
 * 
 * To build SPIN from source, you will need XCode, MacPorts, and XCode's command
 * line tools.
 * 
 * 1. Download XCode from the 
 * <a href=http://itunes.apple.com/us/app/xcode/id497799835?ls=1&mt=12>
 * Mac App Store</a>
 * 2. Launch XCode, open the Preferences, select the Downloads tab and install
 * "Command Line Tools."
 * 3. Download and install the latest version of
 * <a href=http://www.macports.org/install.php>MacPorts</a>.
 * 
 * <br>
 * <b>STEP 2:</b> Install several dependency ports:
 * 
 * <tt>sudo port install automake cmake pkconfig doxygen wget select \ </tt>
 * <tt>boost +python27 ffmpeg +universal freetype +universal liblo +univeral \ </tt>
 * <tt>collada-dom +universal openvrml +universal jasper +universal \ </tt>
 * <tt>bullet +universal gdal +universal </tt>
 * 
 * The +universal option after a library ensures that both i386 and 64 bit
 * libraries are built. This is only needed if you plan to build OSG and SPIN in
 * 64-bit mode (recommended). If you already have some installed libraries which
 * were not built for 64 bit architecture, you may get errors during compilation
 * of other ports. To upgrade an existing port (eg, freetype), do this:
 *
 * <tt>sudo port upgrade --enforce-variants freetype +universal </tt>
 * 
 * (optional) you may need to select the version of python to use.
 *
 * <tt> sudo port select --set pythong python27 (MacPorts 2.x) </tt><br>
 * <tt> sudo python_select python27             (MacPorts 1.x) </tt>
 * 
 * To use web services in SPIN, you need <a href=http://pocoproject.org/>Poco
 * </a>, but as of writing this, the port is broken. You need to download and
 * compile it yourself:
 * 
 * <tt>cd ~/src </tt><br>
 * <tt>wget --no-check-certificate
 * <a href=https://sourceforge.net/projects/poco/files/sources/poco-1.4.3/poco-1.4.3p1.tar.gz>
 * https://sourceforge.net/projects/poco/files/sources/poco-1.4.3/poco-1.4.3p1.tar.gz</a></tt>
 * <br>
 * <tt>tar zxvf poco-1.4.3p1.tar.gz</tt><br>
 * <tt>./configure</tt><br>
 * <tt>make</tt><br>
 * <tt>sudo make install</tt>
 * 
 * <br>
 * <b>STEP 3:</b> Get OpenSceneGraph (OSG)
 *
 * OSG is available through MacPorts, and you can install it like this:
 * 
 * <tt>sudo port install OpenSceneGraph-devel +universal </tt>
 * 
 * Assuming that you want the latest developer's release (3.1.1 at the time of
 * this writing), follow these instructions:
 * 
 * <tt>cd ~/src </tt>
 * <tt>svn co 
 * <a href=http://www.openscenegraph.org/svn/osg/OpenSceneGraph/tags/OpenSceneGraph-3.1.1>
 * http://www.openscenegraph.org/svn/osg/OpenSceneGraph/tags/OpenSceneGraph-3.1.1</a>
 * </tt><br>
 * <tt>mkdir OpenSceneGraph-3.1.1-build</tt><br>
 * <tt>cd OpenSceneGraph-3.1.1-build</tt><br>
 * <tt>ccmake ../OpenSceneGraph-3.1.1</tt><br>
 * 
 * - Press 'c' for initial configure
 * - Press 'e' after reading notes
 * - Set the following build options to "ON" (use cursor and key):
 *
 * <tt> BUILD_OSG_EXAMPLES : ON </tt><br>
 * <tt> BUILD_OSG_PACKAGES : ON </tt><br>
 * 
 * - For OSX 10.6x
 * 
 * <tt>CMAKE_OSX_ARCHITECTURES :   x86_64;i386 </tt><br>
 * 
 * - For OSX 10.7+
 * 
 * <tt>OPENTHREADS_ATOMIC_USE_MUTEX : ON </tt><br>
 * <tt>OSG_WINDOWING_SYSTEM : Cocoa </tt><br>
 * 
 * - Press 'c' to configure again (twice?)
 * - Press 'g' to generate Makefiles
 * - Now you can build and install OSG (will take some time):
 * 
 * <tt> make </tt><br>
 * <tt> sudo make install </tt><br>
 * 
 * <br>
 * <b>STEP 4:</b> Get SPIN's sister projects, build them, and install them.
 * 
 * NOTE: These instructions switch to the 'develop' branch (unstable). Skip this
 * if you just want to compile the latest release build yourself.
 * 
 * <b>cppintrospection:</b><br>
 * <tt>cd ~/src </tt><br>
 * <tt>git clone git@github.com:sat-metalab/cppintrospection.git </tt><br>
 * <tt>cd cppintrospection</tt><br>
 * <tt>git checkout develop</tt><br>
 * <tt>./one_step_build.sh</tt><br>
 * <tt>sudo make install</tt><br><br>
 * <b>spatosc:</b><br>
 * <tt>cd ~/src</tt><br>
 * <tt>git clone git@github.com:sat-metalab/spatosc.git</tt></b>
 * <tt>cd spatosc</tt><br>
 * <tt>git checkout develop</tt><br>
 * <tt>./one_step_build.sh</tt><br>
 * <tt>sudo make install</tt><br>
 * 
 * NOTE: For both cppintrospection and spatosc, if you want a universal build,
 * instead of ./one_step_build.sh, you need to do:
 * 
 * <tt>./autogen.sh</tt><br>
 * <tt>./configure --enable-universal --disable-dependency-tracking</tt><br>
 * <tt>make</tt><br>
 * 
 * <br>
 * <b>STEP 5:</b> Build SPIN
 * 
 * Download the source from the git repository:
 * 
 * <tt>sudo git clone git://code.sat.qc.ca/spinframework.git</tt><br>
 * <tt>cd spinframework</tt><br>
 * 
 * If you want the bleeding edge (unstable) branch:
 *
 * <tt>git checkout develop</tt><br>
 *
 * Now build and install:
 * 
 * <tt>./one_step_build.sh</tt><br>
 * <tt>sudo make install</tt><br>
 * 
 * The rest of the instructions are coming soon.
 * 
 * <br>
 * Start learning, with an <a href=introtospin.html>Introduction to SPIN</a>
 * <br><br>
 * <a href=index.html>Return to Index</a> 
 * 
 */

/**
 * @page quickosx Quick Start Guide for OSX
 * 
 * This guide will show you how to get started using SPIN with
 * <a href=http://puredata.info/>Pure Data</a> and the example patches from
 * <a href=https://code.sat.qc.ca/trac/pdsheefa>pdsheefa</a>. You can use any
 * other software that can send <a href=http://opensoundcontrol.org>OSC</a>
 * messages, but then you won't have any examples to start with.
 * 
 * <br>
 * <b>STEP 0:</b> (Optional) Get extra dependancies
 *
 * For advanced features, like video textures, SPIN requires libraries installed
 * via MacPorts, which requires XCode command line tools. <b>Novice users should
 * skip this step for now.</b>
 * 
 * 1. Downalod XCode from the
 * <a href=http://itunes.apple.com/us/app/xcode/id497799835?ls=1&mt=12>
 * Mac App Store</a>.<br>
 * 2. Launch XCode, open the preferences, select the Downloads tab and "Install
 * command line tools."<br>
 * 3. Download and install the latest version of
 * <a href=http://www.macports.org/install.php>MacPorts</a>.<br>
 * 4. Open the Terminal.app and install the desired ports:
 * 
 * <tt> sudo port install ffmpeg</tt><br>
 * 
 * <br>
 * <b>STEP 1:</b> Get SPIN
 * 
 * Start by downloading the latest version of the SPIN Framework package and
 * drag the "spinviewerUI" and "spinserverUI" .apps to your Applications folder:
 * 
 * <a href=http://spinframework.org/downloads>http://spinframework.org/downloads</a>
 * 
 * <br>
 * <b>STEP 2:</b> Get Pd-Extended
 * 
 * Do the same for Pure Data Extended. Download the latest version and drag it
 * to your Applications folder:
 * 
 * <a href=http://puredata.info/downloads/pd-extended>
 * http://puredata.info/downloads/pd-extended</a>
 * 
 * <br>
 * <b>STEP 3:</b> Get pdsheefa
 * 
 * Download pdsheefa:
 * 
 * <a href=http://code.sat.qc.ca/downloads/pdsheefa/osx>
 * http://code.sat.qc.ca/downloads/pdsheefa/osx</a><br>
 * 
 * Unpack this package to some other place on your hard drive, for example:
 * 
 * <tt>~/Library/Pd/pdsheefa</tt><br>
 * (This folder can be placed in any location you like, however)
 *
 * <br>
 * <b>STEP 4:</b> Tell Pd where to find pdsheefa
 * 
 * Open Pd-extended. From the dropdown menu at the top of the screen, select
 * Pd-extended > Preferences > Path (pictured below).
 *
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/PD_Path_Selection_0.png
 * 
 * <br>
 * A window will open which displays all of the locations that Pure Data looks
 * to find the information it needs when it loads up. We need to add the
 * pdsheefa tools to this list so that Pure Data can access them.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/PD_path_addition_0.png
 * 
 * <br>
 * Select New. In the file browser that opens, select the location where you
 * have placed pdsheefa. These tools are now available through your Pure Data
 * interface.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/select_pdsheefa_location_0.png
 * 
 * <br>
 * Select OK to close the window.
 * 
 * <br>
 * <b>STEP 5:</b> Test the spinserver and spinviewer
 * 
 * Now we want to open up the two main SPIN framework applications themselves.
 * One is a software server called spinserverUI, and the other is a graphical
 * window called spinviewerUI. Open your Applications folder to find these
 * programs and open them.
 * 
 * Open spinserverUI. 
 *
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/open_spin_server_0.png
 * 
 * <br>
 * Also, open spinviewerUI.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/open_spin_viewer_0.png
 * 
 * It is important here to verify that spinviewerUI and spinserverUI are
 * communicating to each other. When you open sinviewerUI, you should see a
 * message appear in your spinserverUI window that begins "Got new subscriber"
 * and is followed with the name of your machine and its IP address.
 * 
 * Another test is to try to turn the grid on in the spin viewer. Click the
 * button "Toggle Grid," which is the rightmost button in the list of buttons at
 * the top left of your spin viewer window (see below).
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/open_pd_0.png
 * 
 * <br>
 * <b>STEP 6:</b> Try the examples
 * 
 * From with Pure Data, select File > Open from the dropdown menu at the top of
 * the screen. 
 *
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/pdsheefa_examples_0.png
 * 
 * Using the file browser that opens, find the pdsheefa folder where
 * you have placed it on the hard drive, and find the examples subfolder.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/spin_messaging_tool_in_pdsheefa_0.png
 * 
 * The files that begin with the numbers 01-99 are SPIN tools that come with
 * included explanations. Open the first one and begin learning SPIN!
 * 
 * <br>
 * Start learning, with an <a href=introtospin.html>Introduction to SPIN</a>
 * <br><br>
 * <a href=index.html>Return to Index</a>
 * 
 */

 /**
 * @page quickubuntu Quick Start Guide for Ubuntu Linux
 * 
 * NOTES:
 * 
 * - These instructions are intended for the latest LTR: Ubuntu 12.04 (Precise Pangolin)
 * - Instructions are terminal commands. Use the CTRL-ALT-T shortcut to open a terminal.
 * - This guide explains how to get started using SPIN with
 * <a href=http://puredata.info/>Pure Data</a> and the example patches from
 * pdsheefa. You can use any other software that can send 
 * <a href=http://opensoundcontrol.org/>OSC</a> messages, but you will miss out
 * on the example patches.
 * 
 * <br>
 * <b>STEP 1:</b> Setup repositories
 * 
 * SPIN requires some packages not found in the default Ununtu repositories, so
 * use the following commands to add some new PPAs to your system:
 * 
 * <tt> sudo apt-add-repository ppa:sat-metalab/metalab</tt>
 * 
 * Whenever you change repositories, you need to update your package list:
 * 
 * <tt>sudo apt-get update</tt>
 * 
 * <br>
 * <b>STEP 2 (OPTIONAL):</b> Install dependancies
 * 
 * This step is optional, since the spinframework package will automatically
 * require you to install these in step 3. However, this is here for general
 * reference, and for those who may want to compile SPIN from source:
 * 
 * <tt>sudo apt-get install doxygen git git-core cmake-curses-gui build-essential \ </tt>
 * <tt>automake libtool libxml++2.6-dev python-dev python-setuptools \ </tt>
 * <tt>libboost-dev libboost-filesystem-dev libboost-python-dev \  </tt>
 * <tt>libboost-regex-dev libboost-thread-dev liblo-dev help2man \ </tt>
 * <tt>openscenegraph libopenscenegraph-dev libopenthreads-dev \ </tt>
 * <tt>libboost-program-options-dev libboost-system-dev \ </tt>
 * <tt>libcppintrospection-3.0-dev libspatosc-0.3-dev </tt>
 * 
 * These following dependancies are <b>not</b> required in order to build and
 * run SPIN, but add optional support that will be included if the libraries are
 * present:
 * 
 * For physics, we need <a href=http://bulletphysics.com/>Bullet Physics</a>:
 * 
 * <tt> sudo apt-add-repository ppa:openrave/release</tt>
 * <tt>sudo apt-get update</tt>
 * <tt>sudo apt-get install libbullet-dev</tt>
 * 
 * For web services, we use <a href=http://pocoproject.org/>Poco</a>:
 * 
 * <tt>sudo apt-get install libpoco-dev</tt>
 * 
 * For video textures, we need <a href=http://ffmpeg.org/>ffmpeg</a>:
 *
 * <tt>sudo apt-get install ffmpeg</tt>
 * 
 * If you have a 
 * <a href=http://www.3dconnexion.com/products/spacenavigator.html>
 * SpaceNavigator</a> device:
 * 
 * <tt>sudo apt-get install libspnav-dev spacenavd</tt>
 * 
 * For streaming video textures, we use GStreamer and
 * <a href=http://code.sat.qc.ca/redmine/projects/libshmdata/wiki>libshmdata</a>:
 * 
 * <tt>sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-based0.10-dev</tt>
 * <tt>sudo apt-get install libshmdata-0.4-dev</tt>
 * 
 * For point clouds (and support for Microsoft Kinect), we use
 * <a href=http://pointclouds.org/>PointCloudLibrary</a>:
 * 
 * <tt>sudo apt-add-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl</tt>
 * <tt>sudo apt-get update</tt>
 * <tt>sudo apt-get install libpcl-1.6-all</tt>
 * 
 * <br>
 * <b>STEP 3:</b> Install SPIN
 * 
 * <b>option a)</b> Use the package (may be slightly out of date):
 *
 * <tt>sudo apt-get install spinframework</tt>
 * 
 * <b>option b)</b> Build it from source:
 * 
 * Download the source from the git repository:
 * 
 * <tt>sudo git clone git://code.sat.qc.ca/spinframework.git</tt>
 * <tt>cd spinframework</tt>
 * 
 * If you want the bleeding edge (unstable) branch:
 *
 * <tt> git checkout develop</tt>
 * 
 * Now build and install:
 * 
 * <tt>./one_step_build.sh</tt>
 * <tt>sudo make install</tt>
 * 
 * <br>
 * <b>STEP 4:</b> Install Pd-Extended
 * 
 * The most comprehensive examples for SPIN are written for Pure Data 
 * (specifically, Pd-Extended), which is unfortunately not packaged very well.
 *
 * Try this first:
 * 
 * <tt>sudo apt-get install pd-extended</tt>
 * 
 * If you get an error that says "E: Couldn't find package pd-extended," try
 * downloading the latest version from the Pd website:
 * 
 * <a href=http://puredata.info/downloads/pd-extended>
 * http://puredata.info/downloads/pd-extended</a>
 * 
 * At the time of writing this, there was no 64-bit download for Precise, so
 * we've put one here:
 * <a href=http://code.sat.qc.ca/redmine/attachments/download/202/Pd-0.43.1-extended-20120430.deb>
 * Pd-0.43.1-extended-20120430.deb</a>.
 * 
 * <br>
 * <b>STEP 5:</b> Install pdsheefa
 * 
 * Download the latest version of pdsheefa from: 
 * <a href=http://code.sat.qc.ca/downloads/pdsheefa>
 * http://code.sat.qc.ca/downloads/pdsheefa</a>
 * 
 * The file will be compressed. Unpack the archive and move the contents to a
 * place on your hard drive, for example: ~/pd-externals/ or some other location
 * (in this example, you will need to create a 'pd-externals' folder).
 *
 * Then we will need to open <b>pd-extended</b> and add that path to Pd's list
 * of search paths. For Pd versions 0.43+, select Edit > Preferences from the
 * menu. For older versions, select File > Paths from the menu.
 * 
 * Click the "New" button and use the file browser to show Pd the location where
 * you have placed the pdsheeda files.
 * 
 * <br>
 * <b>STEP 6:</b> Test SPIN
 * 
 * Launch two applications: <b>spinserver</b> and <b>spinviewer</b>.
 * 
 * You can do this either by opening two terminals and typing one command in
 * each window, or using the Dash.
 * 
 *  Launch <b>pd-extended</b>, and open the patch in the pdsheefa/examples
 * folder called 02.SpinWidgets.pd. You will see a connect toggle under the
 * first step, and it should get checked, indicating that the connections are
 * all set up properly.
 * 
 * Toggle the grid on and off to ensure that OSC messages are in fact being
 * transmitted.
 * 
 * <br>
 * <b>STEP 7:</b>
 * 
 * Go through all the examples in the pdsheefa/examples that are numbered 01 to
 * 99 and learn about various features of SPIN.
 * 
 * <br>
 * Start learning, with an <a href=introtospin.html>Introduction to SPIN</a>
 * <br><br>
 * <a href=index.html>Return to Index</a>
 *
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
 * @image html http://spinframework.org/images/spin-banners-immersive.png
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
 * @image html http://spinframework.org/images/spin-banners-touch.png
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
 * @image html http://spinframework.org/images/spin-banners-telepresence.png
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
 * @image html http://spinframework.org/images/spin-banners-installations.png
 * 
 * Artistics works developed with the SPIN framework have been presented in a
 * number of international festivals, conferences, and performances. See the
 * <a href=showcase.html>showcase</a> section for several examples.
 *
 * <br>
 * @section audio Rich support for 3d audio:
 *
 * @image html http://spinframework.org/images/spin-banners-cyclo.png
 *
 * Along with <a href=https://github.com/sat-metalab/spatosc>SpatOSC</a> and
 * <a href=https://code.sat.qc.ca/trac/pdsheefa>Pdsheefa</a>, the framework can
 * provide opportunities for deep experimentation with spatial audio.
 *
 * <br>
 * @section prototype Rapid prototyping of 3d interaction:
 *
 * @image html http://spinframework.org/images/spin-banners-wii.png
 *
 * Network communication for SPIN is based on OpenSoundControl (OSC), a standard
 * protocol used by rapid prototyping systems like Pure Data, Max/MSP,
 * OpenFrameWorks, Processing, etc. SPIN provides generica mechanisms for
 * spatial manipulation, based on geometric events (movement, intersection,
 * collisions, etc.) so a number of interaction paradigms can be explored.
 *
 * <br>
 * Start learning, with an <a href=introtospin.html>Introduction to SPIN</a>
 * <br><br>
 * <a href=index.html>Return to Index</a>
 *
 */

/** @page oscprotocol OSC Protocol
 */

/** @page introtospin Introduction to SPIN
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/SPIN_setup_0.png
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

 /** 
 * @page basicnavigation Basic Navigation
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
 * <br>
 * Next page, <a href=pdsheefa.html>Introduction to Pure Data Extended 
 * and pdsheefa</a><br><br>
 * <a href=index.html>Return to Index</a>
 * 
 */

 /**
 * 
 * @page pdsheefa Introduction to Pure Data and pdsheefa
 * 
 * @section pde Pure Data Extended
 * 
 * Pd (aka Pure Data) is a real-time graphical programming environment for
 * audio, video, and graphical processing. It is the third major branch of the
 * family of patcher programming languages known as Max (Max/FTS, ISPW Max, 
 * Max/MSP, jMax, etc.) originally developed by 
 * <a href=http://crca.ucsd.edu/~msp/>Miller Puckette</a> and company at
 * IRCAM. The core of Pd is written and maintained by Miller Puckette and 
 * includes the work of many developers, making the whole package very much a
 * community effort.
 * 
 * Pd-extended is a community project that includes most of the libraries 
 * from the pure-data source code repository. It is generally the most 
 * complete assembly of all available libraries, extensions, and 
 * documentation there is.
 * 
 * <br>
 * @section pdsheefa pdsheefa
 * 
 * Pdsheefa is a collection of patches built in Pd-extended and explicitly
 * intended as control mechanisms for SPIN. They are not the only way to
 * interact with SPIN, but as of this moment it is probably the easiest and
 * fastest, and it is certainly a good entry point to begin learning about how
 * everything functions.
 * 
 * So, to start out, let's open a complete SPIN setup. We will need a
 * spinserver application, a spinviewer, Pure Data extended and the colleciton
 * of pdsheefa patches. Once these are all opened, open the first tutorial
 * pdsheefa patch through your File menu in Pure Data extended.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/widgets.png
 * 
 * <br><br>
 * @section widgets 01.SpinWidgets.pd
 * 
 * "SPIN Widgets" is a collection of graphic interface widgets, which help to
 * accomplish common tasks in the SPIN environment. It is composed of three
 * indispensible tools, with which just about all basic functions can be done.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/SPIN.Connect.png
 * 
 * <br>
 * <b> SPIN.Connect</b>
 * 
 * <br>
 * SPIN.Connect is an indispensible tool for verifying that you are connected
 * to the SPIN server, that you are viewing the correct scene and that it is
 * currently responsive in input.
 * 
 * First, at top right, you'll notice a drop-down menu where it is possible to
 * find the names of all SPIN scenes available on the network. A checkbox next
 * to the dropdown indicates whether you are currently connected to the scene
 * whose name is displayed. Underneath that are a few basic commands:
 * 
 * clearScene: delete all nodes except for your own UserNode and clear the 
 * state of the current scene.
 * 
 * clearUsers: kick all other users from the scene
 * 
 * refresh: Use this to force spinviewer to refresh the content visible in the
 * scene (useful if you believe you have momentarily lost connection and are
 * not seeing the scene up-to-date).
 *
 * grid: This little button toggles the world grid on or off in the scene. This
 * is probably the easiest way to rapidly check connectivity to the scene.
 * 
 * msg-print: This toggles whether messages sent to spinserver will be displayed
 * in spinserver.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/NodeCreator.png
 *
 * <br>
 * <b>SPIN.NodeCreator</b>
 * 
 * <br>
 * NodeCreator is an object that facilitates the creation of nodes. It is quite
 * simple to use. A field called "id" is available to write in the name of the
 * node you wish to create. 
 *
 * Nodes are the basic object type in SPIN. Every entity in your scene,
 * including the camera which you are using to navigate, is a sub-type of node.
 * The most common nodes, after the UserNodes (cameras) which users can use to
 * navigate scenes, are shapes, models, and the like.
 * 
 * All nodes must have unique names so that it is possible to send messages
 * to them directly. Go ahead and enter the name Box into the id field. Now, to
 * the right, you'll see a drop-down menu with the different types of possible
 * nodes that you can create. Select ShapeNode from the list. If a list is not
 * displayed, click the 'refresh' button to re-populate the list with the
 * available node types. Lastly, push the 'go!' button to actually issue the
 * command you have just set up.
 * 
 * You have just created a ShapeNode, which by default looks like a box. 
 * (Most nodes are invisible by default until certain changes are made to them). 
 * Now we will look at the Node.Chooser object to assign qualities to the 
 * ShapeNode we have created.
 *
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/NodeChooser.png
 * 
 * <br>
 * <b>SPIN.NodeChooser</b>
 * 
 * The Node.Chooser is a tool that allows you to select from a list of all nodes
 * existing in the scene, and then it displays information about the parameters
 * of that node and allows you to change them directly.
 * 
 * Select your node, "Box" from the drop-down menu at top right. If it does
 * not appear, verify that you are connected to the current scene with the 
 * SPIN.Connect device at the top of this patch. Hit the refresh button.
 * 
 * Once your node is selected, you'll note that there are numerous modifiable
 * parameters. The first is 'Parent,' which is familiar to users of many
 * different kinds of computer applications. If you set another object to be
 * the parent of Box, Box will always move whenever the parent is moved, will
 * resize proportionately with the other object, and generally mimic any changes
 * made to the "parent" item.
 * 
 * Next is "Shape," which is the principal parameter of a ShapeNode, for obvious
 * reasons.
 * 
 * <br><br>
 * @image html http://spinframework.org/sites/default/files/images/Box_Sphere.png
 *
 * The ShapeNode "Box" has been changed into a sphere and moved -1 units on the
 * X axis, using Node.Chooser.
 * 
 * <br><br>
 * There are a number of other useful tools in the Node.Chooser, and they differ
 * for each node type, but let's go through the rest of what is here. We have a
 * StateSet selector, set to Null. This is a feature that allows you to save
 * information about every node into a complete set, which can then be returned
 * to.
 * 
 * Then you have the X,Y, Z coordinates, which have been used in the example
 * picture above to shift the shape one unit to the left. You can drag these
 * values with the mouse, but the shape will quickly go off screen if you do
 * not zoom to get a wider viewpoint.
 * 
 * Pitch, Yaw, Roll: change the angle of the object on the three axes.
 * 
 * Copy, Paste: function just like copy paste in any other application.
 * 
 * Scale X,Y,Z: Scale the object independently in each of the three dimensions.
 * 
 * Or, use uniform-scale to scale it without changing the proportions of the
 * object.
 * 
 * There is an RGBA slider for setting the color and alpha channel of objects.
 * 
 * Three different "billboard" settings, normal, which is not billboarded.
 * Billboard, which forces the image to always face the camera (this is
 * especially useful for making 2d objects look like 3d objects). Z-up, which
 * billboards the object but only on the XY plane (if you fly over it, you
 * will see the top rather than the face, which will always turn to face you
 * if you go around the object by any side at the same height).
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
