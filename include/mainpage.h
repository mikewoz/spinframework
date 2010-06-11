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
*/

/** @page oscprotocol OSC Protocol
 */


/** @page mscdiagrams Diagrams
 *
 * This is a page for diagrams
 *
 * \section sec_milhouse Distributed video textures (using Milhouse)
 *
 * The following describes how multiple users can instantiate shared video 
 * textures on SPIN using Milhouse.
 * 
 * \msc
 * 

width=1000;
spinServer, spinViewer1, milhouseClient1, milhouseClient2;

...;
--- [label="only spinServer is running"];

...;
--- [label="spinViewer1 launches"];

spinViewer1->spinServer [label="/spin createNode user01 UserNode"];
spinServer->* [label="/spin/default createNode user01 UserNode"];

...;

--- [label="milhouseClient1 launches"];
milhouseClient1->spinServer [label="/spin createNode shvid01 SharedVideoNode\n/spin/shvid01 setParent user01"];
---;
spinServer->* [label="/spin/default createNode shvid01 SharedVideoNode\n/spin/shvid01 setParam videoPort 10000\n/spin/shvid01\n/spin/shvid01 start"];
---;
---;
milhouseClient1->milhouseClient1 [label="got /shvid01 start,\nso initiate milhouse SENDER"];

...;
--- [label="milhouseClient2 launches"];
milhouseClient2->spinServer [label="/spin/default createNode shvid02 SharedVideoNode"];
spinServer->* [label="/spin/default createNode shvid02 SharedVideoNode\n/spin/shvid02 setParam videoPort 10020\n/spin/shvid02\n/spin/shvid02 start"];
---;
milhouseClient2->milhouseClient2 [label="got /shvid02 start,\nso initiate milhouse SENDER"];

...;
milhouseClient2->spinServer [label="/spin userRefresh"];
spinServer->* [label="/spin createNode shvid01 SharedVideoNode\n/spin/shvid01 setParam videoPort 10000\n/spin/shvid01\n/spin/shvid01 start"];
---;
milhouseClient2->milhouseClient2 [label="got /shvid01 start,\nso initiate milhouse RECEIVER\n(shmid=shvid01)"];

 *
 * 
 * 
 * \endmsc
 * 
 * A a typical invocation of a milhouse receiver will look something like this:
 * <pre>
 * milhouse -r --videoport 10000 --address 10.10.10.123 --videocodec mpeg4 --videosink sharedvideosink --shared_memory_id shvid01
 * </pre>
 *
 */


#endif
