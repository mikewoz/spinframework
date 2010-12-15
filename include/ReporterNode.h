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

#ifndef ReporterNode_H_
#define ReporterNode_H_

#include "ReferencedNode.h"
#include <osg/Timer>

typedef struct _reporterTarget {
	osg::observer_ptr<ReferencedNode> node;
	osg::Matrix matrix;
	bool contained;
} reporterTarget;

typedef std::map< std::string, bool > reportingType;


/**
 * \brief Reports geometric relations to other nodes in the scene
 * 
 * A list of targets can be specified, and then reports are computed with
 * respect to this node and it's subgraph. One must turn on each of these report
 * types using the setReport method. The current list is:
 *
 * DISTANCE		'distance' between this and the target
 *
 * INCIDENCE	'direction' (angle) to the target on the XY plane, and 'incidence', which is the angle difference between the reporterNode's current orientation to that which would point at the target
 *
 * ANGLES		reports the angle difference between the reporterNode's current orientation to that which would point at the target (available in both 'eulers' or 'quaternion' format)
 *
 * CONTAINMENT	whether this node is contained within this bounds of the target's subgraph (note: assumes the target is convex)
 *
 * OCCLUSION	whether the target is occluded (line-of-sight) for this object
 *
 *
 * Notes:
 *
 * - Globals reporting is not related to this node. See GroupNode for more info.
 *
 * - All angles are reported in radians
 *
 */

class ReporterNode : public ReferencedNode
{

public:

	ReporterNode(SceneManager *sceneManager, char *initID);
	virtual ~ReporterNode();

	/**
	 * The update callback for ReporterNode checks to see if a target or the
	 * the ReporterNode's global matrix has changed (ie, whether it has been
	 * moved or not). If so, it updates the internal matrices, and calls
	 * sendReports()
	 */
	virtual void callbackUpdate();

	/**
	 * sendReports checks which reportTypes are enabled, and actually performs
	 * computation for necessary reports, which are then sent out on the network
	 */
	void sendReports(reporterTarget *target);

	/**
	 * Add a target node to the report list
	 */
	void addTarget (const char *targetID);

	/**
	 * Remove a target from the report list
	 */
	void removeTarget (const char *targetID);
	
	/**
	 * This enables or disables a particular reporting type
	 */
	void setReporting(const char *type, bool enabled);
	int getReporting(const char *type) { return (int)this->reporting_[type]; }

	/**
	 * Set the maximum reporting rate (hz). Note: updates are only sent when
	 * necessary, so there is no constant reporting mode.
	 */
	void setMaxRate(float hz);
	float getMaxRate() { return maxRate_; }
	
	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();
	
private:
	
	reportingType reporting_;

	std::vector<reporterTarget> targets_;
	osg::Matrix matrix_;

	float maxRate_;

	osg::Timer_t lastTick;
};



#endif
