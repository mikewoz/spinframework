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

#ifndef MeasurementNode_H_
#define MeasurementNode_H_

#include "ReferencedNode.h"

/**
 * \brief Reports geometric measurements in relation to another node
 * 
 * 
 * A targetNode must be specified, and then measurements (such as distance,
 * relative orientation, etc.) are computed and can be reported with varying
 * levels of detail.
 */
class MeasurementNode : public ReferencedNode
{

public:

	MeasurementNode(SceneManager *sceneManager, char *initID);
	virtual ~MeasurementNode();
	
	/**
	 * Level of reporting that is sent (see setReportingLevel for more details)
	 *
	 * REPORT_NONE			sends no measurements
	 *
	 * REPORT_BASIC			sends distance and absolute direction (independent) of and orientations
	 *
	 * REPORT_ANGLES		sends above info, plus relative angle of target with respect to the MeasurementNode's current position and orientation
	 *
	 * REPORT_ALL_ANGLES	sends above info, plus angles from the target's perspective
	 */
	enum reportMode { REPORT_NONE, REPORT_BASIC, REPORT_ANGLES, REPORT_ALL_ANGLES };

	/**
	 * The update callback for MeasurementNode checks to see if the target's or
	 * the MeasurementNode's global matrix has changed (ie, whether it has been
	 * moved or not). If so, it updates the internal matrices, and calls
	 * sendMeasurements()
	 */
	virtual void callbackUpdate();

	/**
	 * sendMeasurements is where the actual computation takes place, and,
	 * depending on the reportMode, the measurements are sent out on the network
	 */
	void sendMeasurements();

	/**
	 * MeasurementNode requires a targetNode to be set, which defines which node
	 * in the scene is being measured.
	 */
	void setTarget (const char *targetID);

	/**
	 * This sets the level of reporting (choose a reportMode)
	 */
	void setReportingLevel (reportMode level);
	

	const char* getTarget() { return this->targetName_->s_name; }
	int getReportingLevel() { return (int) this->reportingLevel_; }
	
	
	/**
	 * For each subclass of ReferencedNode, we override the getState() method to
	 * fill the vector with the correct set of methods for this particular node
	 */
	virtual std::vector<lo_message> getState();
	
private:
	
	t_symbol *targetName_;
	reportMode reportingLevel_;
	
	osg::Matrix thisMatrix_, targetMatrix_;
};



#endif
