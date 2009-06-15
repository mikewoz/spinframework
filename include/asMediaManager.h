// ===================================================================
// Audioscape library for PureData
// Copyright (c) 2007
//
// Collaborators:
//    Shared Reality Lab (SRE), McGill University Centre for Intelligent Machines (CIM)
//       www.cim.mcgill.ca/sre
//    La Société des Arts Technologiques (SAT)
//       www.sat.qc.ca
//
// Project Directors:
//    Science - Jeremy R. Cooperstock (SRE/CIM)
//    Arts - Zack Settel
//
// Conception:
//    Zack Settel
//
// Development Team:
//    Mike Wozniewski (SRE/CIM): Researcher, Head Developer
//    Zack Settel: Artist, Researcher, Audio/DSP programming
//    Jean-Michel Dumas (SAT): Assistant Researcher
//    Mitchel Benovoy (SRE/CIM): Video Texture Programming
//    Stéphane Pelletier (SRE/CIM): Video Texture Programming
//    Pierre-Olivier Charlebois (SRE/CIM): Former Developer
//
// Funding by / Souventionné par:
//    Natural Sciences and Engineering Research Council of Canada (NSERC)
//    Canada Council for the Arts
//    NSERC/Canada Council for the Arts - New Media Initiative
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
// ===================================================================



#ifndef __ASMEDIAMANAGER_H
#define __ASMEDIAMANAGER_H

#include "asGlobals.h"


/**
 * \brief Provides an interface for finding media locations (models, textures,
 *        sounds, etc.)
 * 
 * The idea is that different mechanisms may exist for managing a project's
 * files. Examples include a networked media database, or direct access to the
 * filesystem. This class allows for a simple programmatic interface that is 
 * agnostic to the actual media storage method.
 */
class asMediaManager
{
	
	public:
		
		asMediaManager(std::string dataPath);
		~asMediaManager();
		
		std::string getDataPath() { return dataPath; }
		
		std::string getImagePath(std::string s);
		std::string getModelPath(std::string s);
		std::string getSoundPath(std::string s);
		
		std::string getImagePath(int id);
		std::string getModelPath(int id);
		std::string getSoundPath(int id);
		
		std::string getImageName(int id);
		std::string getModelName(int id);
		std::string getSoundName(int id);


	private:
		std::string dataPath;
		
		
		
};


#endif /*ASMEDIAMANAGER_H_*/
