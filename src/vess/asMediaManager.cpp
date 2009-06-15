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

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "asMediaManager.h"
#include "asUtil.h"

using namespace std;


// ***********************************************************
// constructor
asMediaManager::asMediaManager (string p)
{
	this->dataPath = p;
}

// ***********************************************************
// destructor
asMediaManager::~asMediaManager()
{
}

// ***********************************************************
string asMediaManager::getImagePath(string s)
{
	/*
	if (s=="NULL") return "";
	else return this->projectPath + "/sounds/" + s;
	*/
	
	if (s=="NULL") return "";
	else if (s.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + s.substr(1);
	} else return s;
}

string asMediaManager::getModelPath(string s)
{
	/*
	if (s=="NULL") return "";
	
	else
	{
		string f = this->projectPath + "/models/" + s + "/" + s + ".ive";
		if (!fileExists(f)) f = this->projectPath + "/models/" + s + "/" + s + ".osg";
	
		return f;
	}
	*/
	if (s=="NULL") return "";
	else if (s.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + s.substr(1);
	} else return s;

}
string asMediaManager::getSoundPath(string s)
{
	/*
	if (s=="NULL") return "";
	else return this->projectPath + "/sounds/" + s;
	*/
	
	if (s=="NULL") return "";
	else if (s.substr(0,1) == string("~")) // look for "~"
	{
		return getenv("HOME") + s.substr(1);
	} else return s;
}

// TODO: connect to database and get media given the id:
string asMediaManager::getImagePath(int id) { return ""; }
string asMediaManager::getModelPath(int id) { return ""; }
string asMediaManager::getSoundPath(int id) { return ""; }

string asMediaManager::getImageName(int id) { return "NULL"; }
string asMediaManager::getModelName(int id) { return "NULL"; }
string asMediaManager::getSoundName(int id) { return "NULL"; }

